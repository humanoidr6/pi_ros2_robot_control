"""Saved-map navigation panel for Linux / WSLg. Planning never arms motors."""
import fcntl
import io
import json
import math
import os
from pathlib import Path
import queue
import subprocess
import threading
import time
import tkinter as tk
from tkinter import ttk, messagebox, simpledialog
from urllib.error import HTTPError
from urllib.request import Request, urlopen
from PIL import Image, ImageTk
from safety import pixel_to_world, world_to_pixel

HOST = os.environ.get('ROBOT_SSH_HOST', 'robot@100.118.233.95')
SOCKET = str(Path.home()/'.ssh/robot-navigation-control')


def request(path, data=None, timeout=25):
    req = Request('http://127.0.0.1:18766'+path,
                  data=None if data is None else json.dumps(data).encode(),
                  headers={'Content-Type': 'application/json'})
    try:
        with urlopen(req, timeout=timeout) as response:
            raw = response.read()
    except HTTPError as exc:
        raise RuntimeError(json.loads(exc.read()).get('error', str(exc))) from exc
    return raw if path == '/map.pgm' else json.loads(raw)


def connect(root):
    if subprocess.run(['ssh', '-S', SOCKET, '-O', 'check', HOST], capture_output=True).returncode == 0:
        return True
    import pexpect
    password = simpledialog.askstring('Connect to robot', f'SSH password for {HOST}:', show='*', parent=root)
    if password is None:
        return False
    child = pexpect.spawn('ssh', ['-M', '-S', SOCKET, '-fNT', '-o', 'StrictHostKeyChecking=yes',
        '-o', 'ExitOnForwardFailure=yes', '-o', 'ServerAliveInterval=5', '-o', 'ServerAliveCountMax=2',
        '-L', '127.0.0.1:18766:127.0.0.1:18766', HOST], encoding='utf-8', timeout=20)
    try:
        if child.expect(['[Pp]assword:', pexpect.EOF]) == 0:
            child.sendline(password)
            child.expect(pexpect.EOF)
        child.close()
        if child.exitstatus != 0:
            raise RuntimeError('SSH connection failed')
        return True
    except Exception as exc:
        messagebox.showerror('Connection failed', str(exc), parent=root)
        return False


class Panel:
    def __init__(self, root):
        self.root = root
        self.events = queue.Queue()
        self.alive = True
        self.meta = None
        self.image = None
        self.pose = None
        self.scan = []
        self.target = None
        self.start = None
        self.path = []
        self.waypoints = {}
        self.scale = 1
        self.offset = (0, 0)
        root.title('Robot • Saved Map Navigation')
        root.geometry('1180x820')
        body = ttk.Frame(root, padding=16)
        body.pack(fill='both', expand=True)
        left = ttk.Frame(body, width=300)
        left.pack(side='left', fill='y', padx=(0, 16))
        ttk.Label(left, text='Saved-map navigation', font=('Sans', 18, 'bold')).pack(anchor='w', pady=8)
        self.status = tk.StringVar(value='Connecting…')
        ttk.Label(left, textvariable=self.status, wraplength=300).pack(anchor='w', pady=8)
        self.selected = tk.StringVar(value='Click the map to select a destination.')
        ttk.Label(left, textvariable=self.selected, wraplength=300).pack(anchor='w', pady=8)
        ttk.Label(left, text='Heading (degrees; 0 = map +X)').pack(anchor='w')
        self.heading = tk.StringVar(value='0')
        ttk.Entry(left, textvariable=self.heading, width=10).pack(anchor='w', pady=6)
        ttk.Button(left, text='Plan path — no movement', command=self.plan).pack(fill='x', pady=4)
        ttk.Button(left, text='Use selected point as test start', command=self.test_start).pack(fill='x', pady=4)
        ttk.Button(left, text='Use live robot pose for planning', command=self.live_start).pack(fill='x', pady=4)
        ttk.Button(left, text='Set robot pose at selected point', command=self.initial_pose).pack(fill='x', pady=(16, 4))
        ttk.Button(left, text='Confirm lidar / map alignment', command=self.confirm).pack(fill='x', pady=4)
        self.arm_button = ttk.Button(left, text='Arm navigation drive', command=lambda: self.action('/arm', {}))
        self.arm_button.state(['disabled'])
        self.arm_button.pack(fill='x', pady=4)
        self.go_button = ttk.Button(left, text='Go to selected destination', command=self.go)
        self.go_button.state(['disabled'])
        self.go_button.pack(fill='x', pady=4)
        tk.Button(left, text='STOP / CANCEL', bg='#bf2737', fg='white', font=('Sans', 14, 'bold'),
                  command=lambda: self.action('/stop', {}), height=2).pack(fill='x', pady=8)
        ttk.Label(left, text='Named destinations', font=('Sans', 12, 'bold')).pack(anchor='w', pady=(12, 4))
        self.names = ttk.Combobox(left, state='readonly')
        self.names.pack(fill='x', pady=4)
        self.names.bind('<<ComboboxSelected>>', self.choose_waypoint)
        ttk.Button(left, text='Name selected destination…', command=self.save_waypoint).pack(fill='x', pady=4)
        ttk.Label(left, text='Green: robot pose / planned path\nRed dots: live lidar returns\nBlue: selected point\nOrange: test start', wraplength=300).pack(anchor='w', pady=12)
        self.message = tk.StringVar(value='Map cleanup is separate; this panel does not edit the occupancy image.')
        ttk.Label(left, textvariable=self.message, wraplength=300).pack(anchor='w', pady=8)
        self.canvas = tk.Canvas(body, bg='#344052', highlightthickness=0)
        self.canvas.pack(side='left', fill='both', expand=True)
        self.canvas.bind('<Configure>', lambda e: self.draw())
        self.canvas.bind('<Button-1>', self.click)
        root.bind('<space>', lambda e: self.action('/stop', {}))
        root.protocol('WM_DELETE_WINDOW', self.close)
        threading.Thread(target=self.poll, daemon=True).start()
        self.pump()

    def poll(self):
        while self.alive:
            try:
                if self.image is None:
                    self.events.put(('map', (request('/map'), Image.open(io.BytesIO(request('/map.pgm'))).copy())))
                    self.events.put(('waypoints', request('/waypoints')))
                self.events.put(('status', request('/status', timeout=3)))
            except Exception as exc:
                self.events.put(('error', str(exc)))
            time.sleep(0.7)

    def action(self, path, payload):
        def work():
            try:
                result = request(path, payload)
                self.events.put(('result', result))
                if path == '/waypoint':
                    self.events.put(('waypoints', request('/waypoints')))
            except Exception as exc:
                self.events.put(('error', str(exc)))
        self.message.set('Working…')
        threading.Thread(target=work, daemon=True).start()

    def target_payload(self):
        if self.target is None:
            self.message.set('Select a point on the map first.')
            return None
        try:
            yaw = math.radians(float(self.heading.get()))
            if not math.isfinite(yaw):
                raise ValueError()
        except ValueError:
            self.message.set('Enter a finite heading in degrees.')
            return None
        return dict(x=self.target[0], y=self.target[1], yaw=yaw)

    def plan(self):
        payload = self.target_payload()
        if payload is not None:
            if self.start is not None:
                payload['start'] = dict(x=self.start[0], y=self.start[1], yaw=0)
            self.action('/plan', payload)

    def go(self):
        payload = self.target_payload()
        if payload is not None:
            self.action('/goal', payload)

    def test_start(self):
        self.start = self.target
        self.path = []
        self.message.set('Test start selected. Click another point and Plan path. This does not move or localize the robot.')
        self.draw()

    def live_start(self):
        self.start = None
        self.message.set('Plans now start from the estimated live robot pose.')
        self.draw()

    def initial_pose(self):
        payload = self.target_payload()
        if payload and messagebox.askyesno('Initialize localization', 'Is the stationary robot physically at this point and facing the entered heading? Motors will be disarmed.', parent=self.root):
            self.action('/initialpose', payload)

    def confirm(self):
        if messagebox.askyesno('Verify localization', 'Does the green robot pose match its physical position, with red lidar points aligned to the map walls?', parent=self.root):
            self.action('/confirm-localization', {})

    def save_waypoint(self):
        payload = self.target_payload()
        if payload is None:
            return
        name = simpledialog.askstring('Destination name', 'Name this point (for example Entrance or Desk):', parent=self.root)
        if name:
            payload['name'] = name
            self.action('/waypoint', payload)

    def choose_waypoint(self, _):
        value = self.waypoints[self.names.get()]
        self.target = value['x'], value['y']
        self.heading.set(str(round(math.degrees(value['yaw']), 1)))
        self.selected.set(f"{self.names.get()}: {self.target[0]:.2f}, {self.target[1]:.2f} m")
        self.draw()

    def click(self, event):
        if self.image is None:
            return
        col, row = (event.x-self.offset[0])/self.scale, (event.y-self.offset[1])/self.scale
        try:
            self.target = pixel_to_world(col, row, *self.image.size, self.meta['resolution'], self.meta['origin'])
        except ValueError:
            return
        self.path = []
        self.selected.set(f'Selected: x={self.target[0]:.2f}, y={self.target[1]:.2f} m')
        self.draw()

    def screen(self, x, y):
        col, row = world_to_pixel(x, y, self.image.height, self.meta['resolution'], self.meta['origin'])
        return col*self.scale+self.offset[0], row*self.scale+self.offset[1]

    def draw(self):
        if self.image is None:
            return
        w, h = max(1, self.canvas.winfo_width()), max(1, self.canvas.winfo_height())
        self.scale = min(w/self.image.width, h/self.image.height)
        size = max(1, int(self.image.width*self.scale)), max(1, int(self.image.height*self.scale))
        self.offset = (w-size[0])/2, (h-size[1])/2
        self.photo = ImageTk.PhotoImage(self.image.resize(size, Image.Resampling.NEAREST))
        self.canvas.delete('all')
        self.canvas.create_image(*self.offset, anchor='nw', image=self.photo)
        if len(self.path) > 1:
            points = [v for p in self.path for v in self.screen(*p)]
            self.canvas.create_line(*points, fill='#00a65a', width=3)
        for p in self.scan:
            x, y = self.screen(*p)
            self.canvas.create_oval(x-1, y-1, x+1, y+1, fill='#e53935', outline='')
        for point, color in ((self.target, '#168bff'), (self.start, '#ff9800')):
            if point:
                x, y = self.screen(*point)
                self.canvas.create_oval(x-5, y-5, x+5, y+5, fill=color, outline='white')
        if self.pose and self.pose.get('age', 999) < 1:
            x, y, yaw = self.pose['x'], self.pose['y'], self.pose['yaw']
            sx, sy = self.screen(x, y)
            ex, ey = self.screen(x+0.5*math.cos(yaw), y+0.5*math.sin(yaw))
            self.canvas.create_oval(sx-5, sy-5, sx+5, sy+5, fill='#00b86b')
            self.canvas.create_line(sx, sy, ex, ey, fill='#00b86b', width=3, arrow='last')
        for name, value in self.waypoints.items():
            x, y = self.screen(value['x'], value['y'])
            self.canvas.create_text(x+5, y, text=name, anchor='w', fill='#702090', font=('Sans', 10, 'bold'))

    def pump(self):
        try:
            while True:
                kind, value = self.events.get_nowait()
                if kind == 'map':
                    self.meta, self.image = value
                elif kind == 'status':
                    self.pose, self.scan = value['pose'], value['scan']
                    drive = value['drive']
                    mode = 'HARDWARE ENABLED' if drive.get('hardware') else 'PLANNING ONLY — motors disabled'
                    missing = ', '.join(drive.get('missing_calibration', []))
                    self.status.set(f"{mode}\nPlanner: {'ready' if value['planner_ready'] else 'waiting'}\n{drive.get('reason', '')}\n{value['goal']}" + (f'\nNeeded: {missing}' if missing else ''))
                    self.arm_button.state(['!disabled'] if drive.get('hardware') else ['disabled'])
                    self.go_button.state(['!disabled'] if drive.get('armed') else ['disabled'])
                elif kind == 'waypoints':
                    self.waypoints = value
                    self.names['values'] = sorted(value)
                elif kind == 'result':
                    if 'path' in value:
                        self.path = value['path']
                        self.message.set(f'Path preview: {len(self.path)} points. No movement commanded.')
                    else:
                        self.message.set(value.get('message', 'Done'))
                elif kind == 'error':
                    self.message.set(value)
                    self.go_button.state(['disabled'])
                    self.arm_button.state(['disabled'])
                self.draw()
        except queue.Empty:
            pass
        if self.alive:
            self.root.after(100, self.pump)

    def close(self):
        self.alive = False
        try:
            request('/stop', {}, timeout=3)
        except Exception:
            pass
        self.root.destroy()


if __name__ == '__main__':
    root = tk.Tk()
    root.withdraw()
    Path.home().joinpath('.cache').mkdir(exist_ok=True)
    instance = open(Path.home()/'.cache/robot-navigation-gui.lock', 'w')
    try:
        fcntl.flock(instance, fcntl.LOCK_EX | fcntl.LOCK_NB)
    except BlockingIOError:
        messagebox.showinfo('Robot navigation', 'The navigation panel is already open.', parent=root)
        raise SystemExit()
    if connect(root):
        root.deiconify()
        Panel(root)
        root.mainloop()
