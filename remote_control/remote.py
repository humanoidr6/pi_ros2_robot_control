"""Desktop manual mapping remote. No password is stored."""
import io
import json
import os
import queue
import subprocess
import threading
import time
import tkinter as tk
from tkinter import messagebox, simpledialog, ttk
from urllib.request import Request, urlopen
from PIL import Image, ImageTk

HOST = os.environ.get('ROBOT_SSH_HOST', 'robot@100.118.233.95')
SOCKET = os.path.expanduser('~/.ssh/robot-mapping-control')
URL = 'http://127.0.0.1:18765'


def request(path, data=None, timeout=1):
    req = Request(URL+path, data=json.dumps(data).encode() if data is not None else None,
                  headers={'Content-Type': 'application/json'})
    with urlopen(req, timeout=timeout) as response:
        raw = response.read()
    return raw if path == '/map.pgm' else json.loads(raw)


def connect(parent):
    check = subprocess.run(['ssh', '-S', SOCKET, '-O', 'check', HOST], capture_output=True)
    if check.returncode == 0:
        return True
    import pexpect
    password = simpledialog.askstring('Robot SSH login', f'Password for {HOST}:', show='*', parent=parent)
    if password is None:
        return False
    child = pexpect.spawn('ssh', ['-M', '-S', SOCKET, '-fNT', '-o', 'StrictHostKeyChecking=yes',
        '-o', 'ExitOnForwardFailure=yes', '-o', 'ServerAliveInterval=5', '-o', 'ServerAliveCountMax=2',
        '-L', '127.0.0.1:18765:127.0.0.1:18765', HOST], encoding='utf-8', timeout=20)
    try:
        which = child.expect(['[Pp]assword:', pexpect.EOF])
        if which == 0:
            child.sendline(password)
            child.expect(pexpect.EOF)
        child.close()
        if child.exitstatus != 0:
            raise RuntimeError('SSH login or port forwarding failed')
        return True
    except Exception as exc:
        messagebox.showerror('Connection failed', str(exc), parent=parent)
        return False
    finally:
        password = None


class Remote:
    def __init__(self, root):
        self.root = root
        self.events = queue.Queue()
        self.lock = threading.Lock()
        self.target = (0, 0)
        self.heartbeat = 0
        self.alive = True
        self.enabled = True
        self.busy = False
        self.map_image = None
        self.key = None
        self.release_job = None
        root.title('Robot • Manual Floor Mapping')
        root.geometry('1120x760')
        root.minsize(900, 650)
        root.configure(bg='#101827')
        style = ttk.Style()
        style.theme_use('clam')
        style.configure('TFrame', background='#101827')
        style.configure('TLabel', background='#101827', foreground='#e5edf7', font=('Sans', 11))
        style.configure('Title.TLabel', font=('Sans', 21, 'bold'))
        style.configure('TButton', font=('Sans', 12), padding=12)
        style.configure('TCheckbutton', background='#101827', foreground='#e5edf7')
        wrap = ttk.Frame(root, padding=20)
        wrap.pack(fill='both', expand=True)
        ttk.Label(wrap, text='Manual floor mapping', style='Title.TLabel').pack(anchor='w')
        ttk.Label(wrap, text=f'Tailscale • {HOST}   |   Hold controls to drive · release to stop').pack(anchor='w', pady=(5, 15))
        body = ttk.Frame(wrap)
        body.pack(fill='both', expand=True)
        controls = ttk.Frame(body, width=320)
        controls.pack(side='left', fill='y', padx=(0, 20))
        self.status = tk.StringVar(value='Connecting to robot…')
        ttk.Label(controls, textvariable=self.status, wraplength=300).pack(anchor='w', pady=8)
        self.sensor = tk.StringVar(value='Waiting for lidar')
        ttk.Label(controls, textvariable=self.sensor, wraplength=300).pack(anchor='w', pady=8)
        pad = ttk.Frame(controls)
        pad.pack(pady=15)
        for label, move, row, col in [('↑ Forward', (1, 1), 0, 1), ('← Left', (-1, 1), 1, 0),
                                      ('Right →', (1, -1), 1, 2), ('↓ Reverse', (-1, -1), 2, 1)]:
            button = ttk.Button(pad, text=label, takefocus=False)
            button.grid(row=row, column=col, padx=2, pady=4)
            button.bind('<ButtonPress-1>', lambda e, m=move: self.start(m))
        ttk.Button(pad, text='STOP', command=self.stop).grid(row=1, column=1)
        ttk.Label(controls, text='Arrow keys / W A S D also work.\nSpace = emergency stop.').pack(anchor='w', pady=8)
        self.speed = tk.IntVar(value=100)
        self.speed_label = tk.StringVar(value='PWM: 100%')
        ttk.Label(controls, textvariable=self.speed_label).pack(anchor='w')
        tk.Scale(controls, from_=10, to=100, resolution=1, orient='horizontal', variable=self.speed,
                 command=lambda v: (self.stop(), self.speed_label.set(f'PWM: {int(float(v))}%')),
                 bg='#101827', fg='white', highlightthickness=0, length=280).pack(anchor='w')
        self.invert_l, self.invert_r = tk.BooleanVar(value=False), tk.BooleanVar(value=True)
        ttk.Label(controls, text='Wheel direction setup', font=('Sans', 11, 'bold')).pack(anchor='w', pady=(16, 5))
        ttk.Checkbutton(controls, text='Invert left wheel', variable=self.invert_l, command=self.stop).pack(anchor='w')
        ttk.Checkbutton(controls, text='Invert right wheel', variable=self.invert_r, command=self.stop).pack(anchor='w')
        ttk.Label(controls, text='If Forward spins the robot, release it.\nCheck wheel direction before continuing.', wraplength=300).pack(anchor='w', pady=8)
        tk.Button(controls, text='EMERGENCY STOP', bg='#cf3344', fg='white', activebackground='#a62230',
                  font=('Sans', 14, 'bold'), command=self.estop, height=2).pack(fill='x', pady=(12, 5))
        ttk.Button(controls, text='Reset emergency stop', command=self.reset).pack(fill='x')
        self.save_button = ttk.Button(controls, text='Save map + SLAM state', command=self.save)
        self.save_button.pack(fill='x', pady=10)
        self.canvas = tk.Canvas(body, bg='#253044', highlightthickness=0)
        self.canvas.pack(side='left', fill='both', expand=True)
        self.canvas.bind('<Configure>', lambda e: self.render_map())
        self.saved = tk.StringVar(value='Live map: white = free · black = occupied · grey = unknown. Map files save on the robot.')
        ttk.Label(wrap, textvariable=self.saved, wraplength=1000).pack(anchor='w', pady=(12, 0))
        root.bind_all('<ButtonRelease-1>', lambda e: self.stop())
        root.bind('<FocusOut>', lambda e: root.after(20, self.focus_check))
        root.bind('<Unmap>', lambda e: self.stop())
        root.bind('<KeyPress>', self.key_press)
        root.bind('<KeyRelease>', self.key_release)
        root.protocol('WM_DELETE_WINDOW', self.close)
        self.command_thread = threading.Thread(target=self.commands, daemon=True)
        self.command_thread.start()
        threading.Thread(target=self.telemetry, daemon=True).start()
        self.pump()

    def start(self, direction):
        if not self.enabled or self.busy:
            return
        self.root.focus_set()
        duty = self.speed.get()
        with self.lock:
            self.target = (direction[0]*duty*(-1 if self.invert_l.get() else 1),
                           direction[1]*duty*(-1 if self.invert_r.get() else 1))
            self.heartbeat = time.monotonic()

    def stop(self):
        with self.lock:
            self.target = (0, 0)
        self.key = None

    def focus_check(self):
        if self.root.focus_displayof() is None:
            self.stop()

    def key_press(self, event):
        if event.keysym == 'space':
            self.estop()
            return 'break'
        mapping = {'Up': (1, 1), 'w': (1, 1), 'Down': (-1, -1), 's': (-1, -1),
                   'Left': (-1, 1), 'a': (-1, 1), 'Right': (1, -1), 'd': (1, -1)}
        if event.keysym in mapping:
            if self.release_job:
                self.root.after_cancel(self.release_job)
                self.release_job = None
            self.key = event.keysym
            self.start(mapping[event.keysym])
            return 'break'

    def key_release(self, event):
        if event.keysym == self.key:
            self.release_job = self.root.after(40, self.stop)

    def commands(self):
        while self.alive:
            with self.lock:
                if time.monotonic()-self.heartbeat >= 0.25:
                    self.target = (0, 0)
                target = self.target
            try:
                if any(target):
                    request('/drive', dict(left=target[0], right=target[1]), timeout=0.25)
                else:
                    request('/stop', {}, timeout=0.25)
            except Exception:
                with self.lock:
                    self.target = (0, 0)
            time.sleep(0.08)

    def telemetry(self):
        last_map = 0
        while self.alive:
            try:
                state = request('/status')
                self.events.put(('status', state))
                if state['mapping'] and time.monotonic()-last_map > 2:
                    self.events.put(('map', Image.open(io.BytesIO(request('/map.pgm', timeout=2))).copy()))
                    last_map = time.monotonic()
            except Exception as exc:
                self.events.put(('error', 'Connection lost — motors stop automatically'))
                with self.lock:
                    self.target = (0, 0)
            time.sleep(0.5)

    def action(self, path):
        try:
            result = request(path, {}, timeout=45 if path == '/save' else 2)
            self.events.put(('saved' if path == '/save' else 'action', result))
        except Exception as exc:
            self.events.put(('action_error', str(exc)))

    def estop(self):
        self.stop()
        self.enabled = False
        self.status.set('EMERGENCY STOP')
        threading.Thread(target=self.action, args=('/estop',), daemon=True).start()

    def reset(self):
        self.stop()
        threading.Thread(target=self.action, args=('/reset',), daemon=True).start()

    def save(self):
        self.stop()
        self.busy = True
        self.save_button.state(['disabled'])
        self.saved.set('Saving map and SLAM state on robot…')
        threading.Thread(target=self.action, args=('/save',), daemon=True).start()

    def render_map(self):
        if self.map_image is None:
            return
        img = self.map_image.copy()
        img.thumbnail((max(1, self.canvas.winfo_width()-20), max(1, self.canvas.winfo_height()-20)))
        self.photo = ImageTk.PhotoImage(img)
        self.canvas.delete('all')
        self.canvas.create_image(self.canvas.winfo_width()/2, self.canvas.winfo_height()/2, image=self.photo)

    def pump(self):
        with self.lock:
            self.heartbeat = time.monotonic()
        try:
            while True:
                kind, value = self.events.get_nowait()
                if kind == 'status':
                    self.status.set(value['status'])
                    self.enabled = not value['estop']
                    near = f"{value['nearest']:.2f} m" if value['nearest'] is not None else 'unavailable'
                    self.sensor.set(f"Lidar age: {value['scan_age']:.2f} s\nNearest obstacle: {near}")
                elif kind == 'map':
                    self.map_image = value
                    self.render_map()
                elif kind == 'error':
                    self.status.set(value)
                    self.enabled = False
                elif kind in ('saved', 'action_error'):
                    self.busy = False
                    self.save_button.state(['!disabled'])
                    self.saved.set(f"Saved on robot: {value['saved']} (.yaml, .pgm" + (', .pbstream)' if value['state_saved'] else '; SLAM state failed)') if kind == 'saved' else f'Action failed: {value}')
        except queue.Empty:
            pass
        if self.alive:
            self.root.after(50, self.pump)

    def close(self):
        self.stop()
        self.alive = False
        self.command_thread.join(timeout=1)
        try:
            request('/stop', {}, timeout=1)
        except Exception:
            pass
        self.root.destroy()


if __name__ == '__main__':
    import fcntl
    os.makedirs(os.path.expanduser('~/.cache'), exist_ok=True)
    instance_lock = open(os.path.expanduser('~/.cache/robot-mapping-gui.lock'), 'w')
    root = tk.Tk()
    root.withdraw()
    try:
        fcntl.flock(instance_lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
    except BlockingIOError:
        messagebox.showinfo('Robot remote', 'Robot Manual Mapping is already open.', parent=root)
        root.destroy()
        raise SystemExit(0)
    if connect(root):
        root.deiconify()
        Remote(root)
        root.mainloop()
    else:
        root.destroy()
