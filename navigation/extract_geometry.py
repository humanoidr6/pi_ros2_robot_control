"""Measure the supplied assembly's zero-joint collision geometry (metres).

Usage: python3 navigation/extract_geometry.py /path/to/assem1_description
Requires numpy. Does not load ros2_control or move any hardware.
"""
import argparse
import hashlib
import json
import math
from pathlib import Path
import struct
import xml.etree.ElementTree as ET
import numpy as np


def origin_matrix(origin):
    result = np.eye(4)
    if origin is None:
        return result
    result[:3, 3] = np.fromstring(origin.get('xyz', '0 0 0'), sep=' ')
    r, p, y = np.fromstring(origin.get('rpy', '0 0 0'), sep=' ')
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    result[:3, :3] = [[cy*cp, cy*sp*sr-sy*cr, cy*sp*cr+sy*sr],
                     [sy*cp, sy*sp*sr+cy*cr, sy*sp*cr-cy*sr],
                     [-sp, cp*sr, cp*cr]]
    return result


def stl_vertices(path):
    data = Path(path).read_bytes()
    if len(data) < 84:
        raise ValueError('Invalid STL: '+str(path))
    count = struct.unpack('<I', data[80:84])[0]
    if len(data) != 84+50*count:
        raise ValueError('Expected binary STL: '+str(path))
    dtype = np.dtype([('normal', '<f4', (3,)), ('vertices', '<f4', (3, 3)), ('attribute', '<u2')])
    vertices = np.frombuffer(data, offset=84, count=count, dtype=dtype)['vertices'].reshape(-1, 3).astype(float)
    if not len(vertices) or not np.isfinite(vertices).all():
        raise ValueError('Empty/nonfinite mesh: '+str(path))
    return vertices


def measure(package):
    package = Path(package).resolve()
    urdf = package/'urdf/urdf.urdf'
    tree = ET.parse(urdf).getroot()
    transforms = {'base_link': np.eye(4)}
    pending = list(tree.findall('joint'))
    while pending:
        old = len(pending)
        for joint in list(pending):
            parent = joint.find('parent').get('link')
            if parent in transforms:
                transforms[joint.find('child').get('link')] = transforms[parent] @ origin_matrix(joint.find('origin'))
                pending.remove(joint)
        if len(pending) == old:
            raise ValueError('URDF has disconnected/cyclic joints')
    vertices, links, hashes = [], {}, {str(urdf.relative_to(package)): hashlib.sha256(urdf.read_bytes()).hexdigest()}
    wheel_centres = {}
    wheel_radii = {}
    for link in tree.findall('link'):
        name = link.get('name')
        for collision in link.findall('collision'):
            mesh = collision.find('geometry/mesh')
            if mesh is None:
                raise ValueError('This extractor expects collision meshes')
            path = package/'meshes'/Path(mesh.get('filename')).name
            hashes[str(path.relative_to(package))] = hashlib.sha256(path.read_bytes()).hexdigest()
            local = stl_vertices(path) * np.fromstring(mesh.get('scale', '1 1 1'), sep=' ')
            matrix = transforms[name] @ origin_matrix(collision.find('origin'))
            world = local @ matrix[:3, :3].T + matrix[:3, 3]
            vertices.append(world)
            links[name] = dict(min_m=world.min(0).tolist(), max_m=world.max(0).tolist())
            if name in ('Part9_1', 'Part9_2'):
                # Supplied wheel meshes rotate about local Y. Outer tread,
                # unlike the joint origin, is offset +/-3 mm along that axis.
                radii = np.linalg.norm(local[:, [0, 2]], axis=1)
                tread = local[radii > radii.max()*0.98]
                centre = np.array([0., (tread[:, 1].min()+tread[:, 1].max())/2, 0.])
                wheel_centres[name] = matrix[:3, :3] @ centre + matrix[:3, 3]
                wheel_radii[name] = float(radii.max())
    all_vertices = np.vstack(vertices)
    lo, hi = all_vertices.min(0), all_vertices.max(0)
    axle = (wheel_centres['Part9_1']+wheel_centres['Part9_2'])/2
    footprint = [[float(x-axle[0]), float(y-axle[1])] for x, y in
                 ((lo[0], lo[1]), (hi[0], lo[1]), (hi[0], hi[1]), (lo[0], hi[1]))]
    return dict(source_package=str(package), pose='all joints zero; collision STL geometry', units='metres',
        length_m=float(hi[0]-lo[0]), width_m=float(hi[1]-lo[1]), height_m=float(hi[2]-lo[2]),
        bounds_in_cad_base_link_m=[lo.tolist(), hi.tolist()],
        wheel_centres_in_cad_base_link_m={k: v.tolist() for k, v in wheel_centres.items()},
        wheel_radii_m=wheel_radii,
        tyre_centre_separation_m=float(abs(wheel_centres['Part9_1'][1]-wheel_centres['Part9_2'][1])),
        wheel_joint_origin_separation_m=float(abs(transforms['Part9_1'][1, 3]-transforms['Part9_2'][1, 3])),
        axle_midpoint_in_cad_base_link_m=axle.tolist(),
        footprint_about_axle_midpoint_m=footprint,
        circumscribed_radius_about_axle_m=max(math.hypot(*p) for p in footprint),
        link_bounds=links, source_sha256=hashes)


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('package')
    parser.add_argument('--output')
    args = parser.parse_args()
    output = json.dumps(measure(args.package), indent=2)+'\n'
    if args.output:
        Path(args.output).write_text(output)
    else:
        print(output, end='')
