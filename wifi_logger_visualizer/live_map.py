"""Convert a full ROS occupancy grid to the same portable raster as saved maps.

The logger persists at most one latest snapshot per live run. Unknown pixels
remain gray; occupied/free values follow Nav2 trinary map conventions.
"""
import hashlib
import math

import numpy as np
import yaml


def occupancy_snapshot(message, map_frame='map'):
    if message.header.frame_id.lstrip('/') != map_frame.lstrip('/'):
        raise ValueError(f'Expected occupancy grid in {map_frame}')
    info = message.info
    width, height, resolution = info.width, info.height, info.resolution
    if width <= 0 or height <= 0 or not math.isfinite(resolution) or resolution <= 0:
        raise ValueError('Invalid live map dimensions/resolution')
    data = np.asarray(message.data, dtype=np.int16)
    if data.size != width * height or np.any((data < -1) | (data > 100)):
        raise ValueError('Invalid occupancy grid pixels')
    q = info.origin.orientation
    if any(not math.isfinite(v) for v in (q.x, q.y, q.z, q.w, info.origin.position.x, info.origin.position.y)):
        raise ValueError('Invalid live map origin')
    if abs(q.x) > 1e-6 or abs(q.y) > 1e-6 or not math.isclose(q.z*q.z+q.w*q.w, 1., abs_tol=1e-4):
        raise ValueError('Live map requires a normalized planar orientation')
    yaw = 2 * math.atan2(q.z, q.w)
    # Current cell projection uses axis-aligned raster geometry.
    if abs(yaw) > 1e-6:
        raise ValueError('Rotated live map origins are not supported')
    pixels = np.full(data.shape, 205, dtype=np.uint8)
    pixels[(data >= 0) & (data <= 25)] = 254
    pixels[data >= 65] = 0
    image = f'P5\n{width} {height}\n255\n'.encode() + np.flipud(pixels.reshape(height, width)).tobytes()
    origin = [info.origin.position.x, info.origin.position.y, yaw]
    text = yaml.safe_dump(dict(image='live.pgm', resolution=resolution, origin=origin,
                               negate=0, occupied_thresh=.65, free_thresh=.25))
    return dict(content_hash=hashlib.sha256(text.encode()+b'\0'+image).hexdigest(),
                source_yaml_path='live://map', yaml_text=text, image_name='live.pgm',
                image_data=image, resolution=resolution, origin_x=origin[0], origin_y=origin[1],
                origin_yaw=yaw, width=width, height=height, negate=0, occupied_thresh=.65, free_thresh=.25)
