"""Export a database snapshot as portable Nav2 map files and orientation.

The browser downloads a ZIP so the image, standard map YAML and hash-bound
orientation sidecar stay together. Export reads one SQLite snapshot, changes no
records, and requires no ROS nodes or source files to remain available.
"""
from datetime import datetime, timezone
import hashlib
import io
import re
import zipfile

import yaml

from wifi_logger_visualizer.orientation_data import _validated_orientation


def export_map_bundle(database, map_id, name):
    """Return ZIP bytes for a plain map basename, using a consistent DB read."""
    if not isinstance(name, str) or not re.fullmatch(r'[A-Za-z0-9][A-Za-z0-9_-]{0,99}', name):
        raise ValueError('Map name must be 1–100 letters, digits, underscores or hyphens; start with a letter or digit')
    with database.conn:
        database.conn.execute('BEGIN')
        snapshot = database.get_map(map_id)
        if snapshot is None:
            raise LookupError('Map no longer exists')
        # Detach all bytes/fields before ending the read transaction.
        snapshot = dict(snapshot)
    original = yaml.safe_load(snapshot['yaml_text']) or {}
    live = snapshot['source_yaml_path'].startswith('live:')
    settings = dict(image=name+'.pgm', mode='trinary' if live else original.get('mode', 'trinary'),
                    resolution=snapshot['resolution'],
                    origin=[snapshot['origin_x'], snapshot['origin_y'], snapshot['origin_yaw']],
                    negate=snapshot['negate'], occupied_thresh=snapshot['occupied_thresh'],
                    free_thresh=.196 if live else snapshot['free_thresh'])
    map_yaml = yaml.safe_dump(settings, sort_keys=False).encode('utf-8')
    fingerprint = hashlib.sha256(map_yaml+b'\0'+snapshot['image_data']).hexdigest()
    saved = None
    if snapshot.get('orientation_yaml'):
        try:
            saved = yaml.safe_load(snapshot['orientation_yaml'])
        except yaml.YAMLError:
            pass
    validated = _validated_orientation(saved, snapshot)
    if validated['valid'] and snapshot.get('orientation_valid') == 1:
        orientation = dict(saved['orientation'])
    else:
        orientation = dict(valid=False, reason=snapshot.get('orientation_reason')
                           or (validated.get('reason') if saved else None) or 'No captured orientation is available')
    exported_at = datetime.now(timezone.utc).isoformat()
    sidecar = dict(schema_version=1, map_file=name+'.yaml', map_content_hash=fingerprint,
                   saved_at=exported_at,
                   geometry=dict(resolution=settings['resolution'], origin=settings['origin']),
                   orientation=orientation,
                   provenance=dict(source_map_hash=snapshot['content_hash'],
                                   source_map_id=snapshot['id'], source_revision=snapshot['revision'],
                                   captured_at=snapshot['captured_at'],
                                   orientation_captured_at=snapshot.get('orientation_captured_at'),
                                   orientation_saved_at=snapshot.get('orientation_saved_at')))
    result = io.BytesIO()
    with zipfile.ZipFile(result, 'w', compression=zipfile.ZIP_DEFLATED) as archive:
        for suffix, content in (('.pgm', snapshot['image_data']), ('.yaml', map_yaml),
                                ('.orientation.yaml', yaml.safe_dump(sidecar, sort_keys=False))):
            entry = zipfile.ZipInfo(name+suffix, datetime.now().timetuple()[:6])
            entry.create_system = 3
            entry.external_attr = 0o100644 << 16
            entry.compress_type = zipfile.ZIP_DEFLATED
            archive.writestr(entry, content)
    return result.getvalue()
