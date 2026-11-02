"""Validate saved map orientation and read portable database display fields.

The map snapshot's recorded YAML path identifies its orientation sidecar. For
portable database copies, the same sidecar basename may instead be placed next
to the SQLite file. Only metadata matching the captured map hash is accepted;
its age is not a freshness gate because it describes a saved map, not live TF.
"""
import math
from pathlib import Path

import yaml


def _finite_number(value):
    return (isinstance(value, (int, float)) and not isinstance(value, bool)
            and math.isfinite(value))


def _validated_orientation(document, map_data):
    """Validate the sidecar and derive North's clockwise image rotation."""
    if not isinstance(document, dict) or type(document.get('schema_version')) is not int or document['schema_version'] != 1:
        return dict(valid=False, reason='Unsupported orientation file format')
    if not map_data.get('content_hash') or document.get('map_content_hash') != map_data['content_hash']:
        return dict(valid=False, reason='Saved orientation does not match this map')
    orientation = document.get('orientation')
    if not isinstance(orientation, dict) or orientation.get('valid') is not True:
        reason = orientation.get('reason') if isinstance(orientation, dict) else None
        return dict(valid=False, reason=reason if isinstance(reason, str) and reason else 'Saved estimate is unavailable')
    if 'schema_version' in orientation and (type(orientation['schema_version']) is not int or orientation['schema_version'] != 1):
        return dict(valid=False, reason='Unsupported orientation estimate format')
    if orientation.get('reference') != 'true_north':
        return dict(valid=False, reason='Saved orientation is not referenced to true North')
    heading = orientation.get('map_x_heading_from_east_deg')
    yaw = map_data.get('origin_yaw', 0.)
    uncertainty = orientation.get('uncertainty_deg')
    if not _finite_number(heading) or not _finite_number(yaw):
        return dict(valid=False, reason='Saved orientation angle is invalid')
    if uncertainty is not None and (not _finite_number(uncertainty) or uncertainty < 0):
        return dict(valid=False, reason='Saved orientation uncertainty is invalid')
    north = (90. - heading) % 360.
    reported_north = orientation.get('north_yaw_in_map_deg')
    if reported_north is not None:
        if not _finite_number(reported_north) or abs((reported_north - north + 180.) % 360. - 180.) > .001:
            return dict(valid=False, reason='Saved orientation angles disagree')
    # Raster +X is rotated by origin_yaw within the map. Screen +Y points
    # downward, so a North-up arrow rotates clockwise by heading + origin_yaw.
    return dict(
        valid=True, reference='true_north',
        map_x_heading_from_east_deg=(heading + 180.) % 360. - 180.,
        north_yaw_in_map_deg=north,
        north_rotation_in_image_deg=(heading + math.degrees(yaw)) % 360.,
        uncertainty_deg=uncertainty,
        saved_at=str(document.get('saved_at', '')),
        source=str(orientation.get('source', '')),
    )


def load_saved_orientation(map_data, db_path=None):
    """Return a validated display summary or a nonfatal unavailable reason.

    Prefer normalized database fields without parsing the source schema. When
    metadata has never been imported, allow a matching external sidecar. The
    captured map hash binds that fallback to the displayed snapshot.
    """
    stored = map_data.get('orientation_valid')
    if stored is not None:
        if not stored:
            return dict(valid=False, reason=map_data.get('orientation_reason') or 'Saved estimate is unavailable')
        heading = map_data['map_x_heading_from_east_deg']
        return dict(valid=True, reference='true_north',
                    map_x_heading_from_east_deg=heading,
                    north_yaw_in_map_deg=(90. - heading) % 360.,
                    north_rotation_in_image_deg=(heading + math.degrees(map_data['origin_yaw'])) % 360.,
                    uncertainty_deg=map_data.get('orientation_uncertainty_deg'),
                    saved_at=map_data.get('orientation_saved_at') or '', source='')
    source = map_data.get('source_yaml_path')
    if not isinstance(source, str) or not source:
        return dict(valid=False, reason='No saved map path is available')
    sidecar = Path(source).expanduser().with_suffix('.orientation.yaml')
    candidates = [sidecar]
    if db_path:
        fallback = Path(db_path).expanduser().resolve().parent / sidecar.name
        if fallback != sidecar:
            candidates.append(fallback)
    result = dict(valid=False, reason='No saved orientation file found')
    for candidate in candidates:
        try:
            document = yaml.safe_load(candidate.read_text(encoding='utf-8'))
        except FileNotFoundError:
            continue
        except (OSError, UnicodeError, yaml.YAMLError):
            result = dict(valid=False, reason='Could not read the saved orientation file')
            continue
        result = _validated_orientation(document, map_data)
        if result['valid']:
            return result
    return result


def read_orientation_snapshot(map_data):
    """Read original YAML and validate its binding before database import.

    Missing files leave existing metadata intact. Unsupported or malformed files
    can be retained for diagnosis, but never become usable compass estimates.
    """
    path = Path(map_data['source_yaml_path']).with_suffix('.orientation.yaml')
    try:
        raw = path.read_text(encoding='utf-8')
    except (OSError, UnicodeError):
        return None
    try:
        document = yaml.safe_load(raw)
    except yaml.YAMLError:
        return raw, dict(valid=False, reason='Could not parse the saved orientation file'), None
    # A sidecar for an overwritten map must not replace captured metadata.
    if isinstance(document, dict) and document.get('map_content_hash') != map_data['content_hash']:
        return None
    result = _validated_orientation(document, map_data)
    geometry = document.get('geometry') if isinstance(document, dict) else None
    expected = [map_data['origin_x'], map_data['origin_y'], map_data['origin_yaw']]
    origin = geometry.get('origin') if isinstance(geometry, dict) else None
    resolution = geometry.get('resolution') if isinstance(geometry, dict) else None
    if (not _finite_number(resolution)
            or not math.isclose(resolution, map_data['resolution'], rel_tol=1e-9, abs_tol=1e-9)
            or not isinstance(origin, list) or len(origin) != 3
            or any(not _finite_number(a) or not math.isclose(a, b, rel_tol=1e-9, abs_tol=1e-9)
                   for a, b in zip(origin or [], expected))):
        if result['valid']:
            result = dict(valid=False, reason='Saved orientation geometry does not match this map')
    saved_at = document.get('saved_at') if isinstance(document, dict) else None
    return raw, result, str(saved_at) if saved_at is not None else None


def live_orientation_document(map_data, estimate, now_ns, max_age, map_frame, robot_frame):
    """Bind a fresh estimator message to a captured live map, or record why not.

    Freshness is evaluated only on collection. Export does not age out a saved
    estimate after a run ends. Original estimator diagnostics are retained.
    """
    from datetime import datetime, timezone
    from copy import deepcopy
    orientation = deepcopy(estimate) if isinstance(estimate, dict) else {}
    reason = ''
    published = orientation.get('published_at_ros_ns')
    stamp = orientation.get('stamp')
    if not orientation:
        reason = 'No live orientation estimate received'
    elif type(orientation.get('schema_version')) is not int or orientation['schema_version'] != 1:
        reason = 'Unsupported live orientation format'
    elif orientation.get('map_frame') != map_frame or orientation.get('robot_frame') != robot_frame:
        reason = 'Live orientation frames do not match collection frames'
    elif orientation.get('valid') is not True:
        reason = orientation.get('reason') or 'Live orientation estimate is unavailable'
    elif type(published) is not int or not 0 <= now_ns-published <= max_age*1e9:
        reason = 'Live orientation publication is stale or from a different clock'
    elif not _finite_number(stamp) or not 0 <= now_ns/1e9-stamp <= max_age:
        reason = 'Live orientation estimate is stale or from a different clock'
    if reason:
        orientation.update(valid=False, reason=reason)
    document = dict(schema_version=1, map_file='live.yaml',
                    map_content_hash=map_data['content_hash'],
                    saved_at=datetime.now(timezone.utc).isoformat(),
                    geometry=dict(resolution=map_data['resolution'],
                                  origin=[map_data['origin_x'], map_data['origin_y'], map_data['origin_yaw']]),
                    orientation=orientation)
    summary = _validated_orientation(document, map_data)
    if not summary['valid']:
        orientation.update(valid=False, reason=summary['reason'])
    return document
