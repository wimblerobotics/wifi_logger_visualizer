# V1 versus V2: GPS coordinates and vertical position

Date: 2026-10-27. Static code review; no live GNSS receiver testing.

## Scope and conclusion

V1 is local branch `main` at `0f93755ebf546366800a5f63de6e9c02c94d2d1e`.
V2 is the current checkout, branch `wimble_v2_gps`, at
`570da5e42bdf364ca89789a91f3c38ba8cb7be9d` (package version 2.0.0).
The branch name does not imply implemented GPS support: none is present in this V2 snapshot.
V1 references below are Git revision paths, not files in the current checkout.

**V1 has optional GNSS metadata collection and stores map-frame Z. V2 removes
both and operates exclusively on two-dimensional map cells. Neither version
implements terrain elevation or a sea-level vertical datum conversion.**

## Capability comparison

| Capability | V1 | Current V2 |
| --- | --- | --- |
| GNSS input | Optional `NavSatFix` on `/gps/filtered`; `use_gps=false` by default | No GNSS subscription or enable parameter |
| Latitude / longitude | Nullable `lat`, `lon` fields, degrees | Not stored or published |
| GNSS vertical value | Nullable `alt`, copied from message altitude, rounded to 0.1 m | Absent |
| Fix quality | Stores status and GNSS service bitmask | Absent |
| GNSS time / covariance | Neither persisted | Absent |
| Local position | Transformed odometry X/Y/Z, rounded using grid density | Latest map-to-robot TF X/Y; Z discarded |
| Spatial key | Lookup by rounded X/Y/Z | Map ID plus cell X/Y; defaults to 0.25 m cells |
| GNSS without map localization | Insufficient: valid local pose still required | Unsupported; map YAML and fresh TF required |
| Vertical visualization | Stored Z and GNSS height ignored by standard map renderers | Flat markers and browser map; no measured vertical field |
| Geographic plotting utility | Offline lat/lon-to-X/Y converter and diagnostic latitude/longitude plots | Removed |

## V1 collection path and limitations

`main:wifi_logger_visualizer/wifi_logger_node.py` declares and reads `use_gps`
(lines 31–32, 61–62), subscribes to `/gps/filtered` (132–140), and processes fixes
in `gps_callback` (597–619). Latitude and longitude are copied, and altitude is
rounded to one decimal place. An unavailable fix clears the coordinates to
`None`, status to -2, and service to 0 (557–563).

The timer requires a local pose before doing any logging (621–625). It clears
GPS metadata when the last accepted callback is over two seconds old
(627–630), but Wi-Fi logging can continue without GPS. Freshness uses callback
receipt time, not `msg.header.stamp`; a delayed old fix can therefore appear
fresh. GNSS and Wi-Fi observations are not timestamp-synchronized.

Important correctness gaps:

1. **Ordinary valid fixes are rejected.** The callback accepts only status > 0
   (605), whereas NavSatStatus defines status 0 as a valid unaugmented fix.
   Thus only augmented fixes pass the current condition.
2. **Repeated-location updates retain old GPS metadata.** `insert_data`
   (318–353) finds an existing row by X/Y/Z and updates only radio values and
   the SQL timestamp. It does not update lat/lon/alt/status/service. A new Wi-Fi
   timestamp can accompany an old or missing GNSS position. The comment about
   a one-minute update window has no corresponding age condition in the code.
3. **No coordinate quality validation.** The callback does not check finite
   values, coordinate bounds, or covariance. It does not explicitly handle
   unavailable altitude independently from usable horizontal coordinates.
4. **No separate GNSS provenance.** Receiver timestamp, frame ID, covariance,
   and vertical datum are not persisted. The GNSS antenna location is not
   transformed to the robot reference point by this logger.

V1's schema (214–264) distinguishes `z` from `alt`. The former comes from
odometry transformed into `map` (515–554), and all three local coordinates are
rounded by `grid_density` (633–635). The latter is copied from GNSS. They are
not interchangeable measurements.

The RViz text overlay can display lat/lon/alt and fix descriptions (475).
However, the standard visualizer selects only X/Y and Wi-Fi values
(`main:.../wifi_visualizer_node.py`, 202 onward). The heat mapper similarly
selects X/Y and radio values (161, 317), placing markers at display constants
rather than measured Z (225, 234). V1 therefore stores vertical information
without providing a terrain-aware visualization.

`main:database/latlon_to_xy.py` converts latitude/longitude into approximate
relative horizontal distances using a spherical Earth distance formula.
It deletes rows missing horizontal GNSS coordinates and overwrites X/Y. It
neither uses GNSS height nor reconciles the existing Z reference, and is an
explicit destructive offline utility rather than a runtime georeferencing
pipeline. The database README documents an older schema without Z/alt, so it
is not authoritative for the current V1 implementation.

## V2 collection, storage, and presentation

`wifi_logger_visualizer/wifi_logger_node.py:92–118` reads only TF translation
X/Y; `_sample` (120–159) assigns a 2D cell and publishes a 2D position in JSON.
No NavSatFix callback, GNSS cache, or `use_gps` parameter exists.

`database_manager.py:43–122` defines maps, passive cells, and surveys. There
are no latitude, longitude, GNSS height, local Z, fix status, or covariance
columns. Passive records are keyed by `(map_id, cell_x, cell_y)` and surveys by
that key plus target host. Measurements at the same map X/Y but different
heights cannot be distinguished within one map. Separate maps can separate
floors operationally, but have no defined geographic or vertical relationship.

The survey node also reads only X/Y (`survey_node.py:242` onward). Neither
passive collection nor active tests preserve vertical motion. The map origin's
third value is **yaw**, not elevation (`map_data.py:113–130`). Cell indexing
and centers are explicitly two-dimensional (139–146).

RViz places measured points at `z=0.03`, a display offset
(`wifi_visualizer_node.py:214`). The browser's API exposes existing cell/survey
columns and displays X/Y; it cannot recover geographic or vertical information
that was never collected. The status overlay has no GNSS input either.

V2 creates its own tables but provides no V1-to-V2 data conversion. Opening an
old database does not import `wifi_data` into the V2 tables. Legacy rows may
remain physically present when using the same writable database file, but V2
readers do not consume them. Retain the original V1 database for its GPS data.

## Altitude versus elevation

The ROS field name `NavSatFix.altitude` is correct. Its contract is height in
meters above the WGS 84 ellipsoid, not necessarily height above mean sea level,
and not ground elevation. V1 simply copies this value; actual compliance also
depends on the upstream publisher.

Use explicit names in any new schema:

- `gnss_ellipsoid_height_m`: message altitude, with datum documented.
- `orthometric_height_m`: a separately derived sea-level/geoid-referenced height.
- `map_z_m`: local vertical position relative to the map frame.
- `terrain_elevation_m`: ground height with an explicit vertical datum, only
  if it is measured or derived with the necessary antenna-to-ground correction.

Converting ellipsoid height to orthometric height requires geoid separation
(H = h - N), and ground elevation additionally requires accounting for antenna
height and geometry. Neither code version performs these operations. Renaming
V1 `alt` to `elevation` alone would obscure rather than fix the distinction.

Authoritative message definitions:

- [NavSatFix](https://github.com/ros2/common_interfaces/blob/rolling/sensor_msgs/msg/NavSatFix.msg)
- [NavSatStatus](https://github.com/ros2/common_interfaces/blob/rolling/sensor_msgs/msg/NavSatStatus.msg)

## Recommended direction

Restore optional GNSS as metadata without making indoor collection dependent
on a fix. Accept status >= STATUS_FIX, validate horizontal and vertical fields
independently, preserve source timestamp/frame/covariance, enforce configurable
freshness against the source time, and document association with Wi-Fi sample
time. Missing GNSS should stay nullable, not become a zero coordinate.

Choose whether GNSS represents the last sample in a cell or individual
observations. V2's latest-visit aggregates should not silently associate a
whole min/max interval with one unrelated coordinate. A separate observation
or GNSS association table preserves provenance better when history matters.

Store local Z separately from GNSS ellipsoid height. If multi-level coverage
is required, define a floor/vertical-layer identity or a 3D spatial key;
adding one Z column to a 2D overwrite key does not prevent different levels
from overwriting each other. Add map georeferencing explicitly if outdoor
geographic rendering is required; a Nav2 map origin alone is insufficient.

Provide schema migration/versioning, API and tooltip support, and tests for
status 0, stale fixes, absent/NaN height, missing GNSS, revisits, and equal X/Y
at different vertical levels. Keep datum conversion a separate, explicit
feature rather than relabeling the incoming altitude.

## Verification limits

This report is based on the pinned source revisions and message definitions.
No ROS nodes were launched and no live GNSS accuracy, antenna offset, geoid
conversion, or migration was tested. No implementation changes were made.
