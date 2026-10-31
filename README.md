# Wi-Fi Logger and Visualizer for ROS 2

This package maintains a current, map-associated estimate of Wi-Fi coverage for
a mobile robot. It deliberately keeps passive monitoring lightweight and runs
traffic-generating performance tests only in explicit survey mode.

## Architecture change — 2026-10-24

The package changed direction from a general GPS/odometry Wi-Fi sample logger
and plotting utility into a persistent, map-aware coverage service for Titania.
The earlier implementation accumulated lifetime averages/minima/maxima at
rounded coordinates, used `iwconfig`, ran iperf frequently, derived display
geometry from a Nav2 costmap, and produced a separate Matplotlib heatmap. That
model could preserve obsolete repeater performance indefinitely, interfere with
normal robot traffic, and visually imply coverage between measured locations.

Version 2 replaces that architecture:

- The map YAML and image contents define a map identity and are stored in the
  database. Changing either starts a distinct coverage map.
- Passive observations use `iw`, include BSSID/frequency/link rates, and update
  25 cm cells only while the `map -> base_link` pose is fresh and localization
  recovery is inactive.
- Each cell keeps min/max/last values for its newest visit; a revisit replaces
  the old window instead of extending a lifetime average.
- Active iperf and latency tests run only in explicit survey mode after the
  robot is stationary.
- RViz and the browser render measured cells without interpolation. Unknown
  free space can be highlighted explicitly rather than filled with estimated
  values.
- The browser uses the captured house map, exports PNGs, and can open a safe
  SQLite backup made while collection continues.
- The obsolete standalone heat-mapper and its duplicated aggregation path were
  removed. Passive collection, survey results, RViz, and the browser now share
  one schema.

## Data model

- Maps are identified by a SHA-256 digest of both the map YAML and image
  contents. The YAML and image are stored in SQLite, so an offline snapshot can
  reproduce the background map.
- Observations use 25 cm cells by default.
- Passive values retain minimum, maximum, and last values from the latest visit
  to a cell. A later visit replaces the previous visit rather than producing a
  lifetime average.
- Passive and active survey timestamps are independent.
- Active survey results replace the previous result for the same map, cell,
  and test host.
- There is no spatial interpolation. Unmeasured space remains unmeasured.

The default database is `~/.ros/titania_wifi.db`. SQLite WAL mode permits the
logger, RViz visualizer, and browser viewer to read it concurrently.

## Passive monitoring

The logger uses `iw`, not deprecated `iwconfig`, to capture:

- signal strength;
- SSID, BSSID, and frequency;
- negotiated transmit and receive link rates;
- cumulative transmit retry and failure counters.

It looks up `map -> base_link` directly and stops recording while
`/localization/recovery_active` is true or the transform is stale.

The V2 suite launch waits normally if its configured saved map does not exist
yet. It checks the YAML and complete PGM image every five seconds without
blocking localization or mapping. Once both are readable, enabled Wi-Fi
services start automatically; no launch restart is needed. Override the interval
with `map_check_interval:=2.0`. A map filename must still be configured so the
suite knows which save to wait for. An orientation sidecar is optional.

This is an initial startup gate, not continuous map switching: later map changes
require restarting the suite. Raw node executables expect a readable map when
started directly; the suite launch handles the waiting for them.

```bash
ros2 launch wifi_logger_visualizer wifi_logger_visualizer.launch.py \
  map_yaml_path:=/absolute/path/to/map.yaml \
  monitoring:=true survey:=false visualization:=true
```

## Optional GPS metadata

Set `use_gps` in the shared `/**` YAML parameters so collection and the browser
viewer use the same setting (default false):

```yaml
/**:
  ros__parameters:
    use_gps: true
/wifi_logger_node:
  ros__parameters:
    gps_topic: "/gps/filtered"
    gps_max_age: 2.0
```

GPS defaults to disabled, with no `NavSatFix` subscription. When enabled,
the passive logger associates the latest valid fix with each Wi-Fi observation.
The viewer shows GPS details on the Position/Cell line only when enabled.
For direct CLI use, pass `--use-gps`; the standalone viewer launch also accepts
`use_gps:=true` or `use_gps:=false` to override the shared YAML value.
Fix status 0 (ordinary fix) is accepted, along with augmented fixes. Source
timestamps must be nonzero, not in the future, and within `gps_max_age` seconds
of the logger's ROS clock; the publisher must use the same clock.

The nullable `wifi_cells` columns `latitude`, `longitude`, and `altitude`
describe the **latest passive observation**, not averages over a visit.
Latitude/longitude are degrees; altitude is meters above the WGS 84 ellipsoid,
as supplied by ROS, with no rounding or geoid conversion. Unavailable altitude
is NULL independently of valid horizontal coordinates. Provenance is stored in
`gps_stamp_ns` (source ROS timestamp), `gps_frame_id`, `gps_status`,
`gps_service`, `gps_covariance` (JSON row-major ENU matrix in square meters),
and `gps_covariance_type`. Unknown or invalid covariance is NULL with type 0.
Coordinates remain at the antenna frame; no antenna offset is applied.

Each passive update replaces this metadata, even within one visit. Missing,
invalid, stale, or disabled GPS clears it to NULL without stopping Wi-Fi
collection. Map localization is still required; GPS does not define cell
geometry or replace TF. Cell keys remain two-dimensional. Missing databases
are created with the GPS columns; existing schemas are not migrated. Recreate
an older database to use GPS collection. Survey results retain their own
timestamps and do not claim a separately measured GPS fix. GPS rendering and
V1 database import are outside this change.

## Explicit surveys

Survey mode waits until the robot has remained within 4 cm for three seconds,
then measures:

- idle round-trip latency, jitter, and packet loss;
- robot-to-server throughput;
- latency, jitter, and loss while upload traffic is active;
- server-to-robot throughput.

The target must run an iperf3 server:

```bash
iperf3 -s
```

Start a survey with:

```bash
ros2 launch wifi_logger_visualizer wifi_logger_visualizer.launch.py \
  map_yaml_path:=/absolute/path/to/map.yaml \
  monitoring:=true survey:=true target_host:=amdc
```

Survey mode also enables passive collection. Failed tests are stored explicitly
instead of being represented as zero throughput.

## RViz

`/wifi_monitor/cells` is a transient-local `MarkerArray`; adding it to RViz
restores observations from the database after restart. The default signal layer
uses fixed bounds of -90 to -35 dBm.

Change layers while running:

```bash
ros2 param set /wifi_visualizer_node metric upload
ros2 param set /wifi_visualizer_node highlight_unmeasured true
```

Supported metrics are `signal`, `tx_link_rate`, `rx_link_rate`, `upload`,
`download`, `idle_latency`, `loaded_latency`, `age`, and `access_point`.
Unmeasured highlighting covers known-free map cells only and is off by default.

## Browser viewer and snapshots

Run the lightweight HTTP service locally:

```bash
ros2 run wifi_logger_visualizer web_viewer.py \
  --db ~/.ros/titania_wifi.db --host 127.0.0.1 --port 8088
```

For remote use, keep it bound to localhost and forward it:

```bash
ssh -L 8088:127.0.0.1:8088 titania
```

Then open <http://127.0.0.1:8088>. The browser supports map/layer selection,
unmeasured highlighting, cell inspection, live refresh, and a color key to the
right of the map.
Scroll over the map to zoom around the mouse pointer; drag with the left mouse
button to pan. Double-click or select **Fit map** to reset the view. Refresh
preserves zoom and pan; selecting another map resets them. PNG exports retain
the complete map regardless of the current view.

Optionally, launch the standalone viewer with:

```bash
ros2 launch wifi_logger_visualizer web_viewer.launch.py db:=/path/to/wifi_data_v2.db
```

It binds to `0.0.0.0:8088` by default, no need to forward it via ssh.
Optional launch arguments are `host`, `port`, `cell_size`, and `node_names` (access-point names YAML).

The standalone viewer reads `config/wifi_logger_config.yaml` by default.
Select another robot config with `config_filepath:=/path/to/config.yaml`.
It reads `db_path` and `cell_size` from the common `/**` parameters, and `host`, `port`,
and optional `node_names` from `/**/wifi_web_viewer`.
Explicit launch arguments (`db`, `host`, `port`, `cell_size`, `node_names`) override the YAML settings.

### Saved map orientation

The browser displays a true-North compass and the saved map heading/uncertainty
when a matching `<map>.orientation.yaml` sidecar is available. This is independent
of `use_gps`: indoor maps can also have saved orientation. The compass accounts
for the raster origin yaw and is included in exported PNGs.

Save the raster and sidecar with the `map_orientation_saver` package. Collection
captures the complete original sidecar YAML in the map's database row, together
with its import time, saved time, validated status/reason, true-ENU heading and
optional uncertainty. The importer verifies the schema, map hash and geometry.
The passive collector and survey node check every five seconds for a sidecar
saved later.
Missing or mismatched files preserve previously captured metadata. Invalid
replacements also preserve an already validated snapshot.

The viewer uses normalized database fields, so copied databases retain their
orientation without external files. For maps with no imported metadata, it can
still read a sidecar beside the original map YAML or beside the database.
This schema applies to newly created databases; there is no migration logic.

Only schema-version-1, valid true-North estimates whose map-content hash matches
the captured YAML/image are shown. Missing, unreadable, invalid, or mismatched
metadata is displayed as unavailable; the viewer never assumes North is up.
The saved estimate is static metadata and is not rejected just because its
timestamp is old. If map files are overwritten, the sidecar must still match
the particular map snapshot selected in the database.

### Access point names

The viewer maps known mesh BSSIDs to friendly room names in the legend and the
cell tooltip; any other BSSID is labelled `Unknown Node`. The default mappings
are in `config/access_points.yaml`:

| BSSID | Name |
| --- | --- |
| `24:e5:0f:02:ac:1a` | Living Room |
| `24:e5:0f:02:b5:89` | Dining Room |
| `24:e5:0f:43:50:53` | Music Room |
| `b8:7b:d4:b9:81:8f` | Computer Room |

Use a different mapping with `--node-names /path/to/access_points.yaml`. The
file must contain an `access_points` mapping whose keys are BSSIDs.

Create a transactionally consistent offline copy instead of copying a live WAL
database directly:

```bash
ros2 run wifi_logger_visualizer web_viewer.py \
  --db ~/.ros/titania_wifi.db --snapshot ~/titania_wifi_snapshot.db
```

The snapshot can be opened by the same viewer on another computer.

## Dependencies

```bash
sudo apt install iw iperf3 python3-matplotlib python3-numpy python3-yaml
rosdep install --from-paths src --ignore-src -r -y
```

## Future work

- TODO: Add an application-level probe that submits representative camera
  images to an external semantic-analysis service and records end-to-end
  response time. Network throughput alone cannot predict inference latency.

## RViz status overlay

The V2 combined launch starts `wifi_overlay_node.py` by default; use
`overlay:=false` to disable it. The bridge converts JSON from
`wifi_monitor/status` (`std_msgs/msg/String`) to `wifi/overlay`
(`rviz_2d_overlay_msgs/msg/OverlayText`). Topics are relative to the node namespace.
In RViz, add the **OverlayText** display from `rviz_2d_overlay_plugins` and select
`/wifi/overlay` (or the namespaced equivalent). The overlay shows passive radio
link rates, not active iperf throughput.

To run the bridge separately:

```bash
ros2 run wifi_logger_visualizer wifi_overlay_node.py
```

Configure topics, placement, dimensions, font, and RGBA colors in the
`/**/wifi_overlay_node` section of `config/wifi_logger_config.yaml` or your robot
config. The robot needs `rviz_2d_overlay_msgs`; the RViz computer also needs
`rviz_2d_overlay_plugins`. Malformed status messages are skipped with a throttled
warning.

Subprocess tracing is enabled by default on the logger and survey nodes via
`trace_subprocess_runs: true`. Each `iw`, `ping`, and `iperf3` command is logged at
INFO level immediately before execution, with shell quoting to preserve argument
boundaries (commands still execute without a shell). Disable it in the common
YAML parameters or at runtime on each node:

```bash
ros2 param set /wifi_logger_node trace_subprocess_runs false
ros2 param set /wifi_survey_node trace_subprocess_runs false
```

