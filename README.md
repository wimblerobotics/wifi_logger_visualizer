> **Note:** This repository is a fork of the original code by Michael Wimble ([wimblerobotics/wifi_logger_visualizer](https://github.com/wimblerobotics/wifi_logger_visualizer)), to whom all credit goes.
>
> This fork has since diverged to meet different goals. See [this pull request](https://github.com/wimblerobotics/wifi_logger_visualizer/pull/4) for details.
>
> For detailed information about Wi-Fi benchmarking, testing, and setup, see my [benchmarking notes](https://github.com/slgrobotics/robots_bringup/blob/main/Docs/Sensors/WiFi_Logger_Visualizer.md).

----------

# Wi-Fi Logger and Visualizer

A ROS 2 package for collecting map-based Wi-Fi coverage and network performance
data, with RViz visualization and a browser-based viewer.

This package targets **Ubuntu 26.04** and **ROS 2 Lyrical**. Familiarity with
ROS 2 workspaces, launch files, parameters, and coordinate frames is assumed.

**Contents:**

- [Overview](#overview)
- [Features](#features)
- [Dependencies](#dependencies)
- [Installation](#installation)
- [Architecture change — 2026-10-24](#architecture-change--2026-10-24)
- [Collection runs, simulation and live maps](#collection-runs-simulation-and-live-maps)
- [Data model](#data-model)
- [Passive monitoring](#passive-monitoring)
- [Optional GPS metadata](#optional-gps-metadata)
- [Explicit surveys](#explicit-surveys)
- [RViz](#rviz)
- [Browser viewer and snapshots](#browser-viewer-and-snapshots)
  - [Save standard map files from the database](#save-standard-map-files-from-the-database)
  - [Map orientation](#map-orientation)
  - [Access point names](#access-point-names)
- [Future work](#future-work)
- [RViz status overlay](#rviz-status-overlay)

-------------

## Overview

The package includes five ROS nodes and a standalone web server:

| Component | Purpose |
|---|---|
| `wifi_logger_node.py` | Collects passive Wi-Fi measurements at the robot's map-frame position and stores them in SQLite. |
| `survey_node.py` | Measures upload/download throughput, latency, jitter, and packet loss while the robot is stationary. |
| `wifi_visualizer_node.py` | Publishes colored cell markers for RViz, with optional highlighting of unmeasured free space. |
| `wifi_overlay_node.py` | Publishes a text overlay showing Wi-Fi status in RViz. |
| `survey_sim_node.py` | Generates repeatable synthetic passive Wi-Fi measurements from the simulated robot's position. |
| `web_viewer.py` | Serves an interactive coverage map for live viewing and post-run analysis. http://<robot IP>:8088 |

**Note:** Unlike the [V1 version](https://github.com/slgrobotics/wifi_logger_visualizer/blob/main_v1/README.md),
the V2 visualizer publishes markers rather than navigation costmaps. There is no
separate Heat Mapper node; visualization uses measured cells without spatial interpolation.

Here is Web Viewer at http://<robot IP>:8088 (sim run, live map capture):

<img alt="V2 Web Viewer" src="https://github.com/user-attachments/assets/b5f7e322-a703-4db2-96f9-974da24c09a7" />

## Features

- Passive collection of signal strength in dBm, SSID, BSSID, frequency, TX/RX
  link rates, and transmit retry/failure counters.
- Configurable map-frame cells, with minimum, maximum, and latest measurements
  from each cell's most recent visit.
- Optional stationary network surveys using `ping` and `iperf3`.
- Optional GPS coordinates, altitude, fix status, service, and covariance metadata.
- Saved-map snapshots or live occupancy grids from a growing SLAM map.
- Hardware, simulated, or externally published Wi-Fi measurements.
- SQLite storage of map images, geometry, measurements, and orientation metadata
  for analysis without the original map files.
- Temporary runs that can be promoted for retention; the next collection startup
  removes the previous unpromoted temporary run.
- Browser zoom/pan, cell details, selectable layers, fixed or Min/Max color scales,
  local timestamps, and a true-North compass when orientation is available.
- PNG coverage exports and ZIP exports containing a navigation map's PGM, YAML,
  and orientation sidecar.
- Automatic read-only web viewing during collection and optional writable
  post-run analysis with a **Promote** action.
- Nonblocking saved-map readiness checks, measurement/pose freshness checks,
  localization-recovery gating, subprocess diagnostics, and clean shutdown.

## Dependencies

The ROS, Python, build, and test dependencies are declared in `package.xml` and
installed through `rosdep`:

- ROS: `rclpy`, `tf2_ros`, `ament_index_python`, `launch`, `launch_ros`,
  `geometry_msgs`, `std_msgs`, `sensor_msgs`, `nav_msgs`, `visualization_msgs`,
  and `rviz_2d_overlay_msgs`.
- Python: NumPy, Matplotlib, and PyYAML. SQLite support is part of Python's
  standard library; the `sqlite3` command-line tool is optional.
- Hardware/network tools: `iw` and `iputils-ping`.
- Build/test tools: ament CMake/Python support, pytest, ament lint tools, and
  `rosgraph_msgs` for simulation tests.

**Install `iperf3` separately for active surveys.** The standard rosdep database
has no `iperf3` key; the `iperf` package provides the incompatible iperf2 tool.
Simulation and browser-only operation do not execute `iw`, `ping`, or `iperf3`.

Optional components include RViz 2 for marker displays,
`rviz_2d_overlay_plugins` for text overlays, and a geographic orientation
estimator such as `map_orientation_saver`. GPS requires a NavSatFix publisher;
robot localization must provide the configured map-to-robot TF transform.
Joystick nodes belong to the robot's control stack and are not required by this
package.

## Installation

Install and source ROS 2 Lyrical first. For a robot/workstation setup, install the
package on the robot for collection. A workstation can run RViz and access the
robot's viewer at `http://<robot-IP>:8088/` without running another viewer.
To analyze a copied database locally, install the package on the workstation too.
The database is written on whichever computer runs the collector.

1. Clone the repository into your ROS 2 workspace:

   ```bash
   mkdir -p ~/robot_ws/src
   cd ~/robot_ws/src
   git clone https://github.com/slgrobotics/wifi_logger_visualizer.git
   ```

2. Install dependencies. Initialize rosdep only if it has not already been
   initialized on this computer:

   ```bash
   source /opt/ros/lyrical/setup.bash
   cd ~/robot_ws
   sudo rosdep init  # First-time rosdep setup only.
   rosdep update
   rosdep install --from-paths src --ignore-src --rosdistro "$ROS_DISTRO" -y
   sudo apt install iperf3  # Required for active surveys.
   ```

3. Build the package:

   ```bash
   cd ~/robot_ws
   colcon build --packages-select wifi_logger_visualizer
   ```

4. Source the workspace in each terminal where you use the package:

   ```bash
   source ~/robot_ws/install/setup.bash
   ```

The combined launch starts the viewer on port 8088 by default. See the sections
below for saved-map, live-map, simulation, and post-run analysis examples.

----------------------------------------------------

The original README follows, updated to describe the new features.

----------------------------------------------------

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
  database. For saved maps, changing either creates a distinct coverage map
  within the run; live maps update the existing snapshot.
- Passive observations use `iw`, include BSSID/frequency/link rates, and update
  cells (25 cm by default) only while the `map -> base_link` pose is fresh and
  localization recovery is inactive.
- Each cell keeps min/max/last values for its newest visit; a revisit replaces
  the old window instead of extending a lifetime average.
- Active iperf3 and latency tests run only in explicit survey mode after the
  robot is stationary.
- RViz and the browser render measured cells without interpolation. Unknown
  free space can be highlighted explicitly rather than filled with estimated
  values.
- The browser uses the captured map, exports PNGs, and can open a consistent
  SQLite backup made while collection continues.
- The obsolete standalone heat-mapper and its duplicated aggregation path were
  removed. Passive collection, survey results, RViz, and the browser now share
  one schema.

## Collection runs, simulation and live maps

**This version requires a newly created database. There is no schema migration.**
Preserve existing databases separately before selecting a fresh path for this
version. Each suite launch creates one run and passes its ID to all collection
nodes. Source choices are launch arguments, not YAML switches:

| Argument | Choices | Default |
|---|---|---|
| `map_source` | `saved`, `live` | `saved` |
| `wifi_source` | `hardware`, `simulated`, `topic` | `hardware` |
| `temporary_run` | `auto`, `true`, `false` | `auto` |
| `map_topic` | Full `nav_msgs/OccupancyGrid` topic | `map` |
| `wifi_topic` | Wi-Fi measurement topic | `wifi/measurements` |
| `wifi_sim_seed` | Reproducible simulator seed | `42` |
| `use_sim_time` | ROS simulation clock | `false` |

`auto` marks simulation or live-map runs temporary; other runs are retained.
At the **next collection startup**, the previous temporary run and all its maps,
cells, surveys, and orientation metadata are deleted in one transaction. Shutdown,
opening the viewer, and refreshing never delete data. There is one temporary run
per database; run only one collection suite against it at a time.

During simulation and hardware collection, the suite starts a read-only viewer
at port 8088 by default (`web_viewer:=false` disables it). It never writes to the
database and has no Promote action. Temporary map choices are labeled `[temporary]`.
After stopping collection, launch the viewer with `analysis:=true` to open the
database in read/write mode. **Promote** retains the selected map's entire run, including
accumulated measurements and map snapshots; it persists immediately and is safe to retry.

```bash
ros2 launch wifi_logger_visualizer web_viewer.launch.py \
  db:=/home/sergei/robot_ws/wifi_data_v2.db analysis:=true
```

For the Python executable directly, use `--analysis` instead of `analysis:=true`.
Stop the automatic viewer first, or choose another port for analysis. Each
run records its cell size, which the viewer uses for analysis.

A fresh SLAM simulation can collect without saving any map files:

```bash
ros2 launch wifi_logger_visualizer wifi_logger_visualizer.launch.py \
  db_path:=~/launch/wifi_data_v2.db \
  map_source:=live wifi_source:=simulated use_sim_time:=true
```

For real hardware collecting against a growing SLAM map, use
`map_source:=live wifi_source:=hardware`. For a saved raster, use
`map_source:=saved map_yaml_path:=/path/to/map.yaml`. Topic input can also be
selected without starting the simulator with `wifi_source:=topic`.

In `articubot_one`, Seggy and Dragger pass these choices directly. Their
`use_sim_time:=true` defaults select simulated Wi-Fi, live maps, and disable
active surveys. Hardware defaults select hardware Wi-Fi and saved maps. Override
with `wifi_source`, `wifi_map_source`, `wifi_map_topic`, `wifi_temporary_run`,
`wifi_sim_seed`, and `wifi_active_survey`. The suite starts the web viewer in
read-only mode by default.

Live collection waits for its first valid occupancy grid and updates one map
row at most every two seconds (`map_update_interval` on the logger). Raster,
geometry, and revision change atomically; Wi-Fi cells retain map-frame positions.
Only full occupancy grids in the configured map frame with an axis-aligned
origin are accepted. Loop closures do not retrospectively relocate old measurements.
Live snapshots subscribe to `localization/map_orientation` and capture validated
true-North estimates with their full diagnostics. Missing, invalid, wrong-frame,
future, or stale estimates are recorded as unavailable. `orientation_topic` and
`orientation_max_age` (default 2 seconds) are logger parameters. Saved-map
sidecars are not imported into live runs. The initial simulator position is
captured independently of saved-map readiness.

`survey_sim_node.py` simulates **passive Wi-Fi only**, with one access point
anchored to the first fresh map-frame robot pose plus optional `ap_offset_x/y`.
It models distance-based signal strength, link rates, counters and disconnection;
noise defaults to zero. Its ROS parameters include `reference_signal_dbm`,
`path_loss_exponent`, `noise_stddev`, `disconnect_dbm`, `max_rate_mbps`, `ssid`,
`bssid`, `frequency_mhz`, `seed` and `publish_frequency`. A backward clock jump
resets the anchor and counters. It publishes neither TF nor GPS and does not write to the database.
Optional GPS continues to use the existing `use_gps` and NavSatFix path.

The measurement interface uses `std_msgs/String` containing validated JSON:
`schema_version: 1`, `stamp_ns` as a decimal ROS nanosecond string, and `sample`
containing `connected`, `interface`, `ssid`, `bssid`, `frequency_mhz`,
`signal_dbm`, `tx_bitrate_mbps`, `rx_bitrate_mbps`, `tx_retries`, and `tx_failed`.
Unavailable fields are JSON null. The logger rejects malformed/future/stale
samples, consumes each stamp once, and looks up TF at measurement time. Topic
mode never detects a host Wi-Fi interface or runs `iw`. Active ping/iperf3 surveys
are rejected for simulated/topic input; they remain available for hardware input.

## Data model

- Within a collection run, saved maps are identified by a SHA-256 digest of
  both the map YAML and image contents. The YAML and image are stored in SQLite,
  so an offline snapshot can reproduce the background map.
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

In saved-map mode, the V2 suite waits if its configured map does not exist
yet. It checks the YAML and complete PGM image every five seconds without
blocking localization or mapping. Once both are readable, enabled Wi-Fi
services start automatically; no launch restart is needed. Override the interval
with `map_check_interval:=2.0`. A map filename must still be configured so the
suite knows which save to wait for. An orientation sidecar is optional.

In saved-map mode, this is an initial startup gate, not continuous map
switching: later map changes require restarting the suite. Raw node executables
expect a readable saved map when started directly; the suite launch handles
the wait. Live-map mode instead captures occupancy-grid updates during collection.

```bash
ros2 launch wifi_logger_visualizer wifi_logger_visualizer.launch.py \
  map_yaml_path:=/absolute/path/to/map.yaml \
  monitoring:=true survey:=false visualization:=true
```

## Optional GPS metadata

Set `use_gps` in the shared `/**` YAML parameters so collection and the browser
viewer use the same setting (default: `false`):

```yaml
/**:
  ros__parameters:
    use_gps: true
/wifi_logger_node:
  ros__parameters:
    gps_topic: "/gps/filtered"
    gps_max_age: 2.0
```

GPS is disabled by default, with no `NavSatFix` subscription. When enabled,
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
timestamps and do not claim a separately measured GPS fix. V1 database import is not supported.

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
unmeasured highlighting, cell inspection, manual refresh, and a color key to the
right of the map. **Min/Max** (unchecked by default) stretches numeric colors
and legend limits to the selected map's displayed measurements. It uses each
cell's latest value for signal/link rates, rather than historical extrema.
It is disabled for access-point colors; a constant range uses one midpoint color.
Refresh recalculates the range, and PNG exports use the selected coloring.
Scroll over the map to zoom around the mouse pointer; drag with the left mouse
button to pan. Double-click or select **Fit map** to reset the view. Refresh
preserves zoom and pan; selecting another map resets them. PNG exports retain
the complete map regardless of the current view.

Optionally, launch the standalone viewer with:

```bash
ros2 launch wifi_logger_visualizer web_viewer.launch.py db:=/path/to/wifi_data_v2.db
```

The standalone launch binds to `0.0.0.0:8088` by default, so SSH forwarding is
not required when the workstation can reach the robot directly.
Optional launch arguments include `host`, `port`, `cell_size`, `use_gps`,
`analysis`, and `node_names` (the access-point names YAML file).

The standalone viewer reads `config/wifi_logger_config.yaml` by default.
Select another robot config with `config_filepath:=/path/to/config.yaml`.
It reads `db_path` and the fallback `cell_size` from the common `/**` parameters,
and `host`, `port`, and optional `node_names` from `/**/wifi_web_viewer`.
Explicit launch arguments (`db`, `host`, `port`, `cell_size`, `node_names`) override the YAML settings.

### Save standard map files from the database

Enter a **Map name** (a basename without extension, using letters, digits,
underscores, or hyphens) and select **Export ZIP**. The browser downloads
`<name>.zip` containing `<name>.pgm`, `<name>.yaml`, and
`<name>.orientation.yaml`. Extract the ZIP, then give the map YAML to Nav2's map
server and AMCL, or to this suite's saved-map mode. This is separate from **Export
PNG**, which produces a coverage visualization rather than a navigation map.

Saving works in both read-only and analysis modes. It neither writes to the
database nor promotes the run. It does not require the original map files or
ROS to be running. Live runs export standard trinary maps. The sidecar is bound
to the actual exported YAML/PGM hash; it preserves captured estimator timestamps
and includes export/source provenance. If orientation was unavailable, the
sidecar explicitly records an invalid estimate and reason. Captured orientation
is not expired merely because the completed run is old.

### Map orientation

The browser displays a true-North compass and the captured map heading and
uncertainty when valid orientation is stored in the database or supplied by a
matching `<map>.orientation.yaml` sidecar. This is independent of `use_gps`: indoor maps can also have saved orientation. The compass accounts
for the raster origin yaw and is included in exported PNGs.

`map_orientation_saver` is an optional companion package. Its estimator uses
absolute IMU heading and TF to estimate the map frame's orientation relative to
true North and publishes `localization/map_orientation`. During live-map
collection, the Wi-Fi logger captures valid, fresh estimates from that topic.
For saved maps, the companion package's map-saving wrapper writes the
`.orientation.yaml` sidecar that this suite imports. Wi-Fi collection works
without the companion package; geographic orientation is simply unavailable
unless another compatible source provides it. GPS is not required for this
orientation path.

Save the raster and sidecar with the `map_orientation_saver` package. Collection
captures the complete original sidecar YAML in the map's database row, together
with its import time, saved time, validated status/reason, true-ENU heading, and
optional uncertainty. The importer verifies the schema, map hash, and geometry.
The passive collector and survey node check every five seconds for a sidecar
saved later. Missing or mismatched files preserve previously captured metadata.
Invalid replacements also preserve an already validated snapshot.

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

## Future work

- TODO: Add an application-level probe that submits representative camera
  images to an external semantic-analysis service and records end-to-end
  response time. Network throughput alone cannot predict inference latency.

## RViz status overlay

The combined V2 launch starts `wifi_overlay_node.py` by default; use
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
YAML parameters or at runtime for each node:

```bash
ros2 param set /wifi_logger_node trace_subprocess_runs false
ros2 param set /wifi_survey_node trace_subprocess_runs false
```
