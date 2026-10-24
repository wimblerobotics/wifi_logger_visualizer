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

```bash
ros2 launch wifi_logger_visualizer wifi_logger_visualizer.launch.py \
  map_yaml_path:=/absolute/path/to/map.yaml \
  monitoring:=true survey:=false visualization:=true
```

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

Run the dependency-free HTTP service locally:

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

### Access point names

The viewer maps known mesh BSSIDs to friendly room names in the legend and the
cell tooltip; any other BSSID is labelled `Unknown Node`. Edit `nodeNames` in
`wifi_logger_visualizer/web_viewer.py` to match your own mesh:

| BSSID | Name |
| --- | --- |
| `24:e5:0f:02:ac:1a` | Living Room |
| `24:e5:0f:02:b5:89` | Dining Room |
| `24:e5:0f:43:50:53` | Music Room |
| `b8:7b:d4:b9:81:8f` | Computer Room |

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
