# V1 GPS-to-local-coordinate investigation

Reviewed 2026-10-27. V1 reference: local `main` at
`0f93755ebf546366800a5f63de6e9c02c94d2d1e`.
Paths and line numbers in this report refer to that revision, not the current
V2 working files. This is static source analysis; no live robot tests.

## Finding

V1 offered three related mechanisms: storing GPS alongside local position,
an offline GPS-to-relative-X/Y database conversion, and diagnostic plots of
latitude/longitude over local X/Y. **It did not implement a runtime calibrated
transform between geographic coordinates and the robot's map frame.**

## 1. Runtime association: two coordinate sources in one record

`wifi_logger_visualizer/wifi_logger_node.py`:

- Lines 125–129 subscribe to `/odometry/local`.
- Lines 132–140 optionally subscribe to `/gps/filtered`.
- Lines 515–554 transform odometry position into `map` with the latest
  `map <- odom` TF, preserving X/Y/Z.
- Lines 597–619 independently cache GPS latitude, longitude, and altitude.
- Lines 621–635 require a local pose, check GPS freshness, and round local
  coordinates.
- Lines 318–353 insert both coordinate sets into a Wi-Fi row.

This creates paired measurements, not a conversion. The repository does not
calculate X/Y from the GPS callback, estimate a GPS-to-map transform, or use
GPS as a fallback when local pose is missing. An upstream localization system
could incorporate GPS into TF, but that behavior is external to this package.

The association has limitations: samples are not synchronized, GPS freshness
uses callback receipt time, and an existing X/Y/Z row update changes only radio
values and timestamp. Its original GPS coordinates are retained. Also, the
callback's `status > 0` condition rejects ordinary status-0 fixes. These issues
make historical row pairs unsuitable as unquestioned calibration control points.

## 2. Offline conversion: database/latlon_to_xy.py

This is the actual GPS-to-X/Y conversion. Its stated purpose is to recover a
consistent horizontal layout after ROS restarts reset odometry. The workflow
is documented in `database/README.md:59` onward.

Run manually from the database directory, it opens the hardcoded relative
`wifi_data.db` file (line 41), deletes rows whose lat or lon is NULL and commits
that deletion (47–57), then:

1. Computes `min(lat)` and `min(lon)` over the remaining dataset (69–74).
2. Treats their combination as a southwest geographic origin. The two minima
   need not come from the same observation.
3. Computes each row's relative coordinates (97–98):

   ```text
   x = distance(latitude, latitude, minimum_longitude, longitude)
   y = distance(minimum_latitude, latitude, longitude, longitude)
   ```

   Here `distance(lat1, lat2, lon1, lon2)` uses the haversine formula and a
   spherical Earth radius of 6371.01 km (17–39), returning meters.
4. Overwrites the existing row's X/Y values (101), then commits (105–107).

X is an approximately west-to-east distance and Y an approximately
south-to-north distance. The script also prints geographic bounds and the
approximate footprint width/height (75–80). It does not use altitude, update Z,
or preserve the original map alignment.

### What this conversion can and cannot do

For a compact outdoor dataset it can produce a useful north-aligned relative
plot, independent of odometry resets. However:

- **It is not registration to the existing ROS map.** There is no known map
  anchor, rotation estimate, or map-to-geographic transform. Replacing X/Y can
  misalign observations with the Nav2 map or costmap.
- **Its origin is data-dependent.** Adding an observation farther south or
  west changes the origin when the converter is rerun, moving existing points.
- **It is an approximation, not a defined projected coordinate system.** X
  is computed at each observation's latitude rather than in a single fixed
  tangent-plane projection. Haversine distances are nonnegative; direction
  works here because each origin component is the dataset minimum. This is
  unsuitable as-is for a fixed anchor with points on either side, dateline
  crossings, or a general geographic transform.
- **It overwrites and deletes data.** The null-coordinate deletion is committed
  before conversion, and conversion errors can leave a partially processed
  result. There is no backup step, dry run, or separate output database.
- **It has minimal validation.** No uncertainty, fix-quality, timestamp, finite
  coordinate, or empty-dataset handling is built into the conversion workflow.
- **It leaves mixed coordinate provenance.** Z remains from the prior local
  frame while X/Y now describe a new geographic-relative frame.

The V1 database README explicitly advises a backup and warns about deletion
and overwriting. It presents the converted result as a standalone heatmap,
not as proof of alignment with the robot's existing map.

## 3. Diagnostic plots

`wifi_heat_mapper/heat_mapper_lat.py:28` selects X/Y and latitude;
`wifi_heat_mapper/heat_mapper_lon.py:28` selects X/Y and longitude. Both bin by
stored local X/Y and color/annotate with the selected geographic coordinate.

They visualize how latitude or longitude varies across the recorded local
layout. They do not calculate a coordinate transform, fit control points,
change database coordinates, or provide geographic map overlays. Internal
labels referring to signal level are copied from the generic heatmap script
and do not change the values actually selected from SQL.

## Implications for V2

The useful V1 idea is preserving both coordinate systems, then optionally
relating them. The destructive conversion should not be copied into V2's live
cell tables: its X/Y values participate in map identity and cell assignment,
so moving them without re-keying cells would make those records inconsistent.

A V2-compatible follow-up would:

1. Keep TF/map coordinates and GNSS coordinates distinct, with source times,
   frames, altitude semantics, and fix quality.
2. Define a stable per-map geographic anchor and explicit geographic-to-map
   orientation. A single anchor only supplies translation if axis orientation
   is already known; arbitrary map orientation needs additional information
   or multiple reliable control points.
3. Use a documented geographic projection or local tangent frame for derived
   coordinates, retaining signed directions and altitude reference.
4. Store georeferencing metadata or separate derived output, rather than
   overwriting the original map-cell coordinates.
5. If fitting from observations, retain synchronized actual TF positions, not
   just quantized cell centers, and account for the GNSS antenna offset.
   Current V2 cell records store centers and last-sample GNSS metadata; these
   are not exact synchronized calibration pairs, and visits overwrite history.

If the intended use is GPS-only outdoor collection without an existing map,
that requires a separate coordinate-source/map-definition design. It is not
provided by the current optional GPS metadata collector.

## Reproduce the source inspection

```bash
git show main:database/latlon_to_xy.py
git show main:database/README.md
git show main:wifi_heat_mapper/heat_mapper_lat.py
git show main:wifi_heat_mapper/heat_mapper_lon.py
git show main:wifi_logger_visualizer/wifi_logger_node.py
```

No runtime or schema changes were made for this investigation.
