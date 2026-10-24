# Wi-Fi Logger Visualizer TODO

The package's v2 architecture is documented in [README.md](README.md). The
legacy TODO notes below were superseded by the persistent, map-aware database
and explicit survey design.

## Remaining work

- [ ] Add an application-level AI probe that submits representative camera
      images to an external semantic-analysis service and records end-to-end
      response time.
- [ ] Exercise survey mode end-to-end on a moving robot, including stationary
      gating, upload/download tests, and failed-test persistence.
- [ ] Investigate the cause of the observed 100--145 ms latency spikes during
      Titania-to-AMDC testing (roaming, bufferbloat, or competing traffic).
- [ ] Consider moving access-point names into a shared database table if RViz
      should display friendly names as well as the browser.

## Current operating notes

- Passive monitoring uses `iw` and stores current map-cell estimates in
  `~/.ros/titania_wifi.db`.
- Survey mode is intentionally explicit because `iperf3` traffic changes the
  network being measured.
- The browser and RViz use the same database; neither interpolates unmeasured
  space.
- Start an `iperf3 -s` server on the configured target before running surveys.
