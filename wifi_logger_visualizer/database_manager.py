from datetime import datetime, timezone
from pathlib import Path
import sqlite3
from typing import Any, Dict, Iterable, Optional

from wifi_logger_visualizer.gps_data import GPS_COLUMNS
from wifi_logger_visualizer.orientation_data import read_orientation_snapshot


PASSIVE_FIELDS = (
    "signal_dbm",
    "tx_bitrate_mbps",
    "rx_bitrate_mbps",
    "tx_retries",
    "tx_failed",
)


def utc_now() -> str:
    return datetime.now(timezone.utc).isoformat()


class DatabaseManager:
    """Own the current, map-specific Wi-Fi estimate database."""

    def __init__(self, db_path: str, *, read_only: bool = False):
        self.db_path = str(Path(db_path).expanduser().resolve())
        if read_only:
            self.conn = sqlite3.connect(
                Path(self.db_path).as_uri() + "?mode=ro", uri=True, timeout=2.0
            )
            self.conn.row_factory = sqlite3.Row
            return
        Path(self.db_path).parent.mkdir(parents=True, exist_ok=True)
        self.conn = sqlite3.connect(self.db_path, timeout=10.0)
        self.conn.row_factory = sqlite3.Row
        self.conn.execute("PRAGMA journal_mode=WAL")
        self.conn.execute("PRAGMA foreign_keys=ON")
        self._setup_database()

    def close(self) -> None:
        self.conn.close()

    def _setup_database(self) -> None:
        self.conn.executescript(
            """
            CREATE TABLE IF NOT EXISTS collection_runs (
                id INTEGER PRIMARY KEY AUTOINCREMENT,
                started_at TEXT NOT NULL,
                map_source TEXT NOT NULL,
                wifi_source TEXT NOT NULL,
                cell_size REAL NOT NULL DEFAULT 0.25 CHECK (cell_size > 0),
                temporary INTEGER NOT NULL CHECK (temporary IN (0,1))
            );
            CREATE UNIQUE INDEX IF NOT EXISTS one_temporary_run
                ON collection_runs(temporary) WHERE temporary=1;
            CREATE TABLE IF NOT EXISTS maps (
                id INTEGER PRIMARY KEY AUTOINCREMENT,
                run_id INTEGER NOT NULL REFERENCES collection_runs(id) ON DELETE CASCADE,
                revision INTEGER NOT NULL DEFAULT 1,
                content_hash TEXT NOT NULL,
                captured_at TEXT NOT NULL,
                source_yaml_path TEXT NOT NULL,
                yaml_text TEXT NOT NULL,
                image_name TEXT NOT NULL,
                image_data BLOB NOT NULL,
                resolution REAL NOT NULL,
                origin_x REAL NOT NULL,
                origin_y REAL NOT NULL,
                origin_yaw REAL NOT NULL,
                width INTEGER NOT NULL,
                height INTEGER NOT NULL,
                negate INTEGER NOT NULL,
                occupied_thresh REAL NOT NULL,
                free_thresh REAL NOT NULL,
                orientation_yaml TEXT,
                orientation_captured_at TEXT,
                orientation_valid INTEGER CHECK (orientation_valid IN (0, 1)),
                orientation_reason TEXT,
                map_x_heading_from_east_deg REAL,
                orientation_uncertainty_deg REAL,
                orientation_saved_at TEXT,
                UNIQUE(run_id, content_hash)
            );

            CREATE TABLE IF NOT EXISTS wifi_cells (
                map_id INTEGER NOT NULL REFERENCES maps(id) ON DELETE CASCADE,
                cell_x INTEGER NOT NULL,
                cell_y INTEGER NOT NULL,
                center_x REAL NOT NULL,
                center_y REAL NOT NULL,
                visit_id TEXT NOT NULL,
                first_observed_at TEXT NOT NULL,
                last_observed_at TEXT NOT NULL,
                sample_count INTEGER NOT NULL,
                connected INTEGER NOT NULL,
                interface TEXT,
                ssid TEXT,
                bssid TEXT,
                frequency_mhz REAL,
                latitude REAL,
                longitude REAL,
                altitude REAL,
                gps_stamp_ns INTEGER,
                gps_frame_id TEXT,
                gps_status INTEGER,
                gps_service INTEGER,
                gps_covariance TEXT,
                gps_covariance_type INTEGER,
                signal_dbm_min REAL,
                signal_dbm_max REAL,
                signal_dbm_last REAL,
                tx_bitrate_mbps_min REAL,
                tx_bitrate_mbps_max REAL,
                tx_bitrate_mbps_last REAL,
                rx_bitrate_mbps_min REAL,
                rx_bitrate_mbps_max REAL,
                rx_bitrate_mbps_last REAL,
                tx_retries_min REAL,
                tx_retries_max REAL,
                tx_retries_last REAL,
                tx_failed_min REAL,
                tx_failed_max REAL,
                tx_failed_last REAL,
                PRIMARY KEY (map_id, cell_x, cell_y)
            );

            CREATE TABLE IF NOT EXISTS wifi_surveys (
                map_id INTEGER NOT NULL REFERENCES maps(id) ON DELETE CASCADE,
                cell_x INTEGER NOT NULL,
                cell_y INTEGER NOT NULL,
                measured_at TEXT NOT NULL,
                target_host TEXT NOT NULL,
                target_address TEXT,
                bssid TEXT,
                upload_mbps REAL,
                download_mbps REAL,
                idle_latency_ms REAL,
                idle_jitter_ms REAL,
                idle_packet_loss_percent REAL,
                loaded_latency_ms REAL,
                loaded_jitter_ms REAL,
                loaded_packet_loss_percent REAL,
                success INTEGER NOT NULL,
                error_text TEXT,
                PRIMARY KEY (map_id, cell_x, cell_y, target_host)
            );

            CREATE INDEX IF NOT EXISTS wifi_cells_map_time
                ON wifi_cells(map_id, last_observed_at);
            CREATE INDEX IF NOT EXISTS wifi_surveys_map_time
                ON wifi_surveys(map_id, measured_at);
            """
        )
        self.conn.commit()

    def start_run(self, map_source="saved", wifi_source="hardware", temporary=None, cell_size=.25) -> int:
        """Clean the previous disposable run exactly once, at suite startup."""
        if map_source not in ("saved", "live") or wifi_source not in ("hardware", "simulated", "topic"):
            raise ValueError("Invalid map_source or wifi_source")
        if temporary is None:
            temporary = map_source == "live" or wifi_source == "simulated"
        with self.conn:
            self.conn.execute("DELETE FROM collection_runs WHERE temporary=1")
            cursor = self.conn.execute(
                "INSERT INTO collection_runs(started_at,map_source,wifi_source,temporary,cell_size) VALUES(?,?,?,?,?)",
                (utc_now(), map_source, wifi_source, int(temporary), cell_size),
            )
        return int(cursor.lastrowid)

    def keep_run(self, run_id: int) -> bool:
        with self.conn:
            cursor = self.conn.execute("UPDATE collection_runs SET temporary=0 WHERE id=?", (run_id,))
        return cursor.rowcount == 1

    def get_run(self, run_id: int):
        row = self.conn.execute("SELECT * FROM collection_runs WHERE id=?", (run_id,)).fetchone()
        return dict(row) if row else None

    def run_map(self, run_id: int):
        row = self.conn.execute("SELECT * FROM maps WHERE run_id=? ORDER BY id DESC LIMIT 1", (run_id,)).fetchone()
        return dict(row) if row else None

    def update_live_map(self, run_id: int, map_data) -> int:
        """Replace one session raster atomically without moving stored cells."""
        run = self.get_run(run_id)
        if not run or run['map_source'] != 'live':
            raise ValueError("Live updates require a live-map run")
        current = self.run_map(run_id)
        if current is None:
            return self.register_map(map_data, run_id)
        if current['content_hash'] == map_data['content_hash']:
            return current['id']
        fields = ('content_hash', 'yaml_text', 'image_name', 'image_data', 'resolution',
                  'origin_x', 'origin_y', 'origin_yaw', 'width', 'height', 'negate',
                  'occupied_thresh', 'free_thresh')
        with self.conn:
            self.conn.execute(
                'UPDATE maps SET ' + ','.join(f'{key}=?' for key in fields) +
                ',revision=revision+1,orientation_yaml=NULL,orientation_valid=NULL,'
                'orientation_captured_at=NULL,orientation_reason=NULL,'
                'map_x_heading_from_east_deg=NULL,orientation_uncertainty_deg=NULL,'
                'orientation_saved_at=NULL WHERE id=?',
                (*[map_data[key] for key in fields], current['id']),
            )
        return current['id']

    def register_map(self, map_data: Dict[str, Any], run_id: int = 0) -> int:
        if not run_id:
            # Direct node use joins a persistent standalone run; suite launches
            # always supply a freshly coordinated run ID.
            self.conn.execute("INSERT OR IGNORE INTO collection_runs(id,started_at,map_source,wifi_source,temporary) VALUES (0, ?, 'saved', 'hardware', 0)", (utc_now(),))
            self.conn.commit()
        existing = self.conn.execute(
            "SELECT id FROM maps WHERE run_id=? AND content_hash = ?",
            (run_id, map_data["content_hash"]),
        ).fetchone()
        if existing:
            self.refresh_map_orientation(int(existing["id"]), map_data)
            return int(existing["id"])

        self.conn.execute(
            """
            INSERT OR IGNORE INTO maps (
                run_id, content_hash, captured_at, source_yaml_path, yaml_text,
                image_name, image_data, resolution, origin_x, origin_y,
                origin_yaw, width, height, negate, occupied_thresh, free_thresh
            ) VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?)
            """,
            (
                run_id,
                map_data["content_hash"],
                utc_now(),
                map_data["source_yaml_path"],
                map_data["yaml_text"],
                map_data["image_name"],
                map_data["image_data"],
                map_data["resolution"],
                map_data["origin_x"],
                map_data["origin_y"],
                map_data["origin_yaw"],
                map_data["width"],
                map_data["height"],
                map_data["negate"],
                map_data["occupied_thresh"],
                map_data["free_thresh"],
            ),
        )
        self.conn.commit()
        # Multiple collection nodes may register the same saved snapshot at
        # startup; resolve the shared row even when another writer inserted it.
        map_id = int(self.conn.execute(
            'SELECT id FROM maps WHERE run_id=? AND content_hash=?',
            (run_id, map_data['content_hash']),
        ).fetchone()['id'])
        self.refresh_map_orientation(map_id, map_data)
        return map_id

    def refresh_map_orientation(self, map_id: int, map_data=None) -> None:
        """Atomically capture sidecar text and validated display fields.

        Repeated checks avoid writes when unchanged. Missing or mismatched files
        preserve the last captured snapshot; an invalid replacement cannot erase
        an already validated orientation.
        """
        stored = self.get_map(map_id)
        if stored is None:
            return
        snapshot = read_orientation_snapshot(map_data or stored)
        if snapshot is None:
            return
        raw, result, saved_at = snapshot
        if raw == stored['orientation_yaml'] or (stored['orientation_valid'] == 1 and not result['valid']):
            return
        with self.conn:
            self.conn.execute(
                """UPDATE maps SET orientation_yaml=?, orientation_captured_at=?,
                   orientation_valid=?, orientation_reason=?, map_x_heading_from_east_deg=?,
                   orientation_uncertainty_deg=?, orientation_saved_at=? WHERE id=?""",
                (raw, utc_now(), int(result['valid']), result.get('reason', ''),
                 result.get('map_x_heading_from_east_deg'), result.get('uncertainty_deg'),
                 saved_at, map_id),
            )

    def get_map(self, map_id: int) -> Optional[Dict[str, Any]]:
        row = self.conn.execute(
            "SELECT * FROM maps WHERE id = ?", (map_id,)
        ).fetchone()
        return dict(row) if row else None

    def latest_map(self) -> Optional[Dict[str, Any]]:
        row = self.conn.execute(
            "SELECT * FROM maps ORDER BY captured_at DESC, id DESC LIMIT 1"
        ).fetchone()
        return dict(row) if row else None

    def list_maps(self) -> list[Dict[str, Any]]:
        rows = self.conn.execute(
            """
            SELECT m.id, m.run_id, m.revision, m.content_hash, m.captured_at, m.source_yaml_path,
                   m.orientation_saved_at, m.resolution, m.width, m.height, r.temporary,
                   r.map_source, r.wifi_source
            FROM maps m JOIN collection_runs r ON r.id=m.run_id
            ORDER BY m.captured_at DESC, m.id DESC
            """
        ).fetchall()
        return [dict(row) for row in rows]

    def upsert_passive(
        self,
        map_id: int,
        cell_x: int,
        cell_y: int,
        center_x: float,
        center_y: float,
        visit_id: str,
        sample: Dict[str, Any],
        observed_at: Optional[str] = None,
    ) -> None:
        observed_at = observed_at or utc_now()
        existing = self.conn.execute(
            """
            SELECT * FROM wifi_cells
            WHERE map_id = ? AND cell_x = ? AND cell_y = ?
            """,
            (map_id, cell_x, cell_y),
        ).fetchone()

        same_visit = existing is not None and existing["visit_id"] == visit_id
        values: Dict[str, Any] = {
            "map_id": map_id,
            "cell_x": cell_x,
            "cell_y": cell_y,
            "center_x": center_x,
            "center_y": center_y,
            "visit_id": visit_id,
            "first_observed_at": (
                existing["first_observed_at"] if same_visit else observed_at
            ),
            "last_observed_at": observed_at,
            "sample_count": int(existing["sample_count"]) + 1 if same_visit else 1,
            "connected": int(bool(sample.get("connected"))),
            "interface": sample.get("interface"),
            "ssid": sample.get("ssid"),
            "bssid": sample.get("bssid"),
            "frequency_mhz": sample.get("frequency_mhz"),
        }
        for field in PASSIVE_FIELDS:
            current = sample.get(field)
            previous_min = existing[f"{field}_min"] if same_visit else None
            previous_max = existing[f"{field}_max"] if same_visit else None
            present = [value for value in (previous_min, current) if value is not None]
            values[f"{field}_min"] = min(present) if present else None
            present = [value for value in (previous_max, current) if value is not None]
            values[f"{field}_max"] = max(present) if present else None
            values[f"{field}_last"] = current

        # GPS describes the latest observation, not the whole visit window.
        # Explicit NULLs prevent a stale/disabled fix surviving a new sample.
        values.update({name: sample.get(name) for name in GPS_COLUMNS})

        columns = list(values)
        placeholders = ", ".join(f":{column}" for column in columns)
        updates = ", ".join(
            f"{column}=excluded.{column}"
            for column in columns
            if column not in ("map_id", "cell_x", "cell_y")
        )
        self.conn.execute(
            f"""
            INSERT INTO wifi_cells ({", ".join(columns)})
            VALUES ({placeholders})
            ON CONFLICT(map_id, cell_x, cell_y) DO UPDATE SET {updates}
            """,
            values,
        )
        self.conn.commit()

    def upsert_survey(
        self,
        map_id: int,
        cell_x: int,
        cell_y: int,
        target_host: str,
        result: Dict[str, Any],
    ) -> None:
        values = {
            "map_id": map_id,
            "cell_x": cell_x,
            "cell_y": cell_y,
            "measured_at": result.get("measured_at", utc_now()),
            "target_host": target_host,
            "target_address": result.get("target_address"),
            "bssid": result.get("bssid"),
            "upload_mbps": result.get("upload_mbps"),
            "download_mbps": result.get("download_mbps"),
            "idle_latency_ms": result.get("idle_latency_ms"),
            "idle_jitter_ms": result.get("idle_jitter_ms"),
            "idle_packet_loss_percent": result.get("idle_packet_loss_percent"),
            "loaded_latency_ms": result.get("loaded_latency_ms"),
            "loaded_jitter_ms": result.get("loaded_jitter_ms"),
            "loaded_packet_loss_percent": result.get(
                "loaded_packet_loss_percent"
            ),
            "success": int(bool(result.get("success"))),
            "error_text": result.get("error_text"),
        }
        columns = list(values)
        updates = ", ".join(
            f"{column}=excluded.{column}"
            for column in columns
            if column not in ("map_id", "cell_x", "cell_y", "target_host")
        )
        self.conn.execute(
            f"""
            INSERT INTO wifi_surveys ({", ".join(columns)})
            VALUES ({", ".join(f":{column}" for column in columns)})
            ON CONFLICT(map_id, cell_x, cell_y, target_host)
            DO UPDATE SET {updates}
            """,
            values,
        )
        self.conn.commit()

    def get_cells(self, map_id: int) -> list[Dict[str, Any]]:
        rows = self.conn.execute(
            "SELECT * FROM wifi_cells WHERE map_id = ?",
            (map_id,),
        ).fetchall()
        return [dict(row) for row in rows]

    def get_surveys(self, map_id: int) -> list[Dict[str, Any]]:
        rows = self.conn.execute(
            "SELECT * FROM wifi_surveys WHERE map_id = ?",
            (map_id,),
        ).fetchall()
        return [dict(row) for row in rows]

    def execute_many(self, sql: str, rows: Iterable[tuple]) -> None:
        self.conn.executemany(sql, rows)
        self.conn.commit()
