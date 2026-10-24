from datetime import datetime, timezone
from pathlib import Path
import sqlite3
from typing import Any, Dict, Iterable, Optional


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

    def __init__(self, db_path: str):
        self.db_path = str(Path(db_path).expanduser().resolve())
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
            CREATE TABLE IF NOT EXISTS maps (
                id INTEGER PRIMARY KEY,
                content_hash TEXT NOT NULL UNIQUE,
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
                free_thresh REAL NOT NULL
            );

            CREATE TABLE IF NOT EXISTS wifi_cells (
                map_id INTEGER NOT NULL REFERENCES maps(id),
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
                map_id INTEGER NOT NULL REFERENCES maps(id),
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

    def register_map(self, map_data: Dict[str, Any]) -> int:
        existing = self.conn.execute(
            "SELECT id FROM maps WHERE content_hash = ?",
            (map_data["content_hash"],),
        ).fetchone()
        if existing:
            return int(existing["id"])

        cursor = self.conn.execute(
            """
            INSERT INTO maps (
                content_hash, captured_at, source_yaml_path, yaml_text,
                image_name, image_data, resolution, origin_x, origin_y,
                origin_yaw, width, height, negate, occupied_thresh, free_thresh
            ) VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?)
            """,
            (
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
        return int(cursor.lastrowid)

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
            SELECT id, content_hash, captured_at, source_yaml_path,
                   resolution, width, height
            FROM maps ORDER BY captured_at DESC, id DESC
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
