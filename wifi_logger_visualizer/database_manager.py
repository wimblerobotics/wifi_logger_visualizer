import sqlite3
from typing import Optional
import os

class DatabaseManager:
    def __init__(self, db_path: str):
        self.db_path = db_path
        self.create_table()

    def create_table(self):
        """Create the database table with proper constraints and indices."""
        try:
            conn = sqlite3.connect(self.db_path)
            cursor = conn.cursor()
            
            # Create table with proper constraints
            cursor.execute("""
                CREATE TABLE IF NOT EXISTS wifi_data (
                    id INTEGER PRIMARY KEY AUTOINCREMENT,
                    timestamp DATETIME DEFAULT CURRENT_TIMESTAMP,
                    x REAL NOT NULL,
                    y REAL NOT NULL,
                    lat REAL NULL,
                    lon REAL NULL,
                    gps_status INT NULL,
                    gps_service INT NULL,
                    bit_rate REAL CHECK (bit_rate >= 0),
                    link_quality REAL CHECK (link_quality >= 0 AND link_quality <= 1),
                    signal_level REAL CHECK (signal_level >= -90.0 AND signal_level <= -30.0),
                    UNIQUE(x, y, timestamp)
                )
            """)
            
            # Create indices for better query performance
            cursor.execute("CREATE INDEX IF NOT EXISTS idx_timestamp ON wifi_data(timestamp)")
            cursor.execute("CREATE INDEX IF NOT EXISTS idx_coordinates ON wifi_data(x, y)")
            
            conn.commit()
            conn.close()
        except sqlite3.Error as e:
            raise RuntimeError(f"Error creating table: {e}")

    def insert_data(self, x: float, y: float, lat: Optional[float], lon: Optional[float], gps_status: int,
                    gps_service: int, bit_rate: float, link_quality: float, signal_level: float):
        """Insert WiFi data with validation and error handling."""
        try:
            conn = sqlite3.connect(self.db_path)
            cursor = conn.cursor()
            
            # Check if we already have data for this location
            cursor.execute("""
                SELECT id FROM wifi_data 
                WHERE x = ? AND y = ? 
                ORDER BY timestamp DESC LIMIT 1
            """, (x, y))
            
            existing_record = cursor.fetchone()
            
            if existing_record:
                # Update existing record if it's within the last minute
                cursor.execute("""
                    UPDATE wifi_data 
                    SET bit_rate = ?, link_quality = ?, signal_level = ?, timestamp = CURRENT_TIMESTAMP
                    WHERE id = ?
                """, (bit_rate, link_quality, signal_level, existing_record[0]))
            else:
                # Insert new record
                cursor.execute("""
                    INSERT INTO wifi_data (x, y, lat, lon, gps_status, gps_service, bit_rate, link_quality, signal_level)
                    VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?)
                """, (x, y, lat, lon, gps_status, gps_service, bit_rate, link_quality, signal_level))
            conn.commit()
            conn.close()
        except sqlite3.Error as e:
            raise RuntimeError(f"Error inserting data: {e}")

    def cleanup_old_data(self, max_age_days: int = 30):
        """Remove data older than specified days."""
        try:
            conn = sqlite3.connect(self.db_path)
            cursor = conn.cursor()
            
            cursor.execute("""
                DELETE FROM wifi_data 
                WHERE timestamp < datetime('now', '-' || ? || ' days')
            """, (max_age_days,))
            
            conn.commit()
            conn.close()
        except sqlite3.Error as e:
            raise RuntimeError(f"Error cleaning up old data: {e}")