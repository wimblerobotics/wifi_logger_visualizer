import sqlite3
import os
from datetime import datetime, timedelta

class DatabaseManager:
    def __init__(self, db_path):
        self.db_path = db_path
        self.conn = None
        self.cursor = None
        self.setup_database()

    def setup_database(self):
        """Set up the database by creating necessary tables if they don't exist."""
        try:
            # Ensure directory exists
            os.makedirs(os.path.dirname(self.db_path), exist_ok=True)
            
            # Connect to database
            self.conn = sqlite3.connect(self.db_path)
            self.cursor = self.conn.cursor()
            
            # Create table if it doesn't exist
            self.cursor.execute('''
                CREATE TABLE IF NOT EXISTS wifi_data (
                    id INTEGER PRIMARY KEY AUTOINCREMENT,
                    timestamp TEXT NOT NULL,
                    x REAL,
                    y REAL,
                    latitude REAL,
                    longitude REAL,
                    gps_status INTEGER,
                    gps_service INTEGER,
                    bit_rate REAL,
                    link_quality REAL,
                    signal_level REAL
                )
            ''')
            self.conn.commit()
            
        except sqlite3.Error as e:
            print(f"Database setup error: {e}")
            if self.conn:
                self.conn.close()
                self.conn = None
                self.cursor = None

    def insert_data(self, x, y, latitude, longitude, gps_status, gps_service, bit_rate, link_quality, signal_level):
        """Insert WiFi data into the database."""
        if not self.conn:
            self.setup_database()
            
        if not self.conn:
            print("Could not connect to database")
            return False
            
        try:
            timestamp = datetime.now().isoformat()
            self.cursor.execute(
                '''
                INSERT INTO wifi_data (timestamp, x, y, latitude, longitude, gps_status, gps_service, bit_rate, link_quality, signal_level)
                VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?)
                ''',
                (timestamp, x, y, latitude, longitude, gps_status, gps_service, bit_rate, link_quality, signal_level)
            )
            self.conn.commit()
            return True
        except sqlite3.Error as e:
            print(f"Error inserting data: {e}")
            return False

    def cleanup_old_data(self, days=30):
        """Remove data older than the specified number of days."""
        if not self.conn:
            self.setup_database()
            
        if not self.conn:
            print("Could not connect to database")
            return False
            
        try:
            cutoff_date = (datetime.now() - timedelta(days=days)).isoformat()
            self.cursor.execute("DELETE FROM wifi_data WHERE timestamp < ?", (cutoff_date,))
            self.conn.commit()
            return True
        except sqlite3.Error as e:
            print(f"Error cleaning up data: {e}")
            return False

    def close(self):
        """Close the database connection."""
        if self.conn:
            self.conn.close()
            self.conn = None
            self.cursor = None