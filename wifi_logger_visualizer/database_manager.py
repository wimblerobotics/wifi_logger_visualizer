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
            
            # Create table with a unique constraint on x and y
            self.cursor.execute('''
                CREATE TABLE IF NOT EXISTS wifi_data (
                    timestamp TEXT NOT NULL,
                    x REAL,
                    y REAL,
                    latitude REAL,
                    longitude REAL,
                    gps_status INTEGER,
                    gps_service INTEGER,
                    bit_rate REAL,
                    link_quality REAL,
                    signal_level REAL,
                    iperf3_sender_bitrate REAL,
                    iperf3_receiver_bitrate REAL,
                    iperf3_ip TEXT,
                    PRIMARY KEY (x, y)
                )
            ''')
            self.conn.commit()
            
        except sqlite3.Error as e:
            print(f"Database setup error: {e}")
            if self.conn:
                self.conn.close()
                self.conn = None
                self.cursor = None

    def insert_data(self, x, y, latitude, longitude, gps_status, gps_service, bit_rate, link_quality, signal_level, iperf3_sender_bitrate=None, iperf3_receiver_bitrate=None, iperf3_ip=None):
        """Insert or update WiFi data in the database."""
        if not self.conn:
            self.setup_database()

        if not self.conn:
            print("Could not connect to database")
            return False

        try:
            # Check if the x, y entry already exists in the database
            self.cursor.execute(
                'SELECT x, y FROM wifi_data WHERE x = ? AND y = ?', (x, y)
            )
            existing_entry = self.cursor.fetchone()

            if existing_entry:
                # Update the existing entry with new values
                self.cursor.execute(
                    '''
                    UPDATE wifi_data
                    SET timestamp = ?, latitude = ?, longitude = ?, gps_status = ?, gps_service = ?,
                        bit_rate = ?, link_quality = ?, signal_level = ?,
                        iperf3_sender_bitrate = ?, iperf3_receiver_bitrate = ?, iperf3_ip = ?
                    WHERE x = ? AND y = ?
                    ''',
                    (datetime.now().isoformat(), latitude, longitude, gps_status, gps_service,
                     bit_rate, link_quality, signal_level,
                     iperf3_sender_bitrate, iperf3_receiver_bitrate, iperf3_ip, x, y)
                )
            else:
                # Insert a new entry if x, y does not exist
                self.cursor.execute(
                    '''
                    INSERT INTO wifi_data (timestamp, x, y, latitude, longitude, gps_status, gps_service, bit_rate, link_quality, signal_level, iperf3_sender_bitrate, iperf3_receiver_bitrate, iperf3_ip)
                    VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?)
                    ''',
                    (datetime.now().isoformat(), x, y, latitude, longitude, gps_status, gps_service, bit_rate, link_quality, signal_level, iperf3_sender_bitrate, iperf3_receiver_bitrate, iperf3_ip)
                )

            self.conn.commit()
            return True
        except sqlite3.Error as e:
            print(f"Error inserting data: {e}")
            return False

    def close(self):
        """Close the database connection."""
        if self.conn:
            self.conn.close()
            self.conn = None
            self.cursor = None