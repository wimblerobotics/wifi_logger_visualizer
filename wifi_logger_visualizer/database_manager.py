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
            
            # Create table with fields for min, max, sum, and count for each metric
            self.cursor.execute('''
                CREATE TABLE IF NOT EXISTS wifi_data (
                    timestamp TEXT NOT NULL,
                    x REAL,
                    y REAL,
                    latitude REAL,
                    longitude REAL,
                    gps_status INTEGER,
                    gps_service INTEGER,
                    bit_rate_min REAL,
                    bit_rate_max REAL,
                    bit_rate_sum REAL,
                    bit_rate_count INTEGER,
                    link_quality_min REAL,
                    link_quality_max REAL,
                    link_quality_sum REAL,
                    link_quality_count INTEGER,
                    signal_level_min REAL,
                    signal_level_max REAL,
                    signal_level_sum REAL,
                    signal_level_count INTEGER,
                    iperf3_sender_bitrate_min REAL,
                    iperf3_sender_bitrate_max REAL,
                    iperf3_sender_bitrate_sum REAL,
                    iperf3_sender_bitrate_count INTEGER,
                    iperf3_receiver_bitrate_min REAL,
                    iperf3_receiver_bitrate_max REAL,
                    iperf3_receiver_bitrate_sum REAL,
                    iperf3_receiver_bitrate_count INTEGER,
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
                'SELECT * FROM wifi_data WHERE x = ? AND y = ?', (x, y)
            )
            existing_entry = self.cursor.fetchone()

            if existing_entry:
                # Update the existing entry with new values
                # print(f"About to update existing entry for x: {x}, y: {y}, iperf3_sender_bitrate: {iperf3_sender_bitrate}, iperf3_sender_bitrate_sum: {existing_entry[21]}, iperf3_sender_bitrate_count: {existing_entry[22]}")
                self.cursor.execute(
                    '''
                    UPDATE wifi_data
                    SET timestamp = ?, latitude = ?, longitude = ?, gps_status = ?, gps_service = ?,
                        bit_rate_min = MIN(bit_rate_min, ?),
                        bit_rate_max = MAX(bit_rate_max, ?),
                        bit_rate_sum = bit_rate_sum + ?,
                        bit_rate_count = bit_rate_count + 1,
                        link_quality_min = MIN(link_quality_min, ?),
                        link_quality_max = MAX(link_quality_max, ?),
                        link_quality_sum = link_quality_sum + ?,
                        link_quality_count = link_quality_count + 1,
                        signal_level_min = MIN(signal_level_min, ?),
                        signal_level_max = MAX(signal_level_max, ?),
                        signal_level_sum = signal_level_sum + ?,
                        signal_level_count = signal_level_count + 1,
                        iperf3_sender_bitrate_min = COALESCE(MIN(iperf3_sender_bitrate_min, ?), ?),
                        iperf3_sender_bitrate_max = COALESCE(MAX(iperf3_sender_bitrate_max, ?), ?),
                        iperf3_sender_bitrate_sum = COALESCE(iperf3_sender_bitrate_sum + ?, ?),
                        iperf3_sender_bitrate_count = COALESCE(iperf3_sender_bitrate_count + 1, ?),
                        iperf3_receiver_bitrate_min = COALESCE(MIN(iperf3_receiver_bitrate_min, ?), ?),
                        iperf3_receiver_bitrate_max = COALESCE(MAX(iperf3_receiver_bitrate_max, ?), ?),
                        iperf3_receiver_bitrate_sum = COALESCE(iperf3_receiver_bitrate_sum + ?, ?),
                        iperf3_receiver_bitrate_count = COALESCE(iperf3_receiver_bitrate_count + 1, ?),
                        iperf3_ip = COALESCE(?, iperf3_ip)
                    WHERE x = ? AND y = ?
                    ''',
                    (datetime.now().isoformat(), latitude, longitude, gps_status, gps_service,
                     bit_rate, bit_rate, bit_rate,
                     link_quality, link_quality, link_quality,
                     signal_level, signal_level, signal_level,
                     iperf3_sender_bitrate, 0.0, # Min
                     iperf3_sender_bitrate, 0.0, # Max
                     iperf3_sender_bitrate, 0.0, # Sum
                     1 if iperf3_sender_bitrate is not None else 0, # Count
                     iperf3_receiver_bitrate, 0.0, # Min
                     iperf3_receiver_bitrate, 0.0, # Max
                     iperf3_receiver_bitrate, 0.0, # Sum
                     1 if iperf3_receiver_bitrate is not None else 0, # Count
                     iperf3_ip if iperf3_ip is not None else None, x, y)
                )
                # self.cursor.execute(
                #     'SELECT * FROM wifi_data WHERE x = ? AND y = ?', (x, y)
                # )
                # new_entry = self.cursor.fetchone()
                # print(f"update  x: {x}, y: {y}, iperf3_sender_bitrate_sum: {new_entry[21]}, iperf3_sender_bitrate_count: {new_entry[22]}")
            else:
                # Insert a new entry if x, y does not exist
                self.cursor.execute(
                    '''
                    INSERT INTO wifi_data (
                        timestamp, x, y, latitude, longitude, gps_status, gps_service,
                        bit_rate_min, bit_rate_max, bit_rate_sum, bit_rate_count,
                        link_quality_min, link_quality_max, link_quality_sum, link_quality_count,
                        signal_level_min, signal_level_max, signal_level_sum, signal_level_count,
                        iperf3_sender_bitrate_min, iperf3_sender_bitrate_max, iperf3_sender_bitrate_sum, iperf3_sender_bitrate_count,
                        iperf3_receiver_bitrate_min, iperf3_receiver_bitrate_max, iperf3_receiver_bitrate_sum, iperf3_receiver_bitrate_count,
                        iperf3_ip
                    ) VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?)
                    ''',
                    (datetime.now().isoformat(), x, y, latitude, longitude, gps_status, gps_service,
                     bit_rate, bit_rate, bit_rate, 1,
                     link_quality, link_quality, link_quality, 1,
                     signal_level, signal_level, signal_level, 1,
                     iperf3_sender_bitrate if iperf3_sender_bitrate is not None else float('nan'),
                     iperf3_sender_bitrate if iperf3_sender_bitrate is not None else float('nan'),
                     iperf3_sender_bitrate if iperf3_sender_bitrate is not None else float('nan'),
                     1,
                     iperf3_receiver_bitrate if iperf3_receiver_bitrate is not None else float('nan'),
                     iperf3_receiver_bitrate if iperf3_receiver_bitrate is not None else float('nan'),
                     iperf3_receiver_bitrate if iperf3_receiver_bitrate is not None else float('nan'),
                     1,
                     iperf3_ip if iperf3_ip is not None else None)
                )
                self.cursor.execute(
                    'SELECT * FROM wifi_data WHERE x = ? AND y = ?', (x, y)
                )
                new_entry = self.cursor.fetchone()
                # print(f"new entry  x: {x}, y: {y}, iperf3_sender_bitrate_sum: {new_entry[21]}, iperf3_sender_bitrate_count: {new_entry[22]}")

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
            return False