"""Versioned, timestamped Wi-Fi topic contract shared by simulator and logger.

Messages use std_msgs/String JSON, avoiding a separate ROS interface package.
Only validated radio fields reach storage; GPS continues on NavSatFix. Stamps
are decimal ROS nanoseconds, represented as strings to preserve precision.
"""
import json
import math
import re


RADIO_FIELDS = ('connected', 'interface', 'ssid', 'bssid', 'frequency_mhz',
                'signal_dbm', 'tx_bitrate_mbps', 'rx_bitrate_mbps', 'tx_retries', 'tx_failed')


def encode_measurement(stamp_ns, sample):
    return json.dumps(dict(schema_version=1, stamp_ns=str(stamp_ns),
                           sample={key: sample.get(key) for key in RADIO_FIELDS}), allow_nan=False)


def decode_measurement(text):
    data = json.loads(text)
    if not isinstance(data, dict) or type(data.get('schema_version')) is not int or data['schema_version'] != 1:
        raise ValueError('Unsupported Wi-Fi measurement schema')
    stamp = data.get('stamp_ns')
    if not isinstance(stamp, str) or not stamp.isdecimal() or int(stamp) <= 0:
        raise ValueError('Invalid Wi-Fi measurement stamp')
    sample = data.get('sample')
    if not isinstance(sample, dict) or set(sample) != set(RADIO_FIELDS) or type(sample['connected']) is not bool:
        raise ValueError('Invalid Wi-Fi radio fields')
    for key in ('interface', 'ssid', 'bssid'):
        if sample[key] is not None and not isinstance(sample[key], str):
            raise ValueError(f'Invalid {key}')
    if sample['bssid'] is not None and not re.fullmatch(r'[0-9a-fA-F]{2}(?::[0-9a-fA-F]{2}){5}', sample['bssid']):
        raise ValueError('Invalid BSSID')
    for key in RADIO_FIELDS[4:]:
        value = sample[key]
        if value is not None and (type(value) not in (int, float) or not math.isfinite(value)
                                  or (key != 'signal_dbm' and value < 0)):
            raise ValueError(f'Invalid {key}')
    if sample['connected'] and (sample['bssid'] is None or sample['signal_dbm'] is None):
        raise ValueError('Connected measurements require BSSID and signal')
    return int(stamp), sample
