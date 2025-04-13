import subprocess
import re
from typing import Optional, Tuple

class WiFiDataFetcher:
    def __init__(self, wifi_interface: str, min_signal_strength: float, max_signal_strength: float):
        self.wifi_interface = wifi_interface
        self.min_signal_strength = min_signal_strength
        self.max_signal_strength = max_signal_strength

    def get_wifi_data(self) -> Tuple[Optional[float], Optional[float], Optional[float]]:
        """Get WiFi data with improved error handling and validation."""
        try:
            iwconfig_output = subprocess.check_output(["iwconfig", self.wifi_interface]).decode("utf-8")

            # Extract bit rate
            bit_rate_match = re.search(r"Bit Rate[:=](?P<bit_rate>\d+\.?\d*) (Mb/s|Gb/s)", iwconfig_output)
            if bit_rate_match:
                bit_rate = float(bit_rate_match.group("bit_rate"))
                if bit_rate_match.group(2) == "Gb/s":
                    bit_rate *= 1000  # Convert Gb/s to Mb/s
            else:
                bit_rate = None

            # Extract link quality
            link_quality_match = re.search(r"Link Quality=(?P<link_quality>\d+/\d+)", iwconfig_output)
            if link_quality_match:
                link_quality_str = link_quality_match.group("link_quality")
                link_quality = float(link_quality_str.split('/')[0]) / float(link_quality_str.split('/')[1])
            else:
                link_quality = None

            # Extract and validate signal level
            signal_level_match = re.search(r"Signal level[:=](?P<signal_level>-?\d+) dBm", iwconfig_output)
            if signal_level_match:
                signal_level = float(signal_level_match.group("signal_level"))
                if not (self.min_signal_strength <= signal_level <= self.max_signal_strength):
                    signal_level = None
            else:
                signal_level = None

            return bit_rate, link_quality, signal_level

        except subprocess.CalledProcessError:
            return None, None, None
        except Exception as e:
            raise RuntimeError(f"Unexpected error getting WiFi data: {e}")