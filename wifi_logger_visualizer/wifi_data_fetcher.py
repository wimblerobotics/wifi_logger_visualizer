import subprocess
import re
from typing import Tuple, Optional

class WiFiDataFetcher:
    """
    Fetches WiFi data using iwconfig and parses the output to extract metrics.
    
    Attributes:
        wifi_interface (str): Name of the WiFi interface.
        min_signal_level (float): Minimum expected signal level in dBm.
        max_signal_level (float): Maximum expected signal level in dBm.
    """
    
    def __init__(self, wifi_interface: str, min_signal_level: float, max_signal_level: float):
        """
        Initialize the WiFi data fetcher.
        
        Args:
            wifi_interface (str): Name of the WiFi interface.
            min_signal_level (float): Minimum expected signal level in dBm.
            max_signal_level (float): Maximum expected signal level in dBm.
        """
        self.wifi_interface = wifi_interface
        self.min_signal_level = min_signal_level
        self.max_signal_level = max_signal_level
        self.iwconfig_output = ""
        
    def get_wifi_data(self) -> Tuple[Optional[float], Optional[float], Optional[float]]:
        """
        Fetch WiFi data using iwconfig.
        
        Returns:
            Tuple[Optional[float], Optional[float], Optional[float]]: 
                Bit rate in Mb/s, link quality as a percentage, and signal level in dBm.
                Returns None for any value that could not be retrieved.
        """
        try:
            # Get WiFi data using iwconfig
            if self.wifi_interface:
                output = subprocess.check_output(["iwconfig", self.wifi_interface]).decode("utf-8")
            else:
                output = subprocess.check_output(["iwconfig"]).decode("utf-8")
                
            # Store the raw output for later use (e.g., for display)
            self.iwconfig_output = output
            
            # Capture the WiFi interface name and store it in self.wifi_interface
            interface_match = re.search(r"^(\S+)\s+IEEE", output, re.MULTILINE)
            if interface_match:
                self.wifi_interface = interface_match.group(1)
            
            # Extract bit rate
            bit_rate_match = re.search(r"Bit Rate[=:]\s*(\d+\.?\d*)\s*(Gb/s|Mb/s)", output)
            if bit_rate_match:
                bit_rate = float(bit_rate_match.group(1))
                if bit_rate_match.group(2) == "Gb/s":
                    bit_rate *= 1000  # Convert Gb/s to Mb/s
            else:
                bit_rate = None
            
            # Extract link quality
            link_quality_match = re.search(r"Link Quality[=:]\s*(\d+)/(\d+)", output)
            link_quality = None
            if link_quality_match:
                current, max_val = int(link_quality_match.group(1)), int(link_quality_match.group(2))
                link_quality = current / max_val
            
            # Extract signal level
            signal_level_match = re.search(r"Signal level[=:]\s*(-?\d+)\s*dBm", output)
            if signal_level_match:
                signal_level = float(signal_level_match.group(1))
                # Validate signal level
                if signal_level < self.min_signal_level or signal_level > self.max_signal_level:
                    print(f"Warning: Signal level {signal_level} dBm is outside expected range")
            else:
                signal_level = None
            
            return bit_rate, link_quality, signal_level
            
        except (subprocess.CalledProcessError, FileNotFoundError) as e:
            print(f"Error fetching WiFi data: {e}")
            self.iwconfig_output = f"Error: {e}"
            return None, None, None