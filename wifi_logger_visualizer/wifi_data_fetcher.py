from pathlib import Path
import re
import subprocess
from typing import Any, Dict, Optional


def _number(pattern: str, text: str) -> Optional[float]:
    match = re.search(pattern, text, re.MULTILINE)
    return float(match.group(1)) if match else None


def parse_iw_link(interface: str, output: str) -> Dict[str, Any]:
    disconnected = "Not connected." in output
    bssid_match = re.search(r"Connected to ([0-9a-f:]{17})", output, re.I)
    ssid_match = re.search(r"^\s*SSID:\s*(.+)$", output, re.MULTILINE)
    return {
        "connected": not disconnected and bssid_match is not None,
        "interface": interface,
        "ssid": ssid_match.group(1).strip() if ssid_match else None,
        "bssid": bssid_match.group(1).lower() if bssid_match else None,
        "frequency_mhz": _number(r"^\s*freq:\s*([\d.]+)", output),
        "signal_dbm": _number(r"^\s*signal:\s*(-?[\d.]+)\s*dBm", output),
        "tx_bitrate_mbps": _number(
            r"^\s*tx bitrate:\s*([\d.]+)\s*MBit/s", output
        ),
        "rx_bitrate_mbps": _number(
            r"^\s*rx bitrate:\s*([\d.]+)\s*MBit/s", output
        ),
    }


def parse_iw_station(output: str) -> Dict[str, Any]:
    return {
        "tx_retries": _number(r"^\s*tx retries:\s*(\d+)", output),
        "tx_failed": _number(r"^\s*tx failed:\s*(\d+)", output),
    }


class WiFiDataFetcher:
    """Read association and radio counters from Linux nl80211 via iw."""

    def __init__(self, wifi_interface: str = ""):
        self.wifi_interface = wifi_interface or self.detect_interface()

    @staticmethod
    def detect_interface() -> str:
        result = subprocess.run(
            ["iw", "dev"],
            check=True,
            capture_output=True,
            text=True,
        )
        match = re.search(r"^\s*Interface\s+(\S+)", result.stdout, re.MULTILINE)
        if match:
            return match.group(1)
        wireless = Path("/sys/class/net")
        for device in wireless.iterdir():
            if (device / "wireless").exists():
                return device.name
        raise RuntimeError("No Wi-Fi interface was found")

    def get_wifi_data(self) -> Dict[str, Any]:
        link = subprocess.run(
            ["iw", "dev", self.wifi_interface, "link"],
            check=True,
            capture_output=True,
            text=True,
        )
        sample = parse_iw_link(self.wifi_interface, link.stdout)
        if not sample["connected"]:
            sample.update({"tx_retries": None, "tx_failed": None})
            return sample

        station = subprocess.run(
            ["iw", "dev", self.wifi_interface, "station", "dump"],
            check=True,
            capture_output=True,
            text=True,
        )
        sample.update(parse_iw_station(station.stdout))
        return sample
