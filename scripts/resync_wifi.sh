#!/bin/bash

# --- Configuration ---
WIFI_INTERFACE="wlp9s0" # Your WiFi interface name
CONNECTION_NAME="livingroom" # Your NetworkManager connection name
SIGNAL_THRESHOLD="-70" # Signal level (dBm) threshold to trigger reconnect
CHECK_INTERVAL="10" # How often to check (seconds)
# --- End Configuration ---

while true; do
    # Get current signal strength for the connected network
    SIGNAL_STRENGTH=$(nmcli -t -f SIGNAL,ACTIVE dev wifi list ifname "$WIFI_INTERFACE" | grep ':yes' | cut -d':' -f1)

    if [[ -n "$SIGNAL_STRENGTH" ]]; then
        echo "Current signal: ${SIGNAL_STRENGTH} dBm"
        if (( SIGNAL_STRENGTH < SIGNAL_THRESHOLD )); then
            echo "Signal below threshold (${SIGNAL_THRESHOLD} dBm). Forcing reconnect..."
            nmcli device disconnect "$WIFI_INTERFACE"
            sleep 2 # Give it a moment to disconnect fully
            nmcli device connect "$WIFI_INTERFACE"
            # Alternatively, use connection name:
            # nmcli connection down "$CONNECTION_NAME" && nmcli connection up "$CONNECTION_NAME"
            sleep 5 # Wait a bit after reconnecting before checking again
        fi
    else
        echo "Not connected or unable to get signal strength."
        # Optional: Try to connect if not connected
        # nmcli device connect "$WIFI_INTERFACE"
        # sleep 5
    fi

    sleep "$CHECK_INTERVAL"
done