TODO: Use configuration.yaml file
TODO: Update costmap when timestamp changes
TODO: Explain how to display the costmaps in rviz2
TODO: Heat map to check for db updates
TODO: Average readings
TODO: Add iperf3 -C amdc.local --bidir to capture
TODO: In Sigyn, test minimal throuput and warn -- use the MinMaxCur also.
TODO: In Sigyn, use MinMaxCur for both battery and bidir speed

```bash
sudo nano /etc/systemd/system/iperf3.service
[Unit]
Description=iperf3 network testing server
After=network.target

[Service]
ExecStart=/usr/bin/iperf3 -s
Restart=always
User=ros

[Install]
WantedBy=multi-user.target

sudo systemctl daemon-reload
sudo systemctl enable iperf3
sudo systemctl start iperf3

On robot:
iperf3 -c <server_ip> --bidir
```
Before backhaul update to Nest Wifi Pro
ros@sigyn7900:~/wifi_logger_visualizer_ws$ iperf3 -c amdc.local
Connecting to host amdc.local, port 5201
[  5] local 192.168.86.156 port 50664 connected to 192.168.86.155 port 5201
[ ID] Interval           Transfer     Bitrate         Retr  Cwnd
[  5]   0.00-1.00   sec  20.5 MBytes   171 Mbits/sec    0   1.08 MBytes       
[  5]   1.00-2.00   sec  28.5 MBytes   240 Mbits/sec    0   2.18 MBytes       
[  5]   2.00-3.00   sec  26.2 MBytes   220 Mbits/sec    0   2.90 MBytes       
[  5]   3.00-4.00   sec  27.8 MBytes   233 Mbits/sec    0   3.27 MBytes       


After backup update
os@sigyn7900:~/wifi_logger_visualizer_ws$ iperf3 -c amdc.local --bidir
Connecting to host amdc.local, port 5201
[  5] local 192.168.86.156 port 50560 connected to 192.168.86.155 port 5201
[  7] local 192.168.86.156 port 50566 connected to 192.168.86.155 port 5201
[ ID][Role] Interval           Transfer     Bitrate         Retr  Cwnd
[  5][TX-C]   0.00-1.01   sec  15.2 MBytes   127 Mbits/sec    0    786 KBytes       
[  7][RX-C]   0.00-1.01   sec  6.75 MBytes  56.3 Mbits/sec                  
[  5][TX-C]   1.01-2.00   sec  12.6 MBytes   106 Mbits/sec    0   1.20 MBytes       
[  7][RX-C]   1.01-2.00   sec  11.9 MBytes  99.6 Mbits/sec                  
[  5][TX-C]   2.00-3.00   sec  9.50 MBytes  79.7 Mbits/sec    0   1.48 MBytes       
[  7][RX-C]   2.00-3.00   sec  8.25 MBytes  69.2 Mbits/sec                  
[  5][TX-C]   3.00-4.00   sec  11.2 MBytes  94.4 Mbits/sec   15   1.21 MBytes       
[  7][RX-C]   3.00-4.00   sec  14.0 MBytes   117 Mbits/sec                  
[  5][TX-C]   4.00-5.00   sec  6.88 MBytes  57.7 Mbits/sec    0   1.29 MBytes       
[  7][RX-C]   4.00-5.00   sec  21.2 MBytes   178 Mbits/sec                  


Note that iwconfig doesn't really help to show things:
ros@sigyn7900:~/wifi_logger_visualizer_ws$ iwconfig
lo        no wireless extensions.

eno1      no wireless extensions.

wlp8s0    IEEE 802.11  ESSID:"livingroom"  
          Mode:Managed  Frequency:5.745 GHz  Access Point: 24:E5:0F:43:4E:DC   
          Bit Rate=432.3 Mb/s   Tx-Power=3 dBm   
          Retry short limit:7   RTS thr:off   Fragment thr:off
          Power Management:on
          Link Quality=56/70  Signal level=-54 dBm  
          Rx invalid nwid:0  Rx invalid crypt:0  Rx invalid frag:0
          Tx excessive retries:0  Invalid misc:25   Missed beacon:0

docker0   no wireless extensions.
