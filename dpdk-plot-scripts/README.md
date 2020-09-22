# Scripts for ns-3-dpdk integration plots

## Usage

### Throughput

1. Store the iperf output on a file

2. Sanitize the iperf output
```
./sanitize-througput.sh iperf.out > throughput.plotme
```

Use `./sanitize-throughput-udp.sh` for UDP output.

3. Generate Candlestick Data
```
cat throughput.plotme | datamash min 1 q1 1 median 1 q3 1 max 1 > throughput-candlestick.plotme
```

NOTE: Install datamash by running `sudo apt install datamash`

4. Organize

Rename different emulation mode plotmes as follows:
```
throughput-{fd,nm,dpdk}-candlestick.plotme
```

Keep these files in directories named `tcp/1000` and `udp/1000` accordingly. Same goes for 100Mbps.

5. Generate Plot
```
gnuplot generate-candlestick-througput-1000.tolp
```

Same goes for 100Mbps.

### RTT and Bytes In flight (TCP only)

1. Store the packet capture from the client (`fd-client-0-0.pcap`) and Bytes In Flight plotme (`inflight.plotme`) in same location.

2. Generate Candlestick Data from above files
```
./generate-rtt-inflight-tcp-plotme.sh
```

3. Organize

Rename different emulation mode plotmes as follows:
```
{rtt,inflight}-{fd,nm,dpdk}-candlestick.plotme
```

Keep 100Mbps and 1000Mbps files in directories named `100` and `1000` respectively.

4. Generate Plots

RTT:
```
gnuplot generate-candlestick-rtt.tolp
```

Bytes In Flight:
```
gnuplot generate-candlestick-inflight.tolp
```

### Ping Latency (UDP only)

1. Genarate plotme

Run onoff example for UDP with ping enabled. It will generate `ping.plotme`

2. Generate Candlestick
```
cat ping.plotme | datamash min 1 q1 1 median 1 q3 1 max 1 > ping-candlestick.plotme
```

3. Organize

Rename different emulation mode plotmes as follows:
```
ping-{fd,nm,dpdk}-candlestick.plotme
```

Keep 100Mbps and 1000Mbps files in directories named `100` and `1000` respectively.

4. Generate Plot
```
gnuplot generate-candlestick-ping-1000.tolp
```

Same goes for 100Mbps.

