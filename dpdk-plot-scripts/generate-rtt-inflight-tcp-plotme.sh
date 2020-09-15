echo "Generating rtt values and appending to rtt.plotme"
tshark -r fd-client-0-0.pcap -Y 'ip.addr == 192.168.43.1 && tcp.analysis.ack_rtt' -T fields -e tcp.analysis.ack_rtt > rtt.plotme

echo "Generating stats for rtt"
echo -e "1\t`cat rtt.plotme | datamash min 1 q1 1 median 1 q3 1 max 1`" > rtt-candlestick.plotme

echo "Generating stats for inflight"
echo -e "1\t`cat inflight.plotme | datamash min 2 q1 2 median 2 q3 2 max 2 -W`" > inflight-candlestick.plotme
