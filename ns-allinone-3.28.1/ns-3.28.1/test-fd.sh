ifconfig eno1 promisc
./waf --run 'src/fd-net-device/examples/fd-dpdk-emu-onoff \
    --deviceName=eno1 \
    --client=192.168.43.1 \
    --server=192.168.43.2 \
    --mac-server=f4:8e:38:f4:6b:06 \
    --data-rate='"$1"'Mb/s \
    --transportPort='"$2"' \
    --dpdkMode=false '
