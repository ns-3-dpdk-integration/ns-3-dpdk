./waf --run 'src/fd-net-device/examples/fd-dpdk-emu-onoff \
    --deviceName=0000:00:03.0 \
    --client=192.168.43.62 \
    --server=192.168.43.67 \
    --mac-server=08:00:27:82:81:0a \
    --data-rate=10Mb/s \
    --transportPort=Tcp \
    --dpdkMode=true'

# ./waf --run 'src/fd-net-device/examples/fd-dpdk-emu-onoff \
#     --deviceName=enp0s17 \
#     --client=192.168.43.62 \
#     --server=192.168.43.67 \
#     --mac-server=08:00:27:82:81:0a \
#     --data-rate=10Mb/s \
#     --transportPort=Tcp \
#     --dpdkMode=false '

# ./waf --run src/fd-net-device/examples/fd-dpdk-emu-onoff \
#     --command-template="gdb --args %s \
#     --deviceName=0000:00:11.0 \
#     --client=192.168.43.44 \
#     --server=192.168.43.142 \
#     --mac-server=08:00:27:82:81:0a \
#     --data-rate=10Mb/s \
#     --transportPort=Tcp \
#     --dpdkMode=true"