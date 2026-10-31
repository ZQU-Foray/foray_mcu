proc rd {addr} { return [lindex [read_memory $addr 32 1] 0] }
init
set g0 [rd 0x20015dec]; set a0 [rd 0x20015df0]; set k0 [rd 0x2000447c]
sleep 1000
set g1 [rd 0x20015dec]; set a1 [rd 0x20015df0]; set k1 [rd 0x2000447c]
echo "gyro_sequence $g0 -> $g1 ; accel_sequence $a0 -> $a1 ; ticks $k0 -> $k1"
shutdown
