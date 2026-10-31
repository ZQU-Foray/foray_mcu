proc rd {addr} { return [lindex [read_memory $addr 32 1] 0] }
proc rw {addr n} { return [read_memory $addr 32 $n] }
init
set t0 [rd 0x40000024]; set k0 [rd 0x2000447c]; set h0 [rd 0x20014cc0]; set p0 [rd 0x24000168]
set s0 [rw 0x200155b4 8]; set m0 [rw 0x240001a4 7]
sleep 1000
set t1 [rd 0x40000024]; set k1 [rd 0x2000447c]; set h1 [rd 0x20014cc0]; set p1 [rd 0x24000168]
set s1 [rw 0x200155b4 8]; set m1 [rw 0x240001a4 7]
echo "TIM2_CNT $t0 -> $t1 ; xTickCount $k0 -> $k1"
echo "heartbeat.sequence $h0 -> $h1 ; topic publish_count $p0 -> $p1"
echo "runner.started [lindex [read_memory 0x200155da 8 1] 0]"
echo "status(heartbeat: run,max_us,overrun,stack | monitor: run,max_us,overrun,stack) t0: $s0"
echo "status t1: $s1"
echo "monitor.report(received,lost,out_of_order,late,last_latency,max_latency,last_seq) t0: $m0"
echo "monitor.report t1: $m1"
set pcs {}
for {set i 0} {$i < 16} {incr i} { lappend pcs [format 0x%08x [rd 0xE000101C]]; sleep 30 }
echo "DWT_PCSR $pcs"
shutdown
