proc rd {addr} { return [lindex [read_memory $addr 32 1] 0] }
proc rb {addr} { return [lindex [read_memory $addr 8 1] 0] }
init
set t0 [rd 0x40000024]; set k0 [rd 0x2000447c]; set s0 [rd 0x20014cd4]
sleep 1000
set t1 [rd 0x40000024]; set k1 [rd 0x2000447c]; set s1 [rd 0x20014cd4]
echo "TIM2_CNT $t0 -> $t1"
echo "xTickCount $k0 -> $k1"
echo "stack_health_sample_count $s0 -> $s1"
echo "DBGMCU_APB1LFZ1 [format 0x%08x [rd 0x5C00103C]]"
echo "TIM2_CR1 [format 0x%08x [rd 0x40000000]] PSC [rd 0x40000028] ARR [format 0x%08x [rd 0x4000002C]]"
echo "xSchedulerRunning [rd 0x20004484] uxCurrentNumberOfTasks [rd 0x20004478]"
echo "manager_state [rb 0x20014cc8] rtos_init_timing [rb 0x20014cc9] failure_stage [rb 0x20014cca] created_task_count [rd 0x20014ccc] start_refused [rd 0x20014cd0]"
echo "stack_health_archive [read_memory 0x24000160 32 7]"
set pcs {}
for {set i 0} {$i < 8} {incr i} { lappend pcs [format 0x%08x [rd 0xE000101C]]; sleep 50 }
echo "DWT_PCSR $pcs"
echo "DEMCR [format 0x%08x [rd 0xE000EDFC]]"
shutdown
