# DRDY/DMA acquisition regression

Build this directory as a standalone native CMake project and run CTest:

```sh
cmake -S Tests/ImuAcquisition -B /tmp/controller-imu-acquisition-tests
cmake --build /tmp/controller-imu-acquisition-tests
ctest --test-dir /tmp/controller-imu-acquisition-tests --output-on-failure
```

Tests compile the actual BMI088 coordinator and all three child drivers with
mock C SPI/Time ports. They cover configuration readback, no untriggered reads
during initialization, callback forwarding, a DRDY arriving during an in-flight
read, shared-bus contention, coalesced edges, queue overflow/order, completion
before StartTransferAsync returns, 32-bit timestamp wrap, late starts, and DMA
errors. A concurrent producer also checks coherent ISR mailbox snapshots.

The software queue has one task owner. ISR callbacks only timestamp/notify;
they never parse samples, run any fusion algorithm, or manipulate that queue. Each
sensor has 32 records. Full queues reject the newest record and count overflow.
Latest-edge mailboxes count every observed interrupt but intentionally coalesce
older pending edges, since this configuration reads output registers, not FIFO.

`LateStart` means the DRDY-to-launch delay reached a nominal output period.
`NewerEventBeforeCompletion` conservatively marks a later observed DRDY before
completion; it is not proof that data was corrupted or lost. OS/interrupt latency
and sensor-internal conversion/filter delay are not removed by these timestamps.

Step 2 adds chronological PopNextSample: a completed record waits for any
older known pending/in-flight event, and ties deliver acceleration first.
Do not mix this API with the per-sensor Pop consumers in the same application.
Tests cover delayed earlier acceleration and equal-epoch gyro-first DMA completion.
ImuTask now forwards this stream to the raw-data recorder in
`Application/Imu/ImuDiagnostics`; the attitude estimator and its tests have
been moved to `Tests/backups`.

Hardware validation: retain the 532 Hz build, flash and verify the same BIN
image (ELF gaps need not match objcopy BIN padding), let the MCU run without
halting during acquisition, wait for the temperature-qualified frozen capture,
then save and decode it using `Tests/backups/Attitude/analyze_capture.py`. Repeat with
`gyroCaptureRequest=1`; do not reset the CPU between windows unless testing startup.
For acquisition timing when the board cannot reach the thermal qualification,
write `gyroCaptureRequest=2` instead. Report the actual temperatures and do not
compare its noise statistics as if they were measured at steady 50 Celsius.
