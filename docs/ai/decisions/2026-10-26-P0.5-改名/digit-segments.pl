# P0.5 改名的手工修正：常量名里的数字分段（perl -pi 逐行执行）。
# clang-tidy 把数字并入前一个词（ACCEL_ODR100_HZ、CHANNEL1）。约定（Decision-20261026-05）：
#   - 数值+单位、序号单独成段：ACCEL_ODR_100_HZ、POWER_24V_0、CHANNEL_1、LEVEL_0；
#   - 名字里固有的数字保持原样：型号 BMI088、GM6020、c610，引脚名 INTERRUPT1，
#     术语 OSR2、crc8，数学常量 SQRT2，标号 WHEEL_M1。
my %map = (
    # 型号缩写不拆
    'G_M6020_CURRENT' => 'GM6020_CURRENT',
    # 数值+单位
    'POWER5_V'         => 'POWER_5V',
    'POWER24_V0'       => 'POWER_24V_0',
    'POWER24_V1'       => 'POWER_24V_1',
    'power5_v_level'   => 'power_5v_level',
    'ACCEL_ODR12_5_HZ' => 'ACCEL_ODR_12_5_HZ',
    # 序号：与 C 层宏（CAN_PORT_CHANNEL_1）一致
    'CHANNEL1' => 'CHANNEL_1',
    'CHANNEL2' => 'CHANNEL_2',
    'CHANNEL3' => 'CHANNEL_3',
    'LEVEL0'   => 'LEVEL_0',
    'LEVEL1'   => 'LEVEL_1',
    'DIGITAL_CHANNEL17_FLAG' => 'DIGITAL_CHANNEL_17_FLAG',
    'DIGITAL_CHANNEL18_FLAG' => 'DIGITAL_CHANNEL_18_FLAG',
    'digital_channel17'      => 'digital_channel_17',
    'digital_channel18'      => 'digital_channel_18',
);
for my $n (qw(25 50 100 200 400 800 1600)) { $map{"ACCEL_ODR${n}_HZ"} = "ACCEL_ODR_${n}_HZ"; }
for my $n (qw(3 6 12 24))                  { $map{"ACCEL_RANGE${n}G"}  = "ACCEL_RANGE_${n}G"; }
for my $n (qw(250 500 1000 2000))          { $map{"GYRO_RANGE${n}_DPS"} = "GYRO_RANGE_${n}_DPS"; }
for my $pair (qw(1000_116 1000_47 100_12 100_6 2000_230 2000_532 400_23 400_47)) {
    my ($odr, $bandwidth) = split /_/, $pair;
    $map{"GYRO_BANDWIDTH${odr}_HZ${bandwidth}_HZ"} = "GYRO_BANDWIDTH_${odr}_HZ_${bandwidth}_HZ";
}
my $names = join '|', map { quotemeta } sort { length($b) <=> length($a) } keys %map;
s/\b($names)\b/$map{$1}/g;
