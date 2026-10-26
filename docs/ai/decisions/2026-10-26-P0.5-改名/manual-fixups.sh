#!/usr/bin/env bash
# P0.5 改名：tool/ci/naming.py --fix 之后的手工修正（在仓库根执行）。
# 这些位置 clang-tidy 无法自动修正，或改名后需要消除同名冲突；每一项都由编译或残留扫描发现。
set -euo pipefail
here=$(cd "$(dirname "$0")" && pwd)

# 1. clang-tidy 标为无法自动修正的 19 处（模板里的依赖访问），按同一规则改名
perl -pi -e 's/\bPublish\b/publish/g' legacy/Libraries/Device/bmi088/Bmi088.hpp
perl -pi -e 's/\bxAxis\b/x_axis/g; s/\byAxis\b/y_axis/g; s/\bzAxis\b/z_axis/g;
             s/\bGetCompletionTick\b/get_completion_tick/g; s/\bTryGetSample\b/try_get_sample/g;
             s/\bGetReadCount\b/get_read_count/g' \
    legacy/Libraries/Device/bmi088/Bmi088.cpp \
    legacy/Libraries/Device/bmi088/Bmi088Accel.hpp legacy/Libraries/Device/bmi088/Bmi088Accel.cpp \
    legacy/Libraries/Device/bmi088/Bmi088Gyro.hpp legacy/Libraries/Device/bmi088/Bmi088Gyro.cpp \
    test/host/unit/imu_acquisition/bmi088_acquisition_test.cpp
perl -pi -e 's/\bMaximumMotors\b/MAXIMUM_MOTORS/g; s/\bMakeConfig\b/make_config/g' \
    legacy/Libraries/Device/motor/dji/DjiMotorModelMap.hpp

# 2. 未实例化模板里的调用与依赖访问（clang-tidy 看不到；编译器只在实例化时检查）
perl -pi -e 's/\bAt\(row, column\) \+ At\(column, row\)/at(row, column) + at(column, row)/' \
    legacy/Libraries/Algorithm/alg_math/Matrix.h
perl -pi -e 's/\.Added\(/.added(/g; s/\btemplate Multiply</template multiply</g' \
    legacy/Libraries/Algorithm/alg_estimate/KalmanFilter.hpp
perl -pi -e 's/\bUpdateStatus::NotUpdated\b/UpdateStatus::NOT_UPDATED/g;
             s/\bUpdateStatus::Ok\b/UpdateStatus::OK/g;
             s/\bUpdateStatus::NotConfigured\b/UpdateStatus::NOT_CONFIGURED/g;
             s/\bUpdateStatus::InvalidInput\b/UpdateStatus::INVALID_INPUT/g;
             s/\bUpdateStatus::CorrectionInvalid\b/UpdateStatus::CORRECTION_INVALID/g;
             s/\bUpdateStatus::Divergence\b/UpdateStatus::DIVERGENCE/g;
             s/^(\s*)Reset\(\);/$1reset();/' \
    legacy/Libraries/Algorithm/alg_estimate/ExtendedStateObserver.hpp
perl -pi -e 's/\.CountTime\b/.count_time/g; s/\.Stage\b/.stage/g' legacy/Libraries/Algorithm/alg_fsm/Fsm.h
perl -pi -e 's/\.GetOut\(\)/.get_out()/g' legacy/Libraries/Component/AxisController.hpp
perl -pi -e 's/\bMakeMockConfig\(/make_mock_config(/g' test/host/support/MockMotor.hpp
perl -pi -e 's/\breturn Transmit\(/return transmit(/; s/\breturn Transfer\(/return transfer(/;
             s/\breturn StartTransferAsync\(/return start_transfer_async(/' legacy/Platform/Interface/Spi.hpp
perl -pi -e 's/\breturn TryWrite\(/return try_write(/; s/\breturn TryRead\(/return try_read(/' \
    legacy/Platform/Interface/Uart.hpp legacy/Platform/Interface/UsbCdc.hpp

# 3. 改名后与局部变量同名：限定调用，两个名字都不变
perl -pi -e 's/const float norm = norm\(value\);/const float norm = alg_math::norm(value);/' \
    legacy/Libraries/Algorithm/alg_math/Vector3.h legacy/Libraries/Algorithm/alg_math/Quaternion.cpp

# 4. 测试辅助函数 Config() 改名后与局部变量 config 同名
perl -pi -e 's/\bconfig\(\)/default_config()/g' test/host/unit/motor/DjiMotorAdapterTest.cpp

# 5. 注释中提到的旧标识符同步为新名字（多段驼峰、映射唯一、非类型名）
python3 "$here/comments.py" "$here/改名表.tsv"

# 6. 常量名里的数字分段：数值+单位、序号单独成段，型号等固有数字不拆（见 digit-segments.pl）
git ls-files -z -- legacy/Application legacy/Libraries legacy/Platform/Interface test/host \
    | grep -zE '\.(h|hpp|cpp)$' | xargs -0 perl -pi "$here/digit-segments.pl"
