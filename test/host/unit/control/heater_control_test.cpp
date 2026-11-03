// control::HeaterControl：接 DM-MC02 实测辨识的热模型仿真。
// 被控对象：占空比 → 被加热区（τ1，增益 K）→ 传感器（τ2）；寄存器每 1.28 s 采样一次、截断到 0.125 ℃；
// 驱动每 40 ms 读一次，每 128 ms 调一次控制器，输出四舍五入到 1‰（与 device::Bmi088Heater 相同）。
// 验证：标称、加热器强度 ±25%（供电电压）、冷启动时板子自身升温、加热电源断开又恢复，以及参数校验；
// 热启动（板子热着时复位、没 24 V 上电之后来电）与一组宽松的扫描：都要稳到目标，不卡住、不大幅过冲。
#include "lib/control/heater_control.hpp"

#include <cassert>
#include <cmath>
#include <cstdio>

namespace
{

constexpr float K{0.967F};
constexpr float TAU_ZONE{15.5F};
constexpr float TAU_SENSOR{0.95F};
constexpr float TARGET{50.0F};
constexpr float PLANT_DT{0.01F};
constexpr float READ_PERIOD{0.04F};
constexpr float CONTROL_PERIOD{0.128F};
constexpr float REGISTER_PERIOD{1.28F};
constexpr float QUANTUM{0.125F};

control::HeaterControl::Config make_config()
{
    return control::HeaterControl::Config{
        control::HeaterControl::Model{K, TAU_ZONE, TAU_SENSOR},
        control::HeaterControl::Sensor{REGISTER_PERIOD, QUANTUM, READ_PERIOD / 2.0F},
        control::HeaterControl::Tuning{},
    };
}

struct Scenario
{
    float gain_scale{1.0F}; // 实际加热器强度 / 模型
    float start_c{37.0F};   // 起始温度
    float ambient_start_c{36.0F};
    float ambient_end_c{36.0F}; // 板子自身发热使环境温度缓慢变化
    float ambient_tau_s{300.0F};
    float register_phase_s{0.45F};
    float power_off_from_s{-1.0F}; // 加热电源断开的时段
    float power_off_to_s{-1.0F};
    float duration_s{150.0F};
};

struct Result
{
    float peak_c{0.0F};        // 最高读数
    float settle_s{-1.0F};     // 进入并保持目标 ±0.25 ℃ 的时刻
    float tail_min_c{1000.0F}; // 最后 60 s 的读数范围
    float tail_max_c{-1000.0F};
    float peak_after_restore_c{0.0F};
    float gain_scale{0.0F};
    control::HeaterControl::Phase final_phase{control::HeaterControl::Phase::IDLE};
};

Result run(const Scenario &scenario)
{
    control::HeaterControl heater;
    assert(heater.init(make_config()));
    float zone = scenario.start_c;
    float sensor = scenario.start_c;
    float ambient = scenario.ambient_start_c;
    float reg = std::floor(sensor / QUANTUM) * QUANTUM;
    float seen = reg;
    float change_time = 0.0F;
    float next_register = scenario.register_phase_s;
    float next_read = 0.0F;
    float next_control = 0.0F;
    float duty = 0.0F;
    float last_out_of_band = 0.0F;
    Result result{};
    const int steps = static_cast<int>(scenario.duration_s / PLANT_DT);
    for (int step = 0; step < steps; ++step)
    {
        const float now = static_cast<float>(step) * PLANT_DT;
        if (now >= next_register)
        {
            reg = std::floor(sensor / QUANTUM) * QUANTUM;
            next_register += REGISTER_PERIOD;
        }
        if (now >= next_read)
        {
            if (reg != seen)
            {
                seen = reg;
                change_time = now;
            }
            next_read += READ_PERIOD;
        }
        if (now >= next_control)
        {
            duty = std::round(heater.update(TARGET, seen, now - change_time, CONTROL_PERIOD));
            assert(duty >= 0.0F && duty <= 1000.0F);
            next_control += CONTROL_PERIOD;
            // 指标按控制器看到的读数统计
            result.peak_c = seen > result.peak_c ? seen : result.peak_c;
            if (std::fabs(seen - TARGET) > 0.25F)
            {
                last_out_of_band = now;
            }
            if (now >= scenario.duration_s - 60.0F)
            {
                result.tail_min_c = seen < result.tail_min_c ? seen : result.tail_min_c;
                result.tail_max_c = seen > result.tail_max_c ? seen : result.tail_max_c;
            }
            if (scenario.power_off_to_s > 0.0F && now > scenario.power_off_to_s)
            {
                result.peak_after_restore_c = seen > result.peak_after_restore_c ? seen : result.peak_after_restore_c;
            }
        }
        const bool powered = !(now >= scenario.power_off_from_s && now < scenario.power_off_to_s);
        const float heat = powered ? K * scenario.gain_scale * duty : 0.0F;
        ambient += (scenario.ambient_end_c - ambient) / scenario.ambient_tau_s * PLANT_DT;
        zone += (heat + ambient - zone) / TAU_ZONE * PLANT_DT;
        sensor += (zone - sensor) / TAU_SENSOR * PLANT_DT;
    }
    result.settle_s = last_out_of_band;
    result.gain_scale = heater.gain_scale();
    result.final_phase = heater.phase();
    return result;
}

void print(const char *name, const Result &result)
{
    std::printf("%-28s 最高 %.3f ℃  进入并保持 ±0.25 ℃ %.2f s  最后 60 s %.3f–%.3f ℃  强度修正 %.3f\n",
                name,
                static_cast<double>(result.peak_c),
                static_cast<double>(result.settle_s),
                static_cast<double>(result.tail_min_c),
                static_cast<double>(result.tail_max_c),
                static_cast<double>(result.gain_scale));
}

void check_settled(const Result &result, float max_overshoot, float max_settle_s)
{
    assert(result.peak_c <= TARGET + max_overshoot);
    assert(result.settle_s <= max_settle_s);
    assert(result.tail_min_c >= TARGET - 0.25F && result.tail_max_c <= TARGET + 0.25F);
    assert(result.final_phase == control::HeaterControl::Phase::REGULATE);
}

} // namespace

int main()
{
    std::setvbuf(stdout, nullptr, _IONBF, 0);
    // 参数校验
    {
        control::HeaterControl heater;
        assert(heater.update(TARGET, 40.0F, 0.0F, CONTROL_PERIOD) == 0.0F); // 未初始化
        control::HeaterControl::Config bad = make_config();
        bad.model.zone_time_constant_s = 0.0F;
        assert(!heater.init(bad));
        bad = make_config();
        bad.tuning.probe_fraction = 1.0F;
        assert(!heater.init(bad));
        bad = make_config();
        bad.tuning.hold_max_permille = 2000.0F;
        assert(!heater.init(bad));
        assert(heater.init(make_config()));
        assert(heater.update(TARGET, 40.0F, 0.0F, 0.0F) == 0.0F); // 周期无效
        assert(heater.update(TARGET, NAN, 0.0F, CONTROL_PERIOD) == 0.0F);
        // 第一个读数：开始探测级，用预热功率
        const float first = heater.update(TARGET, 40.0F, 0.0F, CONTROL_PERIOD);
        assert(heater.phase() == control::HeaterControl::Phase::PROBE);
        assert(first == make_config().tuning.boost_permille);
        heater.reset();
        assert(heater.phase() == control::HeaterControl::Phase::IDLE && heater.gain_scale() == 1.0F);
    }

    // 标称：37 ℃ 起，板子还在缓慢降温
    Scenario nominal{};
    const Result warm = run(nominal);
    print("标称 37 ℃", warm);
    check_settled(warm, 0.25F, 20.0F);
    assert(std::fabs(warm.gain_scale - 1.0F) < 0.1F);

    // 冷启动：25 ℃ 起，板子自身发热使环境升到 37 ℃
    Scenario cold{};
    cold.start_c = 25.0F;
    cold.ambient_start_c = 25.0F;
    cold.ambient_end_c = 37.0F;
    const Result cold_result = run(cold);
    print("冷启动 25 ℃", cold_result);
    check_settled(cold_result, 0.375F, 25.0F);

    // 加热器偏强 / 偏弱 25%（供电电压），各种寄存器相位
    for (const float phase : {0.05F, 0.45F, 0.85F, 1.25F})
    {
        for (const float strength : {0.75F, 1.25F})
        {
            for (const float start : {25.0F, 37.0F})
            {
                Scenario scenario{};
                scenario.gain_scale = strength;
                scenario.register_phase_s = phase;
                scenario.start_c = start;
                scenario.ambient_start_c = start == 25.0F ? 25.0F : 36.0F;
                scenario.ambient_end_c = start == 25.0F ? 37.0F : 36.0F;
                const Result result = run(scenario);
                char name[64];
                std::snprintf(name,
                              sizeof(name),
                              "强度 %.2f 起点 %.0f 相位 %.2f",
                              static_cast<double>(strength),
                              static_cast<double>(start),
                              static_cast<double>(phase));
                print(name, result);
                check_settled(result, 0.5F, 45.0F);
                // 探测级把加热器强度测出来了（方向正确）
                assert(strength > 1.0F ? result.gain_scale > 1.05F : result.gain_scale < 0.95F);
            }
        }
    }

    // 加热电源断开 60 s 又恢复：温度掉下去后重新预热，恢复后不冲过头
    Scenario outage{};
    outage.power_off_from_s = 60.0F;
    outage.power_off_to_s = 120.0F;
    outage.duration_s = 240.0F;
    const Result outage_result = run(outage);
    print("断电 60 s 后恢复", outage_result);
    assert(outage_result.peak_after_restore_c <= TARGET + 0.5F);
    check_settled(outage_result, 0.5F, 170.0F);

    // 热启动（2026-11-03 上板发现）：控制器把第一次读数当环境温度，板子热着时复位就估高了。
    // 原来会卡在目标下方（上板 48 ℃）
    struct WarmStart
    {
        const char *name;
        float start_c;
        float ambient_c;
        float power_on_s; // 加热电源接通的时刻（0：一直有电）
    };
    for (const WarmStart &warm_start : {WarmStart{"热复位 47.5 ℃（环境 42 ℃）", 47.5F, 42.0F, 0.0F},
                                        WarmStart{"恒温中复位 50 ℃（环境 40 ℃）", 50.0F, 40.0F, 0.0F},
                                        WarmStart{"热复位 45 ℃（环境 36 ℃）", 45.0F, 36.0F, 0.0F},
                                        WarmStart{"无 24 V 起 43.5 ℃，60 s 来电", 43.5F, 42.0F, 60.0F}})
    {
        Scenario scenario{};
        scenario.start_c = warm_start.start_c;
        scenario.ambient_start_c = warm_start.ambient_c;
        scenario.ambient_end_c = warm_start.ambient_c;
        if (warm_start.power_on_s > 0.0F)
        {
            scenario.power_off_from_s = 0.0F;
            scenario.power_off_to_s = warm_start.power_on_s;
        }
        scenario.duration_s = 240.0F;
        const Result result = run(scenario);
        print(warm_start.name, result);
        check_settled(result, 0.5F, 180.0F);
    }

    // 宽松扫描：热复位、恒温中断电、没 24 V 上电之后来电 × 加热器强度 ±25% × 两种传感器相位。
    // 只要求行为不异常：最后稳在目标 ±0.25 ℃，过冲不超过 2.5 ℃（个别组合会冲到 1–2 ℃）
    int scenarios = 0;
    float worst_peak = 0.0F;
    for (const float phase : {0.05F, 0.85F})
    {
        for (const float strength : {0.75F, 1.0F, 1.25F})
        {
            auto expect_normal = [&](const Scenario &scenario)
            {
                const Result result = run(scenario);
                ++scenarios;
                worst_peak = std::fmax(worst_peak, std::fmax(result.peak_c, result.peak_after_restore_c));
                if (!(result.tail_min_c >= TARGET - 0.25F && result.tail_max_c <= TARGET + 0.25F) ||
                    result.peak_after_restore_c > TARGET + 2.5F ||
                    (scenario.start_c <= TARGET && result.peak_c > TARGET + 2.5F))
                {
                    print("异常", result);
                    std::printf("  起 %.1f 环境 %.1f 强度 %.2f 断电 %.0f–%.0f s\n",
                                static_cast<double>(scenario.start_c),
                                static_cast<double>(scenario.ambient_start_c),
                                static_cast<double>(strength),
                                static_cast<double>(scenario.power_off_from_s),
                                static_cast<double>(scenario.power_off_to_s));
                    assert(false);
                }
            };
            for (const float start : {42.0F, 45.0F, 47.5F, 50.0F, 51.0F})
            {
                for (const float ambient : {30.0F, 36.0F, 40.0F})
                {
                    Scenario scenario{};
                    scenario.gain_scale = strength;
                    scenario.register_phase_s = phase;
                    scenario.start_c = start;
                    scenario.ambient_start_c = ambient;
                    scenario.ambient_end_c = ambient;
                    scenario.duration_s = 300.0F;
                    expect_normal(scenario);
                }
            }
            for (const float length : {3.0F, 30.0F, 120.0F})
            {
                Scenario scenario{};
                scenario.gain_scale = strength;
                scenario.register_phase_s = phase;
                scenario.power_off_from_s = 60.0F;
                scenario.power_off_to_s = 60.0F + length;
                scenario.duration_s = 360.0F;
                expect_normal(scenario);
            }
            for (const float ambient : {36.0F, 42.0F})
            {
                for (const float on : {20.0F, 120.0F})
                {
                    Scenario scenario{};
                    scenario.gain_scale = strength;
                    scenario.register_phase_s = phase;
                    scenario.start_c = ambient + 1.5F;
                    scenario.ambient_start_c = ambient;
                    scenario.ambient_end_c = ambient;
                    scenario.power_off_from_s = 0.0F;
                    scenario.power_off_to_s = on;
                    scenario.duration_s = on + 240.0F;
                    expect_normal(scenario);
                }
            }
        }
    }
    std::printf("宽松扫描 %d 个场景都稳到目标，最高读数 %.3f ℃\n", scenarios, static_cast<double>(worst_peak));
    return 0;
}
