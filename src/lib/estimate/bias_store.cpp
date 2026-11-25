#include "lib/estimate/bias_store.hpp"

#include "utils/crc/crc16.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>

namespace estimate
{

namespace
{

constexpr std::uint32_t MAGIC{0x424E5356U}; // 内存里依次是 'V' 'S' 'N' 'B'
constexpr std::uint16_t VERSION{1U};
constexpr std::size_t CRC_COVERED{30U};
constexpr std::size_t CRC_OFFSET{30U};
constexpr std::size_t MAXIMUM_STRIDE{256U};

std::uint16_t get_u16(const std::uint8_t *bytes) noexcept
{
    return static_cast<std::uint16_t>(bytes[0]) | static_cast<std::uint16_t>(static_cast<std::uint16_t>(bytes[1]) << 8U);
}

std::uint32_t get_u32(const std::uint8_t *bytes) noexcept
{
    return static_cast<std::uint32_t>(bytes[0]) | (static_cast<std::uint32_t>(bytes[1]) << 8U) |
           (static_cast<std::uint32_t>(bytes[2]) << 16U) | (static_cast<std::uint32_t>(bytes[3]) << 24U);
}

void put_u16(std::uint8_t *bytes, std::uint16_t value) noexcept
{
    bytes[0] = static_cast<std::uint8_t>(value & 0xFFU);
    bytes[1] = static_cast<std::uint8_t>((value >> 8U) & 0xFFU);
}

void put_u32(std::uint8_t *bytes, std::uint32_t value) noexcept
{
    bytes[0] = static_cast<std::uint8_t>(value & 0xFFU);
    bytes[1] = static_cast<std::uint8_t>((value >> 8U) & 0xFFU);
    bytes[2] = static_cast<std::uint8_t>((value >> 16U) & 0xFFU);
    bytes[3] = static_cast<std::uint8_t>((value >> 24U) & 0xFFU);
}

std::uint32_t float_bits(float value) noexcept
{
    std::uint32_t bits = 0U;
    std::memcpy(&bits, &value, sizeof(bits));
    return bits;
}

float bits_float(std::uint32_t bits) noexcept
{
    float value = 0.0F;
    std::memcpy(&value, &bits, sizeof(value));
    return value;
}

bool finite(const math::Vector3 &value) noexcept
{
    return std::isfinite(value.x) && std::isfinite(value.y) && std::isfinite(value.z);
}

} // namespace

bool BiasStore::config_valid(const Config &config) const noexcept
{
    return config.maximum_records >= 1U && config.maximum_records <= MAXIMUM_POINTS &&
           std::isfinite(config.temperature_span_celsius) && config.temperature_span_celsius > 0.0F &&
           std::isfinite(config.extrapolation_celsius) && config.extrapolation_celsius >= 0.0F &&
           std::isfinite(config.maximum_slope_rad_per_sec_per_celsius) &&
           config.maximum_slope_rad_per_sec_per_celsius > 0.0F &&
           std::isfinite(config.minimum_temperature_celsius) && std::isfinite(config.maximum_temperature_celsius) &&
           config.minimum_temperature_celsius < config.maximum_temperature_celsius;
}

bool BiasStore::init(const Storage &storage, const Config &config) noexcept
{
    storage_ = storage;
    config_ = config;
    snapshot_ = Snapshot{};
    points_ = {};
    count_ = 0U;
    stride_ = RECORD_SIZE;
    sequence_ = 1U;
    constant_bias_ = math::Vector3{};
    slope_ = math::Vector3{};
    minimum_temperature_ = 0.0F;
    maximum_temperature_ = 0.0F;
    if (!config_valid(config))
    {
        return false;
    }
    if (storage_.context == nullptr || storage_.size == nullptr || storage_.granularity == nullptr ||
        storage_.read == nullptr || storage_.write == nullptr || storage_.erase == nullptr)
    {
        return true; // 没有接存储：能力照常运行，只是没有存储零偏
    }

    const std::size_t size = storage_.size(storage_.context);
    const std::size_t granularity = storage_.granularity(storage_.context);
    if (size == 0U || granularity == 0U || granularity > MAXIMUM_STRIDE)
    {
        return true; // 设备不可用：能力照常运行，只是没有存储零偏
    }
    stride_ = ((RECORD_SIZE + granularity - 1U) / granularity) * granularity;
    const std::size_t capacity = size / stride_;
    if (capacity == 0U)
    {
        return true;
    }
    snapshot_.available = true;
    snapshot_.capacity = capacity;
    scan();
    fit();
    return true;
}

void BiasStore::scan() noexcept
{
    std::size_t slot = 0U;
    bool compact_needed = false;
    for (; slot < snapshot_.capacity; ++slot)
    {
        Point point{};
        if (read_record(slot, point))
        {
            append_point(point);
            continue;
        }
        if (slot_erased(slot))
        {
            break; // 日志的末尾
        }
        compact_needed = true; // 中间有写坏的记录：后面即使有有效记录也不再采用
        break;
    }
    snapshot_.append_slot = slot;
    if (compact_needed || slot == snapshot_.capacity)
    {
        compact();
    }
}

void BiasStore::compact() noexcept
{
    if (!storage_.erase(storage_.context))
    {
        snapshot_.available = false;
        snapshot_.append_slot = 0U;
        return;
    }
    snapshot_.compactions = 1U;
    std::size_t rewritten = 0U;
    for (std::size_t index = 0U; index < count_; ++index)
    {
        if (!write_point(index, points_[index]))
        {
            snapshot_.available = false;
            break;
        }
        ++rewritten;
    }
    snapshot_.append_slot = rewritten;
}

void BiasStore::append_point(const Point &point) noexcept
{
    if (count_ == config_.maximum_records)
    {
        for (std::size_t index = 1U; index < count_; ++index)
        {
            points_[index - 1U] = points_[index];
        }
        --count_;
    }
    points_[count_] = point;
    ++count_;
    if (point.sequence >= sequence_)
    {
        sequence_ = point.sequence + 1U;
    }
}

void BiasStore::fit() noexcept
{
    snapshot_.records = count_;
    snapshot_.sequence = sequence_;
    if (count_ == 0U)
    {
        snapshot_.has_bias = false;
        snapshot_.fit_is_line = false;
        constant_bias_ = math::Vector3{};
        slope_ = math::Vector3{};
        snapshot_.bias_at_newest = math::Vector3{};
        return;
    }
    snapshot_.has_bias = true;
    snapshot_.newest_temperature_celsius = points_[count_ - 1U].temperature_celsius;
    snapshot_.newest_bias_rad_per_sec = points_[count_ - 1U].bias_rad_per_sec;

    minimum_temperature_ = points_[0].temperature_celsius;
    maximum_temperature_ = points_[0].temperature_celsius;
    float total_weight = 0.0F;
    math::Vector3 weighted{};
    for (std::size_t index = 0U; index < count_; ++index)
    {
        const Point &point = points_[index];
        minimum_temperature_ = std::min(minimum_temperature_, point.temperature_celsius);
        maximum_temperature_ = std::max(maximum_temperature_, point.temperature_celsius);
        const float weight = static_cast<float>(point.weight);
        total_weight += weight;
        weighted = math::Vector3{weighted.x + point.bias_rad_per_sec.x * weight,
                                 weighted.y + point.bias_rad_per_sec.y * weight,
                                 weighted.z + point.bias_rad_per_sec.z * weight};
    }
    constant_bias_ = math::Vector3{
        weighted.x / total_weight, weighted.y / total_weight, weighted.z / total_weight};
    slope_ = math::Vector3{};
    snapshot_.fit_is_line = false;
    snapshot_.slope_rad_per_sec_per_celsius = math::Vector3{};

    if (count_ < 2U || maximum_temperature_ - minimum_temperature_ < config_.temperature_span_celsius)
    {
        return; // 只有一个温度点：按加权常数
    }

    // 加权最小二乘直线
    float mean_temperature = 0.0F;
    math::Vector3 mean_bias{};
    for (std::size_t index = 0U; index < count_; ++index)
    {
        const float weight = static_cast<float>(points_[index].weight);
        mean_temperature += weight * points_[index].temperature_celsius;
        mean_bias = math::Vector3{mean_bias.x + weight * points_[index].bias_rad_per_sec.x,
                                  mean_bias.y + weight * points_[index].bias_rad_per_sec.y,
                                  mean_bias.z + weight * points_[index].bias_rad_per_sec.z};
    }
    mean_temperature /= total_weight;
    mean_bias = math::Vector3{mean_bias.x / total_weight, mean_bias.y / total_weight, mean_bias.z / total_weight};

    float sxx = 0.0F;
    math::Vector3 sxy{};
    for (std::size_t index = 0U; index < count_; ++index)
    {
        const float weight = static_cast<float>(points_[index].weight);
        const float dt = points_[index].temperature_celsius - mean_temperature;
        sxx += weight * dt * dt;
        sxy = math::Vector3{sxy.x + weight * dt * points_[index].bias_rad_per_sec.x,
                            sxy.y + weight * dt * points_[index].bias_rad_per_sec.y,
                            sxy.z + weight * dt * points_[index].bias_rad_per_sec.z};
    }
    if (!(sxx > 0.0F))
    {
        return;
    }
    const math::Vector3 fitted_slope{sxy.x / sxx, sxy.y / sxx, sxy.z / sxx};
    const float limit = config_.maximum_slope_rad_per_sec_per_celsius;
    if (!finite(fitted_slope) || std::fabs(fitted_slope.x) > limit || std::fabs(fitted_slope.y) > limit ||
        std::fabs(fitted_slope.z) > limit)
    {
        return; // 斜率不可信：退回常数
    }
    slope_ = fitted_slope;
    constant_bias_ = math::Vector3{mean_bias.x - slope_.x * mean_temperature,
                                   mean_bias.y - slope_.y * mean_temperature,
                                   mean_bias.z - slope_.z * mean_temperature};
    snapshot_.fit_is_line = true;
    snapshot_.slope_rad_per_sec_per_celsius = slope_;
    snapshot_.bias_at_newest = bias_at(snapshot_.newest_temperature_celsius);
}

math::Vector3 BiasStore::bias_at(float temperature_celsius) const noexcept
{
    if (!snapshot_.has_bias)
    {
        return math::Vector3{};
    }
    float query = temperature_celsius;
    if (!std::isfinite(query))
    {
        query = snapshot_.newest_temperature_celsius;
    }
    query = std::max(minimum_temperature_ - config_.extrapolation_celsius, query);
    query = std::min(maximum_temperature_ + config_.extrapolation_celsius, query);
    if (!snapshot_.fit_is_line)
    {
        return constant_bias_;
    }
    return math::Vector3{slope_.x * query + constant_bias_.x,
                         slope_.y * query + constant_bias_.y,
                         slope_.z * query + constant_bias_.z};
}

bool BiasStore::record(float temperature_celsius,
                       const math::Vector3 &bias_rad_per_sec,
                       std::uint32_t window_us) noexcept
{
    if (!snapshot_.available || !std::isfinite(temperature_celsius) || !finite(bias_rad_per_sec) ||
        temperature_celsius < config_.minimum_temperature_celsius ||
        temperature_celsius > config_.maximum_temperature_celsius)
    {
        return false;
    }
    if (snapshot_.append_slot >= snapshot_.capacity)
    {
        return false; // 区满：不在运行期擦除，下次 init() 压缩
    }
    Point point{};
    point.temperature_celsius = temperature_celsius;
    point.bias_rad_per_sec = bias_rad_per_sec;
    const std::uint32_t milliseconds = (window_us + 999U) / 1000U;
    const std::uint32_t weight = std::max<std::uint32_t>(1U, std::min<std::uint32_t>(65535U, milliseconds));
    point.weight = static_cast<std::uint16_t>(weight);
    point.sequence = sequence_;
    if (!write_point(snapshot_.append_slot, point))
    {
        snapshot_.available = false;
        return false;
    }
    ++snapshot_.append_slot;
    ++snapshot_.written;
    ++sequence_;
    append_point(point);
    fit();
    return true;
}

bool BiasStore::write_point(std::size_t slot, const Point &point) noexcept
{
    std::array<std::uint8_t, MAXIMUM_STRIDE> buffer{}; // 擦除态写 0 是允许的，多出来的对齐字节留 0
    put_u32(buffer.data() + 0U, MAGIC);
    put_u16(buffer.data() + 4U, VERSION);
    put_u16(buffer.data() + 6U, point.weight);
    put_u32(buffer.data() + 8U, point.sequence);
    put_u32(buffer.data() + 12U, float_bits(point.temperature_celsius));
    put_u32(buffer.data() + 16U, float_bits(point.bias_rad_per_sec.x));
    put_u32(buffer.data() + 20U, float_bits(point.bias_rad_per_sec.y));
    put_u32(buffer.data() + 24U, float_bits(point.bias_rad_per_sec.z));
    put_u16(buffer.data() + 28U, 0U);
    put_u16(buffer.data() + CRC_OFFSET, crc::crc16_calc(buffer.data(), CRC_COVERED));
    return storage_.write(storage_.context, slot * stride_, buffer.data(), stride_);
}

bool BiasStore::read_record(std::size_t slot, Point &point) const noexcept
{
    std::array<std::uint8_t, RECORD_SIZE> buffer{};
    if (!storage_.read(storage_.context, slot * stride_, buffer.data(), buffer.size()))
    {
        return false;
    }
    if (get_u32(buffer.data() + 0U) != MAGIC || get_u16(buffer.data() + 4U) != VERSION)
    {
        return false;
    }
    const std::uint16_t expected = crc::crc16_calc(buffer.data(), CRC_COVERED);
    if (get_u16(buffer.data() + CRC_OFFSET) != expected)
    {
        return false;
    }
    point.temperature_celsius = bits_float(get_u32(buffer.data() + 12U));
    point.bias_rad_per_sec = math::Vector3{bits_float(get_u32(buffer.data() + 16U)),
                                           bits_float(get_u32(buffer.data() + 20U)),
                                           bits_float(get_u32(buffer.data() + 24U))};
    point.sequence = get_u32(buffer.data() + 8U);
    point.weight = std::max<std::uint16_t>(1U, get_u16(buffer.data() + 6U));
    if (!std::isfinite(point.temperature_celsius) || !finite(point.bias_rad_per_sec) ||
        point.temperature_celsius < config_.minimum_temperature_celsius ||
        point.temperature_celsius > config_.maximum_temperature_celsius)
    {
        return false;
    }
    return true;
}

bool BiasStore::slot_erased(std::size_t slot) const noexcept
{
    std::array<std::uint8_t, RECORD_SIZE> buffer{};
    if (!storage_.read(storage_.context, slot * stride_, buffer.data(), buffer.size()))
    {
        return false;
    }
    return std::all_of(buffer.begin(), buffer.end(), [](std::uint8_t byte) { return byte == 0xFFU; });
}

} // namespace estimate
