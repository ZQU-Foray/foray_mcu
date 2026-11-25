// hal::Nvs 在 DM-MC02 上的实现：H723 的最后一个 Flash 扇区（0x080E0000–0x080FFFFF，128 KB），
// 程序镜像由 board/overlay.ld 的链接期断言挡在它前面。
// 写按 H7 的 Flash word（32 B）编程，只能写在擦除态上；擦除是整扇区，会停 CPU 1–2 s。
// 只有一块区，编号 0（board.hpp 的 NVS_BIAS）。
#include "hal/nvs.hpp"

#include "board.hpp"
#include "main.h" // 带进 stm32h7xx_hal.h

#include <cstddef>
#include <cstdint>
#include <cstring>

namespace
{

constexpr std::uint32_t BASE_ADDRESS{0x080E0000U};
constexpr std::size_t REGION_SIZE{128U * 1024U};
constexpr std::size_t WRITE_GRANULARITY{32U}; // FLASH_NB_32BITWORD_IN_FLASHWORD × 4
constexpr std::uint32_t SECTOR{FLASH_SECTOR_7};
constexpr std::uint32_t VOLTAGE_RANGE{FLASH_VOLTAGE_RANGE_3}; // 2.7–3.6 V，板上是 3.3 V
constexpr std::uint8_t ERASED_BYTE{0xFFU};

std::uint8_t *region() noexcept
{
    return reinterpret_cast<std::uint8_t *>(BASE_ADDRESS);
}

bool bounds_ok(std::size_t offset, std::size_t length) noexcept
{
    return offset <= REGION_SIZE && length <= REGION_SIZE - offset;
}

bool erased(std::size_t offset, std::size_t length) noexcept
{
    const std::uint8_t *bytes = region();
    for (std::size_t index = 0U; index < length; ++index)
    {
        if (bytes[offset + index] != ERASED_BYTE)
        {
            return false;
        }
    }
    return true;
}

hal::Nvs::Result program(std::size_t offset, const std::uint8_t *data, std::size_t length) noexcept
{
    if (HAL_FLASH_Unlock() != HAL_OK)
    {
        return hal::Nvs::Result::ERROR;
    }
    hal::Nvs::Result result = hal::Nvs::Result::OK;
    for (std::size_t done = 0U; done < length; done += WRITE_GRANULARITY)
    {
        const std::uint32_t address = BASE_ADDRESS + static_cast<std::uint32_t>(offset + done);
        const std::uint32_t source = reinterpret_cast<std::uint32_t>(data + done);
        if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_FLASHWORD, address, source) != HAL_OK)
        {
            result = hal::Nvs::Result::ERROR;
            break;
        }
    }
    (void)HAL_FLASH_Lock();
    return result;
}

} // namespace

namespace hal
{

std::size_t Nvs::size() const noexcept
{
    return index_ == 0U ? REGION_SIZE : 0U;
}

std::size_t Nvs::write_granularity() const noexcept
{
    return index_ == 0U ? WRITE_GRANULARITY : 0U;
}

Nvs::Result Nvs::read(std::size_t offset, std::uint8_t *data, std::size_t length) const noexcept
{
    if (index_ != 0U)
    {
        return Result::UNSUPPORTED;
    }
    if (data == nullptr || length == 0U || !bounds_ok(offset, length))
    {
        return Result::INVALID_ARGUMENT;
    }
    // Flash 是内存映射的：D-Cache 没有开，直接拷
    std::memcpy(data, region() + offset, length);
    return Result::OK;
}

Nvs::Result Nvs::write(std::size_t offset, const std::uint8_t *data, std::size_t length) const noexcept
{
    if (index_ != 0U)
    {
        return Result::UNSUPPORTED;
    }
    if (data == nullptr || length == 0U || !bounds_ok(offset, length))
    {
        return Result::INVALID_ARGUMENT;
    }
    if ((offset % WRITE_GRANULARITY) != 0U || (length % WRITE_GRANULARITY) != 0U)
    {
        return Result::NOT_ALIGNED;
    }
    if (!erased(offset, length))
    {
        return Result::NOT_ERASED;
    }
    return program(offset, data, length);
}

Nvs::Result Nvs::erase() const noexcept
{
    if (index_ != 0U)
    {
        return Result::UNSUPPORTED;
    }
    FLASH_EraseInitTypeDef erase{};
    erase.TypeErase = FLASH_TYPEERASE_SECTORS;
    erase.Banks = FLASH_BANK_1;
    erase.Sector = SECTOR;
    erase.NbSectors = 1U;
    erase.VoltageRange = VOLTAGE_RANGE;
    std::uint32_t error = 0U;
    if (HAL_FLASH_Unlock() != HAL_OK)
    {
        return Result::ERROR;
    }
    const HAL_StatusTypeDef status = HAL_FLASHEx_Erase(&erase, &error);
    (void)HAL_FLASH_Lock();
    return status == HAL_OK ? Result::OK : Result::ERROR;
}

} // namespace hal
