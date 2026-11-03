#ifndef TEST_HOST_UNIT_ROBOLINK_GOLDEN_HPP
#define TEST_HOST_UNIT_ROBOLINK_GOLDEN_HPP

// 读 RoboLink 标准帧（test/data/robolink/golden_v0_2.txt，由 tool/robolink/make_golden.py 独立生成）。
// 文件路径由 CMake 以 ROBOLINK_GOLDEN_PATH 给出。

#include <cassert>
#include <cstdint>
#include <cstdio>
#include <fstream>
#include <map>
#include <sstream>
#include <string>
#include <vector>

namespace golden
{

struct Vector
{
    std::string expect; // ok、payload，或帧被拒的原因
    std::vector<std::uint8_t> bytes;
};

using Table = std::map<std::string, Vector>;

inline Table load()
{
    std::ifstream in(ROBOLINK_GOLDEN_PATH);
    if (!in)
    {
        std::fprintf(stderr, "打不开标准帧文件 %s\n", ROBOLINK_GOLDEN_PATH);
        assert(false);
    }
    Table table;
    std::string line;
    while (std::getline(in, line))
    {
        if (line.empty() || line[0] == '#')
        {
            continue;
        }
        std::istringstream fields(line);
        std::string name;
        Vector vector;
        std::string hex;
        fields >> name >> vector.expect >> hex;
        assert(!name.empty() && !vector.expect.empty() && hex.size() % 2U == 0U);
        for (std::size_t index = 0U; index < hex.size(); index += 2U)
        {
            vector.bytes.push_back(static_cast<std::uint8_t>(std::stoul(hex.substr(index, 2U), nullptr, 16)));
        }
        const bool inserted = table.emplace(name, vector).second;
        assert(inserted);
        (void)inserted;
    }
    assert(table.size() >= 30U);
    return table;
}

inline const Vector &get(const Table &table, const char *name)
{
    const auto found = table.find(name);
    if (found == table.end())
    {
        std::fprintf(stderr, "标准帧里没有 %s\n", name);
        assert(false);
    }
    return found->second;
}

} // namespace golden

#endif // TEST_HOST_UNIT_ROBOLINK_GOLDEN_HPP
