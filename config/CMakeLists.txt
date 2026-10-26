# config：一台机器人的数据（profiles）与组合根（assembly）。只在固件构建中加入。

# profile_<名称>：纯数据的生成头，叶子 target，没有任何依赖（规则 4）。
set(profile_generated_dir "${CMAKE_CURRENT_BINARY_DIR}/generated/${PROFILE_ID}")
configure_file(profiles/profile.hpp.in "${profile_generated_dir}/profile.hpp" @ONLY)
add_library(profile_${PROFILE_ID} INTERFACE)
target_include_directories(profile_${PROFILE_ID} INTERFACE "${profile_generated_dir}")

add_subdirectory(assembly)
