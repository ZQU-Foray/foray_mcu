add_library(platform_interface INTERFACE)

target_include_directories(platform_interface INTERFACE
    ${CMAKE_CURRENT_SOURCE_DIR}
)

target_compile_features(platform_interface INTERFACE cxx_std_17)
