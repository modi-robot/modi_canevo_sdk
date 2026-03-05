# ---- 示例程序 ----
add_executable(example_csp example_csp.cpp)
add_executable(01_bus_open_close 01_bus_open_close.cpp)
add_executable(02_test_sdo 02_test_sdo.cpp)

target_link_libraries(example_csp PRIVATE canevo_sdk_x86)
target_link_libraries(01_bus_open_close PRIVATE canevo_sdk_x86)
target_link_libraries(02_test_sdo PRIVATE canevo_sdk_x86)
