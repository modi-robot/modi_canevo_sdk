# ---- 示例程序 ----
add_executable(01_bus_open_close 01_bus_open_close.cpp)
add_executable(02_test_sdo 02_test_sdo.cpp)
add_executable(03_csv_test 03_csv_test.cpp)
add_executable(04_pp_test 04_pp_test.cpp)
add_executable(05_csp_test 05_csp_test.cpp)
add_executable(example_csp example_csp.cpp)

target_link_libraries(01_bus_open_close PRIVATE canevo_sdk_x86)
target_link_libraries(02_test_sdo PRIVATE canevo_sdk_x86)
target_link_libraries(03_csv_test PRIVATE canevo_sdk_x86)
target_link_libraries(04_pp_test PRIVATE canevo_sdk_x86)
target_link_libraries(05_csp_test PRIVATE canevo_sdk_x86)
target_link_libraries(example_csp PRIVATE canevo_sdk_x86)

