# majestic-af — out-of-core autofocus plugin for majestic.
#
# Builds majestic-af.so, a shared object majestic dlopen()s at /usr/lib/. Cross
# compile against the same OpenIPC toolchain majestic uses, e.g.:
#
#   cmake -Bbuild -DCMAKE_TOOLCHAIN_FILE=<majestic>/tools/cmake/toolchains/<cc>.cmake
#   cmake --build build
#
# The HAL seams the plugin calls (sdk_get_focus_value, sdk_set_zoom_mag,
# config_get_*, log_log) are intentionally left UNDEFINED here: they resolve at
# dlopen against the majestic executable, which exports them via its
# cmake/dynamic-list.txt when built WITH_PLUGINS_SUPPORT=ON. So this library does
# not link majestic — it only shares its ABI header.

cmake_minimum_required(VERSION 3.13)
project(majestic-af C)

set(CMAKE_C_STANDARD 99)
set(CMAKE_C_STANDARD_REQUIRED ON)

add_library(majestic-af SHARED
    src/plugin.c
    src/motion.c
    src/proto.c
    src/engine.c
    src/af2.c
    src/af3.c
    src/actuator.c
    src/act_uart.c
    src/act_ms41908.c
    src/ms41908_calc.c
    src/act_gpiostep.c
    src/gpiostep_calc.c)

target_include_directories(majestic-af PRIVATE include)

# Real .so name, not lib*.so — majestic looks for /usr/lib/majestic-af.so.
set_target_properties(majestic-af PROPERTIES
    PREFIX ""
    POSITION_INDEPENDENT_CODE ON)

# FILE_BASENAME is the tag majestic's log macros stamp each line with.
target_compile_definitions(majestic-af PRIVATE FILE_BASENAME="majestic-af")
target_compile_options(majestic-af PRIVATE -Os -Wall -Wextra)

include(GNUInstallDirs)
install(TARGETS majestic-af LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR})

# Offline model test — the af2 search's regression guard. It drives src/af2.c
# against a synthetic parfocal lens on a virtual clock, so it runs in
# milliseconds and needs no hardware. Host-native: built only when NOT
# cross-compiling the .so (a cross build has no host runner). Run with `ctest`.
enable_testing()
if(NOT CMAKE_CROSSCOMPILING)
    add_executable(af2_model_test tests/main.c tests/af2_model.c src/af2.c)
    target_include_directories(af2_model_test PRIVATE include tests)
    target_link_libraries(af2_model_test m)
    add_test(NAME af2_model COMMAND af2_model_test)

    # The af3 step-based bracket-and-return search (the MS41908M / vendor libxmaf method),
    # driven against a synthetic stepper lens with a post-move settle transient and reversal
    # backlash — the two things that defeat a naive search on this hardware. Host-native.
    add_executable(af3_model_test tests/af3_main.c tests/af3_model.c src/af3.c)
    target_include_directories(af3_model_test PRIVATE include tests)
    target_link_libraries(af3_model_test m)
    add_test(NAME af3_model COMMAND af3_model_test)

    # Wire frames. proto.c is pure — no HAL seams, no port — so the bytes this
    # plugin puts on the UART can be checked against the frames the btzoom
    # scripts sent for years, byte for byte. That reference is why deleting
    # those scripts changes nothing on the wire.
    add_executable(proto_test tests/proto_main.c tests/proto_test.c src/proto.c)
    target_include_directories(proto_test PRIVATE include tests)
    add_test(NAME proto COMMAND proto_test)

    # The MS41908M backend's pure logic: verb->axis mapping, the soft-limit clamp,
    # and the zoom->magnification curve. ms41908_calc.c has no SPI / /dev/mem /
    # thread / HAL dependency, so like proto.c it links and runs with nothing stubbed.
    add_executable(actuator_test
        tests/actuator_main.c tests/actuator_test.c src/ms41908_calc.c)
    target_include_directories(actuator_test PRIVATE include tests)
    add_test(NAME actuator COMMAND actuator_test)

    # The gpiostep backend's pure logic: the board config it reads, verb->axis with
    # the board's direction signs, and the clamp that keeps the head off its stops.
    add_executable(gpiostep_test
        tests/gpiostep_main.c tests/gpiostep_test.c src/gpiostep_calc.c)
    target_include_directories(gpiostep_test PRIVATE include tests)
    add_test(NAME gpiostep COMMAND gpiostep_test)

    # motion.c's report of motion to the core for a backend that cannot see its
    # motor: the real motion.c and its watchdog thread, a fake actuator, and the
    # engine and core seams stubbed.
    add_executable(motion_report_test tests/motion_report_test.c src/motion.c src/proto.c)
    target_include_directories(motion_report_test PRIVATE include tests)
    target_compile_definitions(motion_report_test PRIVATE FILE_BASENAME="motion")
    target_link_libraries(motion_report_test PRIVATE pthread)
    add_test(NAME motion_report COMMAND motion_report_test)

    # engine.c's state file and its two honesty rules. engine.c imports the core's
    # HAL seams and talks to motion.c, so the test stubs both; the state file and
    # the kernel's boot id are redirected into the build tree the same way the
    # offline model overrides af2's timing constants.
    add_executable(engine_state_test
        tests/engine_main.c tests/engine_state.c src/engine.c src/af2.c src/af3.c src/proto.c)
    target_include_directories(engine_state_test PRIVATE include tests)
    target_compile_definitions(engine_state_test PRIVATE
        FILE_BASENAME="engine"
        AF_ZOOM_STATE="${CMAKE_CURRENT_BINARY_DIR}/test-af.zoom"
        AF_BOOTID_PATH="${CMAKE_CURRENT_BINARY_DIR}/test-bootid"
        # a whole pass in milliseconds instead of a ~42 s cold seek
        AF_WINDOW_MS=100 AF_WIDE_MS=200 AF_BACKLASH_MS=10
        AF_TOTAL_BUDGET_MS=800 AF_SETTLE_MS=2 AF_SAMPLE_MS=2)
    target_link_libraries(engine_state_test m pthread)
    add_test(NAME engine_state COMMAND engine_state_test)
endif()
