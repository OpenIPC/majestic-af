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
    src/engine.c
    src/af2.c)

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
endif()
