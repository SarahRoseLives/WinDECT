cmake_minimum_required(VERSION 3.16)
project(WinDECT VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ── Bundled third-party libraries (headers + import .libs from dlls/) ──────────
set(TP_DIR "${CMAKE_SOURCE_DIR}/third_party")

set(HACKRF_INCLUDE_DIR "${TP_DIR}/hackrf")
set(HACKRF_LIBRARY     "${TP_DIR}/hackrf/hackrf.lib")
set(HACKRF_DLL         "${CMAKE_SOURCE_DIR}/dlls/hackrf.dll")

set(LIBUSB_INCLUDE_DIR "${TP_DIR}/libusb/include")
set(LIBUSB_LIBRARY     "${TP_DIR}/libusb/libusb-1.0.lib")
set(LIBUSB_DLL         "${CMAKE_SOURCE_DIR}/dlls/libusb-1.0.dll")

set(PTHREAD_DLL        "${CMAKE_SOURCE_DIR}/dlls/pthreadVC2.dll")

foreach(f ${HACKRF_LIBRARY} ${HACKRF_DLL} ${LIBUSB_LIBRARY} ${LIBUSB_DLL})
    if(NOT EXISTS "${f}")
        message(FATAL_ERROR "Required file not found: ${f}")
    endif()
endforeach()

message(STATUS "HackRF lib  : ${HACKRF_LIBRARY}")
message(STATUS "libusb lib  : ${LIBUSB_LIBRARY}")

# ── WinDECT executable ─────────────────────────────────────────────────────────
add_executable(windect
    src/main.cpp
    src/hackrf_source.cpp
    src/packet_receiver.cpp
    src/packet_decoder.cpp
    src/wideband_monitor.cpp
    src/g72x.c
    src/audio_out.cpp
)

# g72x.c is a plain C file — tell CMake to compile it as C, not C++
set_source_files_properties(src/g72x.c PROPERTIES LANGUAGE C)

target_include_directories(windect PRIVATE
    ${HACKRF_INCLUDE_DIR}
    ${LIBUSB_INCLUDE_DIR}
    src
)

target_link_libraries(windect PRIVATE
    ${HACKRF_LIBRARY}
    ${LIBUSB_LIBRARY}
    ws2_32
    winmm
)

# Copy all required DLLs next to the built executable
foreach(dll ${HACKRF_DLL} ${LIBUSB_DLL} ${PTHREAD_DLL})
    add_custom_command(TARGET windect POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
            "${dll}"
            "$<TARGET_FILE_DIR:windect>"
        COMMENT "Copying ${dll}"
    )
endforeach()

# ── Compile options ────────────────────────────────────────────────────────────
if(MSVC)
    target_compile_options(windect PRIVATE /W3 /O2 /EHsc)
else()
    target_compile_options(windect PRIVATE -Wall -O2)
endif()
