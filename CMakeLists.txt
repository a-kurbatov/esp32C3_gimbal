cmake_minimum_required(VERSION 3.16)

# Ensure default target is ESP32-C3 when none is configured yet.
# This can still be overridden by `idf.py set-target` or -DIDF_TARGET=...
set(IDF_TARGET esp32c3 CACHE STRING "IDF Target" FORCE)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(gimbal_esp32c3)
