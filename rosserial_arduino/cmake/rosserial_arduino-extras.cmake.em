cmake_minimum_required(VERSION 3.7.2)

@[if DEVELSPACE]@
set(ROSSERIAL_ARDUINO_TOOLCHAIN "@(CMAKE_CURRENT_SOURCE_DIR)/arduino-cmake/cmake/ArduinoToolchain.cmake")
set(ROSSERIAL_TEENSY_TOOLCHAIN "@(CMAKE_CURRENT_SOURCE_DIR)/arduino-cmake/cmake/TeensyToolchain.cmake")
set(ROSSERIAL_TEENSY2_TOOLCHAIN "@(CMAKE_CURRENT_SOURCE_DIR)/arduino-cmake/cmake/Teensy2Toolchain.cmake")
@[else]@
set(ROSSERIAL_ARDUINO_TOOLCHAIN "${rosserial_arduino_DIR}/../arduino-cmake/cmake/ArduinoToolchain.cmake")
set(ROSSERIAL_TEENSY_TOOLCHAIN "${rosserial_arduino_DIR}/../arduino-cmake/cmake/TeensyToolchain.cmake")
set(ROSSERIAL_TEENSY2_TOOLCHAIN "${rosserial_arduino_DIR}/../arduino-cmake/cmake/Teensy2Toolchain.cmake")
@[end if]@

