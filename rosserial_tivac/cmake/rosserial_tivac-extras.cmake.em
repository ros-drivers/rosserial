cmake_minimum_required(VERSION 3.7.2)

@[if DEVELSPACE]@
set(ROSSERIAL_TIVAC_TOOLCHAIN "@(CMAKE_CURRENT_SOURCE_DIR)/tivac-cmake/cmake/TivaCToolchain.cmake")
@[else]@
set(ROSSERIAL_TIVAC_TOOLCHAIN "${rosserial_tivac_DIR}/../tivac-cmake/cmake/TivaCToolchain.cmake")
@[end if]@
