#=============================================================================#
# Author: Tomasz Bogdal (QueezyTheGreat)
# Home:   https://github.com/queezythegreat/arduino-cmake
#
# This Source Code Form is subject to the terms of the Mozilla Public
# License, v. 2.0. If a copy of the MPL was not distributed with this file,
# You can obtain one at http://mozilla.org/MPL/2.0/.
#
# Adaptation by David Lavoie-Boutin for teensy-cmake (dlavoieb)
#
#=============================================================================#
cmake_minimum_required(VERSION 2.8.3)
# Detect Arduino SDK
if(NOT ARDUINO_SDK_PATH)
    if(UNIX)
        list(APPEND SDK_PATH_HINTS 
            /opt/local/arduino
            /opt/arduino
            /usr/local/share/arduino
            ~/arduino
            ~/arduino-1.6.5
            )
    elseif(WIN32)
        set(SDK_PATH_HINTS "C:\\Program Files\\Arduino"
            "C:\\Program Files (x86)\\Arduino"
            )
    endif()
endif()

find_path(ARDUINO_SDK_PATH lib/version.txt
    PATHS ${SDK_PATH_HINTS} ENV arduino_location
    DOC "Arduino SDK path."
    )

if(ARDUINO_SDK_PATH)
    list(APPEND CMAKE_SYSTEM_PREFIX_PATH ${ARDUINO_SDK_PATH}/hardware/tools/avr)
    list(APPEND CMAKE_SYSTEM_PREFIX_PATH ${ARDUINO_SDK_PATH}/hardware/tools/avr/utils)
else()
    message(FATAL_ERROR "Could not find Arduino SDK (set ARDUINO_SDK_PATH)!")
endif()
