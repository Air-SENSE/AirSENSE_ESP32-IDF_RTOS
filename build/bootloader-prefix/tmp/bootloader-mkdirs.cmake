# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION 3.5)

file(MAKE_DIRECTORY
  "E:/esp5.1/Espressif/frameworks/esp-idf-v5.1.2/components/bootloader/subproject"
  "E:/esp5.1/AirSENSE_ESP32-IDF_RTOS/build/bootloader"
  "E:/esp5.1/AirSENSE_ESP32-IDF_RTOS/build/bootloader-prefix"
  "E:/esp5.1/AirSENSE_ESP32-IDF_RTOS/build/bootloader-prefix/tmp"
  "E:/esp5.1/AirSENSE_ESP32-IDF_RTOS/build/bootloader-prefix/src/bootloader-stamp"
  "E:/esp5.1/AirSENSE_ESP32-IDF_RTOS/build/bootloader-prefix/src"
  "E:/esp5.1/AirSENSE_ESP32-IDF_RTOS/build/bootloader-prefix/src/bootloader-stamp"
)

set(configSubDirs )
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "E:/esp5.1/AirSENSE_ESP32-IDF_RTOS/build/bootloader-prefix/src/bootloader-stamp/${subDir}")
endforeach()
if(cfgdir)
  file(MAKE_DIRECTORY "E:/esp5.1/AirSENSE_ESP32-IDF_RTOS/build/bootloader-prefix/src/bootloader-stamp${cfgdir}") # cfgdir has leading slash
endif()
