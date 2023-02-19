# AirSENSE-ESP32-IDF

This is main development repo for ESP32 device in 20221 semester - a simple buildable esp-idf example. For more information please contact [Nhu Hai Long Nguyen](https://www.facebook.com/long.hai.14203544)


## :rocket: Features

**Free and Portable**
  - Support platformIO on Visual Studio Code.
  - Can be compiled to ESP32 for running freeRTOS OS.
  - Distributed under the MIT licence, so you can easily use it in commercial projects too.

**Docs, Tools, and Services**
  - We are writing about documents. 

## :heart: Sponsor

**AirSENSE - Facebook**<br>
[AirSENSE](https://opencollective.com/lvgl)

**AirSENSE - Another Version**<br>
[Another repositories](https://github.com/orgs/Air-SENSE/repositories)


## How to use example
We encourage the users to use the example as a template for the new projects.
A recommended way is to follow the instructions on a [docs page](https://docs.espressif.com/projects/esp-idf/en/latest/api-guides/build-system.html#start-a-new-project).

## Example folder contents

The project **sample_project** contains one source file in C language [main.c](main/main.c). The file is located in folder [main](main).

ESP-IDF projects are built using CMake. The project build configuration is contained in `CMakeLists.txt`
files that provide set of directives and instructions describing the project's source files and targets
(executable, library, or both). 

Below is short explanation of remaining files in the project folder.

```
├── CMakeLists.txt
├── main
│   ├── CMakeLists.txt
│   └── main.c
└── README.md                  This is the file you are currently reading
```
Additionally, the sample project contains Makefile and component.mk files, used for the legacy Make based build system. 
They are not used or needed when building with CMake and idf.py.
