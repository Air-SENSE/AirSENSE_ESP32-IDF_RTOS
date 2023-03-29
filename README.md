# AirSENSE-ESP32-IDF

This is main development repo for ESP32 device in 20221 semester - a simple buildable esp-idf example. For more information please contact [Nhu Hai Long Nguyen](https://www.linkedin.com/in/hải-long-nguyễn-như-95b718207) or [Gmail]:long27032002@gmaill.com


## :rocket: Features

**Feature Rich**<br>
It has all the features which you need in a IOT system: Read Sensor data, save data to micro SD card, publish data to MQTT server... To integrate AirSENSE into your project, all you need is an ESP32 Board, a computer with Visual Studio Code to run PlatformIO, usb capble for flashing.

**Free and Portable**
  - Support platformIO on Visual Studio Code.
  - Can be compiled to ESP32 for running freeRTOS OS.
  - Distributed under the MIT licence, so you can easily use it in commercial projects too.
  - Using ESP-IDF (In this project we use esp-idf version 4.4.3).

**Docs, Tools, and Services**
  - We are writing about documents. 

## :heart: Sponsor

**AirSENSE - Facebook**<br>
[AirSENSE](https://www.facebook.com/airsenseairqualitymornitoringsystem)

**AirSENSE - Another Version**<br>
[Another repositories](https://github.com/orgs/Air-SENSE/repositories)

## How to use project
We encourage the users to use the example as a template for the new projects.
A recommended way is to follow the instructions on a [docs page](https://docs.espressif.com/projects/esp-idf/en/latest/api-guides/build-system.html#start-a-new-project).

**Project Structure**<br>
Below is short explanation of remaining files in the project folder.

```
├── component                   This is the folder store all component of project
│   ├─ BME280
│   │  ├─ bme280.h
│   │  ├─ bme280.c
│   │  ├─ CmakeLists.txt
│   │  ├─ component.mk
│   │  └─ Kconfig.projbuild
│   │
│   ├─ Buttons
│   ├─ DataManager
│   ├─ DS3231
│   ├─ Filemanager
│   ├─ MHZ14a
│   ├─ OTA
│   ├─ PMS7003
│   └─ SNTP_Sync
│
├── main
│   ├── CMakeLists.txt
│   ├── component.mk
│   ├── Kconfig.projbuild
│   └── main.c
│
├── server_certs                This is the folder store file .pem for update OTA
│
├── Test                        This is the folder store test tool(python file)
│
├── CmakeLists.txt
├── LICENSE
├── Makefile
└── README.md                   This is the file you are currently reading
```
Note:
 - Kconfig.projbuild is file to config number pin GPIO, parameters, frequency,... these values can be changed. Run the command `idf.py menuconfig` in the ESP-IDF terminal to change it to suit your hardware.
 - In this project, we used to use some libraries of github account: "UncleRus".


## :star2: Contributing
AirSENSE is an open project and contribution is very welcome. There are many ways to contribute from simply speaking about your project, through writing examples, improving the documentation, fixing bugs or even hosting your own project under the AirSENSE organization.


Alot of people already left their fingerprint in AirSENSE. Be one them! See your here! :slightly_smiling_face:

<a href="https://github.com/Air-SENSE/AirSENSE_ESP32-IDF_RTOS/graphs/contributors">
  <img src="https://contrib.rocks/image?repo=Air-SENSE/AirSENSE_ESP32-IDF_RTOS&max=48" />
</a>

... and many other.
