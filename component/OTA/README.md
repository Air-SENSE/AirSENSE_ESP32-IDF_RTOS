# Overview

## Run HTTPS Server

After a successful build, we need to create a self-signed certificate and run a simple HTTPS server as follows:

![create_self_signed_certificate](https://dl.espressif.com/dl/esp-idf/docs/_static/ota_self_signature.gif)

* Enter the directory containing build artifact/s of project, that will be hosted by HTTPS server, e.g. `cd build`.
* To create a new self-signed certificate and key, run the command `openssl req -x509 -newkey rsa:2048 -keyout ca_key.pem -out ca_cert.pem -days 365 -nodes`.
  * When prompted for the `Common Name (CN)`, enter the name of the server that the "ESP-Dev-Board" will connect to. When running this example from a development machine, this is probably the IP address. The HTTPS client will check that the `CN` matches the address given in the HTTPS URL.
* To start the HTTPS server, run the command `openssl s_server -WWW -key ca_key.pem -cert ca_cert.pem -port 8070`.
* This directory should contain the firmware (e.g. `hello_world.bin`) to be used in the update process. This can be any valid ESP-IDF application, as long as its filename corresponds to the name configured using `Firmware Upgrade URL` in menuconfig. The only difference to flashing a firmware via the serial interface is that the binary is flashed to the `factory` partition, while OTA update use one of the OTA partitions.
* **Note:** Make sure incoming access to port *8070* is not prevented by firewall rules.
* **Note:** Windows users may encounter issues while running `openssl s_server -WWW`, due to CR/LF translation and/or closing the connection prematurely
  (Some windows builds of openssl translate CR/LF sequences to LF in the served files, leading to corrupted images received by the OTA client; others interpret the `0x1a`/`SUB` character in a binary as an escape sequence, i.e. end of file, and close the connection prematurely thus preventing the OTA client from receiving a complete image).
  * We recommend  using the `openssl` binary bundled in `Git For Windows` from the [ESP-IDF Tool installer](https://docs.espressif.com/projects/esp-idf/en/latest/get-started/windows-setup.html):
  Open the ESP-IDF command prompt and add the internal openssl binary to your path: `set PATH=%LocalAppData%\Git\usr\bin;%PATH%` and run openssl's http server command as above.
  * Alternatively, use any windows based openssl with version `v1.1.1i` or greater built on the `Msys-x86_64` platform, or a simple python https server -- see `start_https_server` in the [example_test](simple_ota_example/example_test.py) script.
