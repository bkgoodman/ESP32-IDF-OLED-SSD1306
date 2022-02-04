
# SSD1306 OLED display for ESP32 - Espressif IDF development environment

(See the README.md file in the upper level 'examples' directory for more information about examples.)

#### Pin Assignment:

**Note:** The following pin assignments are used by default, yout can change these  in the `menuconfig` .

|                  | SDA    | SCL    |
| ---------------- | ------ | ------ |
| ESP32 I2C Master | GPIO5  | GPIO4  |

- master:
  - GPIO5 is assigned as the data signal of I2C master port
  - GPIO4 is assigned as the clock signal of I2C master port

### Configure the project

Open the project configuration menu (`idf.py menuconfig`). Then go into `Example Configuration` menu.

- In the `I2C Master` submenu, you can set the pin number of SDA/SCL according to your board. Also you can modify the I2C port number and freauency of the master.


### Build and Flash

Enter `idf.py -p PORT flash monitor` to build, flash and monitor the project.

(To exit the serial monitor, type ``Ctrl-]``.)

See the [Getting Started Guide](https://docs.espressif.com/projects/esp-idf/en/latest/get-started/index.html) for full steps to configure and use ESP-IDF to build projects.

