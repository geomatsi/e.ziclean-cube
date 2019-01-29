# `e.ziclean cube`
Experiments with custom firmware for [E.ZICLEAN CUBE](https://www.e-zicom.com/aspirateur-robot-eziclean-cube.html) robot vacuum cleaner.

## Hardware notes
### Board
![alt text](pics/board-p1.jpg)

### Components
* Microcontroller STM32f101VBT6
* Accelerometer KXCJ9
* Operational amplifier (3 pcs) LM324
* Quad buffer/line driver 74HC125D
* Display driver TM1668

### Connector pinout
| VDD (75) | TMS/SWD (72) | GND | TDI (77) | NTRST (90)|
|-|-|-|-|-|
|__TX (68)__| __RX (69)__ | __TDO (89)__ | __TCK/SWCLK (76)__ | __NRST (14)__|

## Firmware notes
