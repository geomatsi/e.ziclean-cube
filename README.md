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

### Runtime pin configuration

#### GPIOA

| Pin | Configuration | Mapping | Function |
|-|-|-|-|
| PA0 .. PA7 | analog input | | |
| PA8 .. PA10 | floating input | | |
| PA11 | general purpose output (50MHz) push-pull | | |
| PA12 | floating input | | |
| PA13 .. PA14 | input with pull-up/pull-down | | |
| PA15 | general purpose output (50MHz) push-pull | | |

#### GPIOB

| Pin | Configuration | Mapping | Function |
|-|-|-|-|
| PB0 .. PB1 | analog input | | |
| PB2 | general purpose output (50MHz) push-pull | | |
| PB3 | floating input | | |
| PB4 .. PB10 | alternate function output (50MHz) push-pull | | |
| PB11 | general purpose output (50MHz) open-drain | | |
| PB12 .. PB14 | general purpose output (50MHz) push-pull | | |
| PB15 | floating input | | |

#### GPIOC

| Pin | Configuration | Mapping | Function |
|-|-|-|-|
| PC0 .. PC5 | analog input | | |
| PC6 | floating input | | |
| PC7 .. PC8 | general purpose output (50MHz) push-pull | | |
| PC9 .. PC15 | floating input | | |

#### GPIOD

| Pin | Configuration | Mapping | Function |
|-|-|-|-|
| PD0 .. PD1 | floating input | | |
| PD2 | general purpose output (50MHz) open-drain | | |
| PD3 | floating input | | |
| PD4 | general purpose output (50MHz) push-pull | | |
| PD5 | general purpose output (50MHz) open-drain | | |
| PD6 .. PD8 | floating input | | |
| PD9 | general purpose output (50MHz) push-pull | | |
| PD10 .. PD12 | floating input | | |
| PD13 | general purpose output (50MHz) open-drain | | |
| PD14 | general purpose output (50MHz) push-pull | | |
| PD15 | floating input | | |

#### GPIOE

| Pin | Configuration | Mapping | Function |
|-|-|-|-|
| PE0 | general purpose output (50MHz) push-pull | | beeper |
| PE1 .. PE6 | floating input | | |
| PE7 | general purpose output (50MHz) push-pull | | |
| PE8 .. PE11 | floating input | | |
| PE12 | general purpose output (50MHz) open-drain | | |
| PE13 | floating input | | |
| PE14 | general purpose output (50MHz) open-drain | | |
| PE15 | general purpose output (50MHz) push-pull | | |

#### GPIOF


| Pin | Configuration | Mapping | Function |
|-|-|-|-|
| PF0 .. PF15 | input / analog | | |

#### GPIOG

| Pin | Configuration | Mapping | Function |
|-|-|-|-|
| PG0 .. PG15 | input / analog | | |



