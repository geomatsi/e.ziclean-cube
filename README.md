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
| PA0 | analog input | ADC_IN0 | |
| PA1 | analog input | ADC_IN1 | |
| PA2 | analog input | ADC_IN2 | |
| PA3 | analog input | ADC_IN3 | |
| PA4 | analog input | ADC_IN4 | |
| PA5 | analog input | ADC_IN5 | |
| PA6 | analog input | ADC_IN6 | |
| PA7 | analog input | ADC_IN7 | |
| PA8 .. PA10 | floating input | | |
| PA11 | general purpose output (50MHz) push-pull | | |
| PA12 | floating input | | TM1667 STB |
| PA13 .. PA14 | input with pull-up/pull-down | | |
| PA15 | general purpose output (50MHz) push-pull | | |

#### GPIOB

| Pin | Configuration | Mapping | Function |
|-|-|-|-|
| PB0 | analog input | ADC_IN8 | |
| PB1 | analog input | ADC_IN9 | |
| PB2 | general purpose output (50MHz) push-pull | | KXCJ9 SDA |
| PB3 | floating input | | |
| PB4 | alternate function output (50MHz) push-pull | TIM3_CH1 | |
| PB5 | alternate function output (50MHz) push-pull | TIM3_CH2 | all brushes |
| PB6 | alternate function output (50MHz) push-pull | TIM4_CH1 | 74HC125D 2A |
| PB7 | alternate function output (50MHz) push-pull | TIM4_CH2 | 74HC125D 1A |
| PB8 | alternate function output (50MHz) push-pull | TIM4_CH3 | 74HC125D 4A |
| PB9 | alternate function output (50MHz) push-pull | TIM4_CH4 | 74HC125D 3A |
| PB10 | alternate function output (50MHz) push-pull | TIM2_CH3 | |
| PB11 | general purpose output (50MHz) open-drain | | |
| PB12 .. PB14 | general purpose output (50MHz) push-pull | | |
| PB15 | floating input | | |

#### GPIOC

| Pin | Configuration | Mapping | Function |
|-|-|-|-|
| PC0 | analog input | ADC_IN10 | |
| PC1 | analog input | ADC_IN11 | |
| PC2 | analog input | ADC_IN12 | |
| PC3 | analog input | ADC_IN13 | |
| PC4 | analog input | ADC_IN14 | |
| PC5 | analog input | ADC_IN15 | |
| PC6 | floating input | | |
| PC7 | general purpose output (50MHz) push-pull | | IR LEDs of all 5 front IR obstacle sensors |
| PC8 | general purpose output (50MHz) push-pull | | TM1667 CLK |
| PC9 .. PC15 | floating input | | |

#### GPIOD

| Pin | Configuration | Mapping | Function |
|-|-|-|-|
| PD0 | floating input | source input for EXTI0 | |
| PD1 | floating input | source input for EXIT1 | |
| PD2 | general purpose output (50MHz) open-drain | | |
| PD3 | floating input | | |
| PD4 | general purpose output (50MHz) push-pull | | |
| PD5 | general purpose output (50MHz) open-drain | | |
| PD6 .. PD8 | floating input | | |
| PD9 | general purpose output (50MHz) push-pull | | IR LEDs of all 3 bottom IR floor sensors |
| PD10 .. PD12 | floating input | | |
| PD13 | general purpose output (50MHz) open-drain | | |
| PD14 | general purpose output (50MHz) push-pull | | TM1667 DIO |
| PD15 | floating input | | |

#### GPIOE

| Pin | Configuration | Mapping | Function |
|-|-|-|-|
| PE0 | general purpose output (50MHz) push-pull | | Beeper |
| PE1 .. PE6 | floating input | | |
| PE7 | general purpose output (50MHz) push-pull | | KXCJ9 SCL |
| PE8 | floating input | | |
| PE9 | floating input | | KXCJ9 INT |
| PE10 .. PE11 | floating input | | |
| PE12 | general purpose output (50MHz) open-drain | | |
| PE13 | floating input | | |
| PE14 | general purpose output (50MHz) open-drain | | |
| PE15 | general purpose output (50MHz) push-pull | | |
