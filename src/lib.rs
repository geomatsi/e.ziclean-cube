#![no_std]

use stm32f1xx_hal as hal;

use embedded_hal::digital::v2::OutputPin;
use embedded_hal::PwmPin;

pub mod motion;
