use embedded_hal::digital::v2::OutputPin;
use embedded_hal::spi;

use nb::block;

use super::*;

/// Display
pub struct Display<SPI, STB>
where
    SPI: spi::FullDuplex<u8>,
    STB: OutputPin,
{
    /// spi interface
    spi: SPI,
    /// strob (chip select)
    stb: STB,
    /// display matrix data
    addr: [u8; 7],
    grid: [u8; 4],
    sym: [[[u8; 7]; 10]; 4],
}

/// Display errors
#[derive(Debug, Eq, PartialEq, Clone, Copy)]
pub enum Error {
    /// Invalid display position: should be 0..3
    InvalidPosition,
    /// Invalid symbol: should be 0..9
    InvalidSymbol,
    /// Number is too large, should be less than 9999
    InvalidNumber,
    /// Hardware failure
    HardwareError,
}

impl<SPI, STB> Display<SPI, STB>
where
    SPI: spi::FullDuplex<u8>,
    STB: OutputPin,
{
    pub fn init(spi: SPI, stb: STB) -> Self {
        // e.ziclean display connected to TM1668 so that address byte varies
        // while data byte is fixes for each grid
        let addr = [0x0, 0x2, 0x4, 0x6, 0x8, 0xa, 0xc];
        let grid = [0x08, 0x80, 0x40, 0x20];

        // segment 'bitmaps' for digits in all 4 grids
        let sym = [
            [
                [1, 1, 1, 1, 1, 1, 0],
                [1, 0, 0, 0, 0, 1, 0],
                [1, 1, 0, 1, 1, 0, 1],
                [1, 1, 0, 0, 1, 1, 1],
                [1, 0, 1, 0, 0, 1, 1],
                [0, 1, 1, 0, 1, 1, 1],
                [0, 1, 1, 1, 1, 1, 1],
                [1, 1, 0, 0, 0, 1, 0],
                [1, 1, 1, 1, 1, 1, 1],
                [1, 1, 1, 0, 1, 1, 1],
            ],
            [
                [1, 1, 1, 1, 0, 1, 1],
                [0, 1, 0, 0, 0, 1, 0],
                [0, 1, 1, 1, 1, 0, 1],
                [0, 1, 1, 1, 1, 1, 0],
                [1, 1, 0, 0, 1, 1, 0],
                [1, 0, 1, 1, 1, 1, 0],
                [1, 0, 1, 1, 1, 1, 1],
                [0, 1, 1, 0, 0, 1, 0],
                [1, 1, 1, 1, 1, 1, 1],
                [1, 1, 1, 1, 1, 1, 0],
            ],
            [
                [1, 1, 1, 1, 1, 1, 0],
                [1, 0, 0, 0, 0, 1, 0],
                [1, 1, 0, 1, 1, 0, 1],
                [1, 1, 0, 0, 1, 1, 1],
                [1, 0, 1, 0, 0, 1, 1],
                [0, 1, 1, 0, 1, 1, 1],
                [0, 1, 1, 1, 1, 1, 1],
                [1, 1, 0, 0, 0, 1, 0],
                [1, 1, 1, 1, 1, 1, 1],
                [1, 1, 1, 0, 1, 1, 1],
            ],
            [
                [1, 1, 1, 1, 1, 0, 1],
                [0, 1, 0, 0, 1, 0, 0],
                [0, 1, 1, 1, 0, 1, 1],
                [0, 1, 1, 1, 1, 1, 0],
                [1, 1, 0, 0, 1, 1, 0],
                [1, 0, 1, 1, 1, 1, 0],
                [1, 0, 1, 1, 1, 1, 1],
                [0, 1, 1, 0, 1, 0, 0],
                [1, 1, 1, 1, 1, 1, 1],
                [1, 1, 1, 1, 1, 1, 0],
            ],
        ];

        let mut disp = Display {
            spi,
            stb,
            addr,
            grid,
            sym,
        };

        // display mode setting: 7 grids, 11 segments
        disp.write(0b0000_0011).ok();

        // data setting: normal mode, fixed addr, write data to display
        disp.write(0b0100_0100).ok();

        // display control: display ON, PWM 13/16
        disp.write(0b1000_1010).ok();

        disp
    }

    fn write(&mut self, data: u8) -> Result<(), Error> {
        self.stb.set_low().map_err(|_| Error::HardwareError)?;
        // FIXME: error handling
        block!(self.spi.send(data)).ok();
        self.stb.set_high().map_err(|_| Error::HardwareError)?;
        Ok(())
    }

    fn write2(&mut self, data1: u8, data2: u8) -> Result<(), Error> {
        self.stb.set_low().map_err(|_| Error::HardwareError)?;
        // FIXME: error handling
        block!(self.spi.send(data1)).ok();
        // FIXME: error handling
        block!(self.spi.send(data2)).ok();
        self.stb.set_high().map_err(|_| Error::HardwareError)?;
        Ok(())
    }

    pub fn clear(&mut self) -> Result<(), Error> {
        for i in 0..7 {
            self.write2(0b1100_0000 | self.addr[i], 0b0000_0000)?;
        }

        Ok(())
    }

    pub fn print(&mut self, pos: u8, num: u8) -> Result<(), Error> {
        if pos > 3 {
            return Err(Error::InvalidPosition);
        }

        if num > 9 {
            return Err(Error::InvalidSymbol);
        }

        for i in 0..7 {
            let data = self.grid[pos as usize] * self.sym[pos as usize][num as usize][i];
            self.write2(0b1100_0000 | self.addr[i], data)?;
        }

        Ok(())
    }

    pub fn print4(&mut self, n: [u8; 4]) -> Result<(), Error> {
        if n[0] > 9 || n[1] > 9 || n[2] > 9 || n[3] > 9 {
            return Err(Error::InvalidSymbol);
        }

        for i in 0..7 {
            let mut data = 0;

            for (k, v) in n.iter().enumerate() {
                data |= self.grid[k] * self.sym[k][*v as usize][i];
            }

            self.write2(0b1100_0000 | self.addr[i], data)?;
        }

        Ok(())
    }

    pub fn print_time(&mut self, t: [u8; 2], colon: bool) -> Result<(), Error> {
        if t[0] > 24 || t[1] > 60 {
            return Err(Error::InvalidSymbol);
        }

        let n = [
            t[0] / 10,
            t[0] - (t[0] / 10) * 10,
            t[1] / 10,
            t[1] - (t[1] / 10) * 10,
        ];

        for i in 0..7 {
            let mut data = 0;

            for (k, v) in n.iter().enumerate() {
                data |= self.grid[k] * self.sym[k][*v as usize][i];
            }

            if self.addr[i] == 0x06 && colon {
                data |= 2;
            }

            self.write2(0b1100_0000 | self.addr[i], data)?;
        }

        Ok(())
    }

    pub fn print_num(&mut self, num: u16) -> Result<(), Error> {
        if num > 9999 {
            return Err(Error::InvalidNumber);
        }

        let mut n = [0; 4];

        n[0] = num / 1000;
        n[1] = (num - n[0] * 1000) / 100;
        n[2] = (num - n[0] * 1000 - n[1] * 100) / 10;
        n[3] = num - n[0] * 1000 - n[1] * 100 - n[2] * 10;

        for i in 0..7 {
            let mut data = 0;

            for (k, v) in n.iter().enumerate() {
                data |= self.grid[k] * self.sym[k][*v as usize][i];
            }

            self.write2(0b1100_0000 | self.addr[i], data)?;
        }

        Ok(())
    }
}
