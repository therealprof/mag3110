//! A driver for the NXP MAG3110 magnetometer
//!
//! This driver was built using [`embedded-hal`] traits.
//!
//! [`embedded-hal`]: https://docs.rs/embedded-hal/~0.1
//!
//! # Examples
//!
//! You should find at least one example in the [microbit] crate.
//!
//! [microbit]: https://github.com/therealprof/microbit

#![deny(warnings)]
#![no_std]

extern crate cast;
extern crate embedded_hal as hal;

use core::mem;

use cast::u16;
use hal::blocking::i2c::{Write, WriteRead};

pub const ADDRESS: u8 = 0x0E;

#[allow(dead_code)]
#[allow(non_camel_case_types)]
#[derive(Copy, Clone)]
pub enum Register {
    DR_STATUS = 0x00, // Data ready status per axis
    OUT_X_MSB = 0x01, // Bits [15:8] of X measurement
    OUT_X_LSB = 0x02, // Bits [7:0] of X measurement
    OUT_Y_MSB = 0x03, // Bits [15:8] of Y measurement
    OUT_Y_LSB = 0x04, // Bits [7:0] of Y measurement
    OUT_Z_MSB = 0x05, //Bits [15:8] of Z measurement
    OUT_Z_LSB = 0x06, // Bits [7:0] of Z measurement
    WHO_AM_I = 0x07,  // Device ID Number
    SYSMOD = 0x08,    // Current System Mode
    OFF_X_MSB = 0x09, // Bits [14:7] of user X offset
    OFF_X_LSB = 0x0A, // Bits [6:0] of user X offset
    OFF_Y_MSB = 0x0B, // Bits [14:7] of user Y offset
    OFF_Y_LSB = 0x0C, // Bits [6:0] of user Y offset
    OFF_Z_MSB = 0x0D, // Bits [14:7] of user Z offset
    OFF_Z_LSB = 0x0E, // Bits [6:0] of user Z offset
    DIE_TEMP = 0x0F,  // Temperature, signed 8 bits in C
    CTRL_REG1 = 0x10, // Operation modes
    CTRL_REG2 = 0x11, // Operation modes
}

impl Register {
    pub fn addr(&self) -> u8 {
        *self as u8
    }
}

#[allow(dead_code)]
#[derive(Copy, Clone)]
pub enum DataRate {
    HZ80 = 0b0000_0000,   // 80Hz, maximum speed at 16 times oversampling
    HZ40 = 0b0010_0000,   // 40Hz
    HZ20 = 0b0100_0000,   // 20Hz
    HZ10 = 0b0110_0000,   // 10Hz
    HZ5 = 0b1000_0000,    // 5Hz
    HZ2_5 = 0b1010_0000,  // 2.5Hz
    HZ1_25 = 0b1100_0000, // 1.25Hz
    HZ0_63 = 0b1110_0000, // 0.63Hz
}

impl DataRate {
    pub fn bits(&self) -> u8 {
        *self as u8
    }
}

#[allow(dead_code)]
#[derive(Copy, Clone)]
pub enum Oversampling {
    OV16 = 0b0000_0000,  // 16 times oversampling, maximum speed at 80Hz
    OV32 = 0b0000_1000,  // 32 times oversampling
    OV64 = 0b0001_0000,  // 64 times oversampling
    OV128 = 0b0001_1000, // 128 times oversampling
}

impl Oversampling {
    pub fn bits(&self) -> u8 {
        *self as u8
    }
}

pub struct Mag3110<I2C> {
    i2c: I2C,
}

impl<I2C, E> Mag3110<I2C>
where
    I2C: WriteRead<Error = E> + Write<Error = E>,
{
    pub fn new(i2c: I2C) -> Result<Self, E> {
        let mut mag3110 = Self { i2c };

        /* Initialise with highest sampling rate, lowest oversampling and continous measurement */
        mag3110.set_sampling_mode(DataRate::HZ80, Oversampling::OV16)?;

        Ok(mag3110)
    }

    pub fn set_sampling_mode(&mut self, dr: DataRate, ov: Oversampling) -> Result<(), E> {
        /* Stop sampling */
        let _ = self.stop_sampling()?;

        /* Restart sampling with new settings */
        self.write_register(Register::CTRL_REG1, dr.bits() | ov.bits() | 1)?;
        Ok(())
    }

    pub fn mag(&mut self) -> Result<(i16, i16, i16), E> {
        let mut buffer: [u8; 6] = unsafe { mem::uninitialized() };
        self.i2c
            .write_read(ADDRESS, &[Register::OUT_X_MSB.addr()], &mut buffer)?;

        Ok((
            (u16(buffer[1]) | (u16(buffer[0]) << 8)) as i16,
            (u16(buffer[3]) | (u16(buffer[2]) << 8)) as i16,
            (u16(buffer[5]) | (u16(buffer[4]) << 8)) as i16,
        ))
    }

    pub fn temp(&mut self) -> Result<i8, E> {
        let mut buffer: [u8; 1] = unsafe { mem::uninitialized() };
        self.i2c
            .write_read(ADDRESS, &[Register::DIE_TEMP.addr()], &mut buffer)?;

        Ok(buffer[0] as i8)
    }

    pub fn stop_sampling(&mut self) -> Result<(), E> {
        loop {
            /* Stop sampling */
            self.write_register(Register::CTRL_REG1, 0)?;

            /* Read mode register so we know it has settled */
            let mut buffer: [u8; 1] = unsafe { mem::uninitialized() };
            self.i2c
                .write_read(ADDRESS, &[Register::SYSMOD.addr()], &mut buffer)?;

            if buffer[0] == 0 {
                break Ok(());
            }
        }
    }

    fn write_register(&mut self, reg: Register, byte: u8) -> Result<(), E> {
        self.i2c.write(ADDRESS, &[reg.addr(), byte])
    }
}
