//! Common functions.

use crate::{
    conversion, devices::OperatingMode, Ads1x1x, Ads1x1xPin, BitFlags, Config, Error, Register,
};

impl<I2C, IC, CONV, MODE, E> Ads1x1x<I2C, IC, CONV, MODE>
where
    I2C: embedded_hal::i2c::I2c<Error = E>,
{
    pub(super) fn write_register(&mut self, register: u8, data: u16) -> Result<(), Error<E>> {
        let data = data.to_be_bytes();
        let payload: [u8; 3] = [register, data[0], data[1]];
        self.i2c.write(self.address, &payload).map_err(Error::I2C)
    }

    pub(super) fn read_register(&mut self, register: u8) -> Result<u16, Error<E>> {
        let mut data = [0, 0];
        self.i2c
            .write_read(self.address, &[register], &mut data)
            .map_err(Error::I2C)
            .and(Ok(u16::from_be_bytes(data)))
    }

    pub(super) fn set_operating_mode(&mut self, mode: OperatingMode) -> Result<(), Error<E>> {
        let config = match mode {
            OperatingMode::OneShot => self.config.with_high(BitFlags::OP_MODE),
            OperatingMode::Continuous => self.config.with_low(BitFlags::OP_MODE),
        };
        self.write_register(Register::CONFIG, config.bits)?;
        self.config = config;
        Ok(())
    }

    /// Checks whether a measurement is currently in progress.
    pub fn is_measurement_in_progress(&mut self) -> Result<bool, Error<E>> {
        let config = Config {
            bits: self.read_register(Register::CONFIG)?,
        };
        Ok(!config.is_high(BitFlags::OS))
    }
}

impl<I2C, PIN, IC, CONV, MODE, E> Ads1x1xPin<I2C, PIN, IC, CONV, MODE>
where
    I2C: embedded_hal::i2c::I2c<Error = E>,
    PIN: embedded_hal_async::digital::Wait<Error = E>,
{
    /// Waits for a measurement to be ready.
    pub async fn wait_for_measurement(&mut self) -> Result<(), Error<E>> {
        if self.config.is_high(BitFlags::COMP_POL) {
            // active high
            self.alert_pin
                .wait_for_falling_edge()
                .await
                .map_err(Error::AlertPin)
        } else {
            // active low
            self.alert_pin
                .wait_for_rising_edge()
                .await
                .map_err(Error::AlertPin)
        }
    }
}

impl<I2C, PIN, IC, CONV, MODE, E> Ads1x1xPin<I2C, PIN, IC, CONV, MODE>
where
    I2C: embedded_hal::i2c::I2c<Error = E>,
    IC: crate::ic::Tier2Features,
    CONV: conversion::ConvertThreshold<E>,
{
    /// Creates a new driver with the alert pin in continuous mode.
    pub fn new(
        mut driver: Ads1x1x<I2C, IC, CONV, MODE>,
        alert_pin: PIN,
    ) -> nb::Result<Self, Error<E>> {
        driver.a_conversion_was_started = false;
        driver.use_alert_rdy_pin_as_ready()?;

        Ok(Ads1x1xPin { driver, alert_pin })
    }
}
