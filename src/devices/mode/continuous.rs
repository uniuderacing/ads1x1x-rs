//! Continuous measurement mode.

use crate::{
    conversion, devices::OperatingMode, mode, types::Ads1x1xPin, Ads1x1x, ChannelId, Error,
    ModeChangeError, Register,
};
use core::marker::PhantomData;

impl<I2C, IC, CONV, E> Ads1x1x<I2C, IC, CONV, mode::Continuous>
where
    I2C: embedded_hal::i2c::I2c<Error = E>,
    CONV: conversion::ConvertMeasurement,
{
    /// Changes to one-shot operating mode.
    pub fn into_one_shot(
        mut self,
    ) -> Result<Ads1x1x<I2C, IC, CONV, mode::OneShot>, ModeChangeError<E, Self>> {
        if let Err(Error::I2C(e)) = self.set_operating_mode(OperatingMode::OneShot) {
            return Err(ModeChangeError::I2C(e, self));
        }
        Ok(Ads1x1x {
            i2c: self.i2c,
            address: self.address,
            config: self.config,
            fsr: self.fsr,
            a_conversion_was_started: false,
            _conv: PhantomData,
            _ic: PhantomData,
            _mode: PhantomData,
        })
    }

    /// Reads the most recent measurement.
    pub fn read(&mut self) -> Result<i16, Error<E>> {
        let value = self.read_register(Register::CONVERSION)?;
        Ok(CONV::convert_measurement(value))
    }

    /// Selects the channel used for measurements.
    ///
    /// Note that when changing the channel in continuous conversion mode, the
    /// ongoing conversion will be completed.
    /// The following conversions will use the new channel configuration.
    #[allow(unused_variables)]
    pub fn select_channel<CH: ChannelId<Self>>(&mut self, channel: CH) -> Result<(), Error<E>> {
        let config = self.config.with_mux_bits(CH::channel_id());
        self.write_register(Register::CONFIG, config.bits)?;
        self.config = config;
        Ok(())
    }
}

impl<I2C, PIN, IC, CONV, E> Ads1x1xPin<I2C, PIN, IC, CONV, mode::Continuous>
where
    I2C: embedded_hal::i2c::I2c<Error = E>,
    IC: crate::ic::Tier2Features,
    CONV: conversion::ConvertThreshold<E>,
{
    /// Creates a new driver with the alert pin in continuous mode.
    pub fn new(
        mut driver: Ads1x1x<I2C, IC, CONV, mode::Continuous>,
        alert_pin: PIN,
    ) -> nb::Result<Self, Error<E>> {
        driver.a_conversion_was_started = false;
        driver.use_alert_rdy_pin_as_ready()?;

        Ok(Ads1x1xPin { driver, alert_pin })
    }
}

impl<I2C, PIN, IC, CONV, E> Ads1x1xPin<I2C, PIN, IC, CONV, mode::Continuous>
where
    I2C: embedded_hal::i2c::I2c<Error = E>,
    CONV: conversion::ConvertMeasurement,
    PIN: embedded_hal_async::digital::Wait<Error = E>,
    IC: crate::ic::Tier2Features,
{
    /// Changes to one-shot operating mode.
    #[allow(clippy::type_complexity)]
    pub fn into_continuous(
        mut self,
    ) -> Result<Ads1x1xPin<I2C, PIN, IC, CONV, mode::OneShot>, ModeChangeError<E, Self>> {
        self.driver.a_conversion_was_started = false;
        match self.driver.into_one_shot() {
            Ok(driver) => Ok(Ads1x1xPin {
                driver,
                alert_pin: self.alert_pin,
            }),
            Err(ModeChangeError::I2C(e, driver)) => Err(ModeChangeError::I2C(
                e,
                Ads1x1xPin {
                    driver,
                    alert_pin: self.alert_pin,
                },
            )),
        }
    }

    /// Waits for a measurement to be ready and reads it.
    pub async fn read(&mut self) -> Result<i16, Error<E>> {
        if self.config.is_high(crate::BitFlags::COMP_POL) {
            // active high
            self.alert_pin
                .wait_for_falling_edge()
                .await
                .map_err(Error::Pin)?;
        } else {
            // active low
            self.alert_pin
                .wait_for_rising_edge()
                .await
                .map_err(Error::Pin)?;
        }
        let value = self.read_register(Register::CONVERSION)?;
        Ok(CONV::convert_measurement(value))
    }
}
