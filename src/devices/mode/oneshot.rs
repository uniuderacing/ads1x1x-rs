//! One-shot measurement mode.

use core::marker::PhantomData;

use crate::{
    conversion, devices::OperatingMode, mode, types::Ads1x1xPin, Ads1x1x, BitFlags, ChannelId,
    Config, Error, ModeChangeError, Register,
};

impl<I2C, IC, CONV, E> Ads1x1x<I2C, IC, CONV, mode::OneShot>
where
    I2C: embedded_hal::i2c::I2c<Error = E>,
    CONV: conversion::ConvertMeasurement,
{
    /// Changes to continuous operating mode.
    pub fn into_continuous(
        mut self,
    ) -> Result<Ads1x1x<I2C, IC, CONV, mode::Continuous>, ModeChangeError<E, Self>> {
        if let Err(Error::I2C(e)) = self.set_operating_mode(OperatingMode::Continuous) {
            return Err(ModeChangeError::I2C(e, self));
        }
        Ok(Ads1x1x {
            i2c: self.i2c,
            address: self.address,
            config: self.config,
            fsr: self.fsr,
            a_conversion_was_started: true,
            _conv: PhantomData,
            _ic: PhantomData,
            _mode: PhantomData,
        })
    }

    fn trigger_measurement(&mut self, config: &Config) -> Result<(), Error<E>> {
        let config = config.with_high(BitFlags::OS);
        self.write_register(Register::CONFIG, config.bits)
    }
}

impl<I2C, IC, CONV, E> Ads1x1x<I2C, IC, CONV, mode::OneShot>
where
    I2C: embedded_hal::i2c::I2c<Error = E>,
    CONV: conversion::ConvertMeasurement,
{
    /// Requests that the ADC begins a conversion on the specified channel.
    ///
    /// The output value will be within `[2047..-2048]` for 12-bit devices
    /// (`ADS101x`) and within `[32767..-32768]` for 16-bit devices (`ADS111x`).
    /// The voltage that these values correspond to must be calculated using
    /// the full-scale range ([`FullScaleRange`](crate::FullScaleRange)) selected.
    ///
    /// Returns `nb::Error::WouldBlock` while a measurement is in progress.
    ///
    /// In case a measurement was requested and after is it is finished a
    /// measurement on a different channel is requested, a new measurement on
    /// using the new channel selection is triggered.
    #[allow(unused_variables)]
    pub fn read<CH: ChannelId<Self>>(&mut self, channel: CH) -> nb::Result<i16, Error<E>> {
        if self
            .is_measurement_in_progress()
            .map_err(nb::Error::Other)?
        {
            return Err(nb::Error::WouldBlock);
        }
        let config = self.config.with_mux_bits(CH::channel_id());
        let same_channel = self.config == config;
        if self.a_conversion_was_started && same_channel {
            // result is ready
            let value = self
                .read_register(Register::CONVERSION)
                .map_err(nb::Error::Other)?;
            self.a_conversion_was_started = false;
            return Ok(CONV::convert_measurement(value));
        }
        self.trigger_measurement(&config)
            .map_err(nb::Error::Other)?;
        self.config = config;
        self.a_conversion_was_started = true;
        Err(nb::Error::WouldBlock)
    }
}

impl<I2C, PIN, IC, CONV, E> Ads1x1xPin<I2C, PIN, IC, CONV, mode::OneShot>
where
    I2C: embedded_hal::i2c::I2c<Error = E>,
    IC: crate::ic::Tier2Features,
    CONV: conversion::ConvertThreshold<E>,
{
    /// Creates a new driver with the alert pin in one-shot mode.
    pub fn new(
        mut driver: Ads1x1x<I2C, IC, CONV, mode::OneShot>,
        alert_pin: PIN,
    ) -> nb::Result<Self, Error<E>> {
        driver.a_conversion_was_started = false;
        driver.use_alert_rdy_pin_as_ready()?;

        Ok(Ads1x1xPin { driver, alert_pin })
    }
}

impl<I2C, PIN, IC, CONV, E> Ads1x1xPin<I2C, PIN, IC, CONV, mode::OneShot>
where
    I2C: embedded_hal::i2c::I2c<Error = E>,
    CONV: conversion::ConvertMeasurement,
    PIN: embedded_hal_async::digital::Wait<Error = E>,
    IC: crate::ic::Tier2Features,
{
    /// Changes to continuous operating mode.
    #[allow(clippy::type_complexity)]
    pub fn into_continuous(
        self,
    ) -> Result<Ads1x1xPin<I2C, PIN, IC, CONV, mode::Continuous>, ModeChangeError<E, Self>> {
        match self.driver.into_continuous() {
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

    /// Triggers a measurement and waits for it to be ready.
    #[allow(unused_variables)]
    pub async fn read<CH: ChannelId<Self>>(&mut self, channel: CH) -> Result<i16, Error<E>> {
        let config = self.config.with_mux_bits(CH::channel_id());
        self.trigger_measurement(&config)?;

        self.config = config;

        if self.config.is_high(BitFlags::COMP_POL) {
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
