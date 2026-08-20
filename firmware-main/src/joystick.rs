use crate::bus::{GlobalBus, JoystickState};
use crate::prelude::*;

use fixed::types::{I16F16, I1F15, U0F16};

use embassy_nrf::gpio::{Input, Output};
use embassy_nrf::saadc::Saadc;
use embassy_time::{Duration, Timer, Ticker};

use crate::SharedExpander;


fn adc12_to_u0f16(adc: i16) -> U0F16 {
    // converts a unsigned raw 12-bit adc reading to U0F16, using the entire range
    U0F16::from_bits(((adc as u16) << 4) | ((adc as u16) >> 8))
}

fn scale_bipolar(adc: U0F16, center: U0F16, fullscale: U0F16, deadzone: U0F16) -> I1F15 {
    // takes a adc reading [0, 1) and maps it to a bipolar range [-1, 1), with a configurable center, full-scale, and deadzone
    // full-scale and deadzone are both half-span
    // convert everything to larger working type
    let adc = I16F16::from_num(adc);
    let center = I16F16::from_num(center);
    let fullscale = I16F16::from_num(fullscale);
    let deadzone = I16F16::from_num(deadzone);

    let scale_except_deadzone = fullscale - deadzone;
    let adc_offset = if adc > center + deadzone {
        adc - center - deadzone
    } else if adc < center - deadzone {
        adc - (center - deadzone)
    } else {
        I16F16::ZERO
    };
    I1F15::saturating_from_num(I16F16::saturating_div(adc_offset, scale_except_deadzone))
}

// the ADC API seems to be a bit of a dumpster fire
// https://github.com/esp-rs/esp-hal/issues/449
// such that it was removed int he embedded hal 1.0 API
// https://github.com/rust-embedded/embedded-hal/pull/376
#[embassy_executor::task]
pub(crate) async fn joystick_task(
    bus: &'static GlobalBus,
    mut stick_gate: Output<'static>,
    mut trig_gate: Output<'static>,
    mut adc: Saadc<'static, 3>,
    stick_sw: Input<'static>,
) {
    const CENTER_X: U0F16 = U0F16::lit("0.44");
    const CENTER_Y: U0F16 = U0F16::lit("0.48");
    const FULLSCALE_XY: U0F16 = U0F16::lit("0.40"); // half-span, including of deadzone
    const DEADZONE_XY: U0F16 = U0F16::lit("0.02");

    let josytick_state_sender = bus.joystick_state.sender();

    let mut ticker = Ticker::every(Duration::from_millis(20));

    loop {
        let mut buf = [0; 3];
        stick_gate.set_low();
        Timer::after_millis(3).await;
        trig_gate.set_low(); // A1304 has a shorter power on time
        Timer::after_micros(100).await;
        adc.sample(&mut buf).await;
        trig_gate.set_high();
        stick_gate.set_high();

        let x_adc = adc12_to_u0f16(buf[0]);
        let y_adc = adc12_to_u0f16(buf[1]);
        let x_linear = scale_bipolar(x_adc, CENTER_X, FULLSCALE_XY, DEADZONE_XY);
        let y_linear = scale_bipolar(y_adc, CENTER_Y, FULLSCALE_XY, DEADZONE_XY);

        let trig_adc = adc12_to_u0f16(buf[2]);
        let trig_linear = trig_adc
            .saturating_sub(U0F16::lit("0.55"))
            .saturating_div(U0F16::lit("0.2"));

        let btn_value = stick_sw.is_low();

        debug!(
            "JX {} {}    JY {} {}    Tr {} {}    Btn {}",
            x_adc, x_linear, y_adc, y_linear, trig_adc, trig_linear, btn_value,
        );

        let joystick_state = JoystickState {
            x: x_linear,
            y: y_linear,
            trig: I1F15::from_num(trig_linear),
            btn: false, //btn_value,
        };
        josytick_state_sender.send(joystick_state);

        ticker.next().await;
    }
}

#[embassy_executor::task]
pub(crate) async fn btns_task(
    bus: &'static GlobalBus,
    expander: &'static SharedExpander
) {
    let mut ticker = Ticker::every(Duration::from_millis(20));

    loop {
        let btns = {
            let mut expander = expander.lock().await;
            expander.read_btns().await.unwrap_or(0)
        };

        let rgb_val = if btns != 0 {
            smart_leds::RGB8 { r: 15, g: 0, b: 15 }
        } else {
            smart_leds::RGB8 { r: 0, g: 0, b: 0 }
        };
        {
            let mut expander = expander.lock().await;
            expander.write_rgb(2, rgb_val).await.ok();
            expander.update_rgb().await.ok();
        }

        ticker.next().await;
    }
}