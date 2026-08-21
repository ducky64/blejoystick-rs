use core::sync::atomic::Ordering;

use embassy_nrf::gpio::{Output};
use embassy_time::{Duration, Timer, Ticker};
use smart_leds::RGB8;

use crate::bus::GlobalBus;
use crate::SharedExpander;


fn soc_to_rgb(soc: u8) ->RGB8 {
    // convert SOC to RGB color
    if soc <= 20 {
        RGB8 { r: 31, g: 0, b: 0} // red
    } else if soc < 50 {
        RGB8 { r: 15, g: 15, b: 0} // yellow
    } else if soc < 75 {
        RGB8 { r: 15, g: 31, b: 0} // green-yellow
    } else {
        RGB8 { r: 0, g: 31, b: 0} // green
    }
}


#[embassy_executor::task]
pub(crate) async fn leds_task(bus: &'static GlobalBus, mut _led: Output<'static>, expander: &'static SharedExpander) {
    const RGB_ZERO : RGB8 = RGB8 { r: 0, g: 0, b: 0 };

    // initialize: clear all LEDs
    {
        let mut expander = expander.lock().await;
        for i in 0..12 {
            expander.write_rgb(i, RGB_ZERO).await.ok();
        }
        expander.update_rgb().await.ok();
    }

    // startup animation
    for i in 0..9 {
        {
            let mut expander = expander.lock().await;
            if i > 0 {
                expander.write_rgb(i - 1, RGB_ZERO).await.ok();
            }
            if i < 8 {
                expander.write_rgb(i, RGB8 { r: 0, g: 15, b: 15 }).await.ok();
            }
            expander.update_rgb().await.ok();
        }
        Timer::after(Duration::from_millis(50)).await;
    }

    Timer::after(Duration::from_millis(500)).await;

    let mut soc_recv = bus.vbat_soc.receiver().unwrap();

    loop {
        let soc = soc_recv.get().await;
        let rgb_soc = soc_to_rgb(soc);
        if bus.usb_powered.load(Ordering::Relaxed) {  // USB powered, keep LED on
            if bus.charging.load(Ordering::Relaxed) {  // charging, blink
                {
                    let mut expander = expander.lock().await;
                    expander.write_rgb(9, rgb_soc).await.ok();
                    expander.update_rgb().await.ok();
                }

                Timer::after(Duration::from_millis(500)).await;
                {
                    let mut expander = expander.lock().await;
                    expander.write_rgb(9, RGB_ZERO).await.ok();
                    expander.update_rgb().await.ok();
                }
                Timer::after(Duration::from_millis(500)).await;
            } else {  // not charging, solid
                let mut expander = expander.lock().await;
                expander.write_rgb(9, rgb_soc).await.ok();
                expander.update_rgb().await.ok();
                Timer::after(Duration::from_millis(500)).await;
            }
        } else { // battery powered, slow blink

            {
                let mut expander = expander.lock().await;
                expander.write_rgb(9, rgb_soc).await.ok();
                expander.update_rgb().await.ok();
            }
            Timer::after(Duration::from_millis(5)).await;

            {
                let mut expander = expander.lock().await;
                expander.write_rgb(9, RGB_ZERO).await.ok();
                expander.update_rgb().await.ok();
            }
            Timer::after(Duration::from_millis(2500)).await;
        }
    }
}
