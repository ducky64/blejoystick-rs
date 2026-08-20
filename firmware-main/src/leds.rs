use embassy_nrf::gpio::{Output};
use embassy_time::{Duration, Timer, Ticker};

use crate::bus::GlobalBus;
use crate::SharedExpander;


#[embassy_executor::task]
pub(crate) async fn leds_task(bus: &'static GlobalBus, mut _led: Output<'static>, expander: &'static SharedExpander) {
    use smart_leds::RGB8;
    // initialize: clear all LEDs
    {
        let mut expander = expander.lock().await;
        for i in 0..12 {
            expander.write_rgb(i, RGB8 { r: 0, g: 0, b: 0 }).await.ok();
        }
        expander.update_rgb().await.ok();
    }

    // startup animation
    for i in 0..9 {
        {
            let mut expander = expander.lock().await;
            if i > 0 {
                expander.write_rgb(i - 1, RGB8 { r: 0, g: 0, b: 0 }).await.ok();
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

    let mut ticker = Ticker::every(Duration::from_millis(1000));
    loop {
        let soc = soc_recv.get().await;
        let voltage_rgb = if soc <= 20 {
            RGB8 { r: 31, g: 0, b: 0 }
        } else if soc < 50 {
            RGB8 { r: 15, g: 15, b: 0 }
        } else if soc < 75 {
            RGB8 { r: 15, g: 31, b: 0 }
        } else {
            RGB8 { r: 0, g: 31, b: 0 }
        };

        {
            let mut expander = expander.lock().await;
            expander.write_rgb(9, voltage_rgb).await.ok();
            expander.update_rgb().await.ok();
        }
        Timer::after(Duration::from_millis(5)).await;

        {
            let mut expander = expander.lock().await;
            expander.write_rgb(9, RGB8 { r: 0, g: 0, b: 0 }).await.ok();
            expander.update_rgb().await.ok();
        }
        
        ticker.next().await;
    }
}
