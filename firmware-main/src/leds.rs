use embassy_nrf::gpio::{Output};
use embassy_time::{Duration, Timer, Ticker};

use crate::SharedExpander;


#[embassy_executor::task]
pub(crate) async fn leds_task(mut _led: Output<'static>, expander: &'static mut SharedExpander) {
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

    let mut ticker = Ticker::every(Duration::from_millis(1000));
    loop {
        {
            let mut expander = expander.lock().await;
            expander.write_rgb(9, RGB8 { r: 0, g: 31, b: 0 }).await.ok();
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
