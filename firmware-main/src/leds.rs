use embassy_time::{Duration, Timer};
use smart_leds::RGB8;

use embassy_futures::select::{self, select, select3};

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


const RGB_ZERO : RGB8 = RGB8 { r: 0, g: 0, b: 0 };


#[embassy_executor::task]
pub(crate) async fn battery_led_task(bus: &'static GlobalBus, expander: &'static SharedExpander) {
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

    let mut usb_powered_rcv = bus.usb_powered.receiver().unwrap();
    let mut charging_rcv = bus.charging.receiver().unwrap();

    loop {
        let soc = soc_recv.get().await;
        let rgb_soc = soc_to_rgb(soc);
        
        let blink_wait = if bus.usb_powered.try_get().unwrap_or(false) {  // USB powered, keep LED on
            if bus.charging.try_get().unwrap_or(false) {  // charging, blink
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
                500
            } else {  // not charging, solid
                let mut expander = expander.lock().await;
                expander.write_rgb(9, rgb_soc).await.ok();
                expander.update_rgb().await.ok();
                500
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
            2500
        };

        select3(
            Timer::after(Duration::from_millis(blink_wait)),
            usb_powered_rcv.changed(),
            charging_rcv.changed(),
        ).await;
    }
}


#[embassy_executor::task]
pub(crate) async fn io_led_task(bus: &'static GlobalBus, expander: &'static SharedExpander) {
    const RGB_PRESSED: RGB8 = RGB8 { r: 15, g: 31, b: 0 };

    Timer::after(Duration::from_millis(500)).await;  // wait for init animation from other task

    let mut btns_recv = bus.buttons_state.receiver().unwrap();
    let mut joystick_recv = bus.joystick_state.receiver().unwrap();

    let mut last_joystick_btn = false;
    let mut last_btns: u8 = 0;

    loop {
        match select(
            btns_recv.changed(),
            joystick_recv.changed(),
        ).await {
            select::Either::First(_) => {
                let btns = btns_recv.get().await;
                if btns == last_btns {
                    continue;  // button state unchanged, nothing to do
                }
                last_btns = btns;

                let mut expander = expander.lock().await;
                for (btn_i, led_i) in [(7, 1), (1, 7)] {
                    let rgb = if (btns & (1 << btn_i)) != 0 {
                        RGB_PRESSED
                    } else {
                        RGB_ZERO
                    };
                    expander.write_rgb(led_i, rgb).await.ok();
                }
                expander.update_rgb().await.ok();
            },
            select::Either::Second(_) => {
                let joystick = joystick_recv.get().await;
                if joystick.btn == last_joystick_btn {
                    continue;  // button state unchanged, nothign to do
                }
                last_joystick_btn = joystick.btn;

                let rgb = if joystick.btn {
                    RGB_PRESSED
                } else {
                    RGB_ZERO
                };
                let mut expander = expander.lock().await;
                expander.write_rgb(0, rgb).await.ok();
                expander.update_rgb().await.ok();
            },
        }

    }
}
