#![no_std]
#![no_main]

use core::panic::PanicInfo;

use embassy_executor::Spawner;

// TODO 1 - set this function as the main embassy-rs task
//          delete #[allow(unused)]
#[allow(unused)]
fn main(_spawner: Spawner) {
    // TODO 2 - init the RP2040

    // TODO 3 - init the GPIO pin used for the LED

    let mut value = 1;
    loop {
        value = 1 - value;
        // TODO 4 - write the value to the LED

        // TODO 5 - sleep
    }
}

#[panic_handler]
fn panic(_info: &PanicInfo) -> ! {
    loop {}
}
