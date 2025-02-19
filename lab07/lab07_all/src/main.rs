#![no_std]
#![no_main]
#![allow(unused_imports, dead_code, unused_variables, unused_mut)]

use core::panic::PanicInfo;
use embassy_executor::Spawner;
//gpio
use embassy_rp::gpio::{Input, Output, Level, Pin, Pull};
use embassy_rp::pwm::{Config as PwmConfig, Pwm};
//usb driver
use embassy_rp::usb::{Driver, Endpoint, InterruptHandler as USBInterruptHandler};
use embassy_rp::{bind_interrupts, peripherals::USB};
use log::info;
use embassy_time::{Timer, Duration};
// I2C
use embassy_rp::i2c::{Config as I2cConfig, I2c, InterruptHandler as I2CInterruptHandler};
use embassy_rp::peripherals::I2C0;
use embassy_rp::peripherals::I2C1;
use embedded_hal_async::i2c::I2c as _;
use embassy_rp::adc::{Adc, Channel, Config as AdcConfig, InterruptHandler as AdcInterruptHandler};
//wifi 
use core::str::from_utf8;

use byte_slice_cast::AsByteSlice;
use cyw43_pio::PioSpi;
use embassy_futures::select::select;
use embassy_net::tcp::TcpSocket;
use embassy_net::udp::{PacketMetadata, UdpSocket};
use embassy_net::{Config, IpAddress, IpEndpoint, Ipv4Address, Ipv4Cidr, Stack, StackResources};
use embassy_rp::peripherals::{DMA_CH0, PIO0};
use embassy_rp::pio::{InterruptHandler, Pio};
use embedded_io_async::Write;
use heapless::Vec;
use log::{ warn};
use static_cell::StaticCell;



fn calculate_temperature(temperature_raw: u32) -> i32 {
    let var1: i32 = ((temperature_raw as i32 >> 3) - (27504 << 1)) * (26435 >> 11);
    let var2: i32 = ((temperature_raw as i32 >> 4) - 27504)
        * (((temperature_raw as i32 >> 4) - 27504) >> 12)
        * (-1000 >> 14);
    ((var1 + var2) * 5 + 128) >> 8
}

bind_interrupts!(struct Irqs {
    USBCTRL_IRQ => USBInterruptHandler<USB>;
    ADC_IRQ_FIFO => AdcInterruptHandler;
    I2C0_IRQ => I2CInterruptHandler<I2C0>;
    I2C1_IRQ => I2CInterruptHandler<I2C1>;
    PIO0_IRQ_0 => InterruptHandler<PIO0>;
});

const WIFI_NETWORK: &str = "hdsiph";
const WIFI_PASSWORD: &str = "12345678";


#[embassy_executor::task]
async fn logger_task(driver: Driver<'static, USB>) {
    embassy_usb_logger::run!(1024, log::LevelFilter::Info, driver);
}

#[embassy_executor::task]
async fn wifi_task(runner: cyw43::Runner<'static, Output<'static>, PioSpi<'static, PIO0, 0, DMA_CH0>>) -> ! {
    runner.run().await
}

#[embassy_executor::task]
async fn net_task(stack: &'static Stack<cyw43::NetDriver<'static>>) -> ! {
    stack.run().await
}

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let peripherals = embassy_rp::init(Default::default());

    let driver = Driver::new(peripherals.USB, Irqs);
    spawner.spawn(logger_task(driver)).unwrap();


    // LED rosu
    let mut ledr = Output::new(peripherals.PIN_8, Level::Low);
    

    // LED verde1
    let mut ledv1 = Output::new(peripherals.PIN_7, Level::Low);

    // LED verde2
    let mut ledv2 = Output::new(peripherals.PIN_4, Level::Low);
   

    // LED galben
    let mut ledg = Output::new(peripherals.PIN_5, Level::Low);
    

    //BUZZER
    let mut buzzer = Output::new(peripherals.PIN_1, Level::Low);
    // let mut config_pwm: PwmConfig = Default::default();
    // config_pwm.top=0xFFFF;
    // config_pwm.compare_a=0;
    // let mut pwm_buzzer=Pwm::new_output_b(peripherals.PWM_SLICE0, peripherals.PIN_1, config_pwm.clone());
    
    
    // SENZOR TEMPERATURA
    let sda = peripherals.PIN_20;
    let scl = peripherals.PIN_21; 
 
    let mut i2c = I2c::new_async(&peripherals.I2C0, scl, sda, Irqs, I2cConfig::default());

    const BMP280_ADDR: u16 = 0x76;
 
    const REG_ADDR_ID: u8 = 0xD0;
    const REG_ADDR_CTRL_MEAS: u8 = 0xF4;
    const REG_ADDR_PRESS_MSB: u8 = 0xF7;
    const REG_ADDR_TEMP_MSB: u8 = 0xFA;
 
    let tx_buf = [REG_ADDR_ID]; 
    let mut rx_buf = [0x00u8];
 
    i2c.write_read(BMP280_ADDR, &tx_buf, &mut rx_buf).await.unwrap(); 
    let register_value = rx_buf[0];

    // SENZOR LUMINOZITATE
    let mut adc = Adc::new(peripherals.ADC, Irqs, AdcConfig::default());
    let mut luminosity_sensor = Channel::new_pin(peripherals.PIN_27, Pull::None);

    // SENZOR UMIDITATE SOL
    // Define ADC
    let mut soil_moist_sensor = Channel::new_pin(peripherals.PIN_26, Pull::None);

    // POMPA APA
    let mut pompa = Output::new(peripherals.PIN_0, Level::Low);

    

    // constante pentru planta folosita: dracena
    const TEMP_MIN: i32 = 18;
    const TEMP_MAX: i32 = 24;
    const LUM_MIN: u16 = 1638;
    const LUM_MAX: u16 = 2457;
    const UMID_MIN: u16 = 2718;
    const UMID_MAX: u16 = 3177;



///////////////////////////////////////////////////////////
//  // Link CYW43 firmware
//  let fw = include_bytes!("../../../cyw43-firmware/43439A0.bin");
//  let clm = include_bytes!("../../../cyw43-firmware/43439A0_clm.bin");

//  // Init SPI for communication with CYW43
//  let pwr = Output::new(peripherals.PIN_23, Level::Low);
//  let cs = Output::new(peripherals.PIN_25, Level::High);
//  let mut pio = Pio::new(peripherals.PIO0, Irqs);
//  let spi = PioSpi::new(
//      &mut pio.common,
//      pio.sm0,
//      pio.irq0,
//      cs,
//      peripherals.PIN_24,
//      peripherals.PIN_29,
//      peripherals.DMA_CH0,
//  );

//  // Start Wi-Fi task
//  static STATE: StaticCell<cyw43::State> = StaticCell::new();
//  let state = STATE.init(cyw43::State::new());
//  let (net_device, mut control, runner) = cyw43::new(state, pwr, spi, fw).await;
//  spawner.spawn(wifi_task(runner)).unwrap();

//  // Init the device
//  control.init(clm).await;
//  control
//      .set_power_management(cyw43::PowerManagementMode::PowerSave)
//      .await;

//  let config = Config::dhcpv4(Default::default());

//  // Generate random seed
//  let seed = 0x0123_4567_89ab_cdef;

//  // Init network stack
//  static STACK: StaticCell<Stack<cyw43::NetDriver<'static>>> = StaticCell::new();
//  static RESOURCES: StaticCell<StackResources<2>> = StaticCell::new();
//  let stack = &*STACK.init(Stack::new(
//      net_device,
//      config,
//      RESOURCES.init(StackResources::<2>::new()),
//      seed,
//  ));

//  // Start network stack task
//  spawner.spawn(net_task(stack)).unwrap();

//  loop {
//     // Join WPA2 access point
//     match control.join_wpa2(WIFI_NETWORK, WIFI_PASSWORD).await {
//         Ok(_) => break,
//         Err(err) => {
//             info!("join failed with status {}", err.status);
//         }
//     }
// }

// // Wait for DHCP
// info!("waiting for DHCP...");
// while !stack.is_config_up() {
//     Timer::after_millis(100).await;
// }
// info!("DHCP is now up {:?}!", stack.config_v4());

// // Create buffers for UDP
// let mut rx_buffer = [0; 4096];
// let mut rx_metadata_buffer = [PacketMetadata::EMPTY; 3];
// let mut tx_buffer = [0; 4096];
// let mut tx_metadata_buffer = [PacketMetadata::EMPTY; 3];

// let mut buf = [0u8; 4096];

// loop {
//     let mut socket = UdpSocket::new(
//         stack,
//         &mut rx_metadata_buffer,
//         &mut rx_buffer,
//         &mut tx_metadata_buffer,
//         &mut tx_buffer,
//     );

//     info!("Starting server on UDP:1234...");

//     // Bind socket to port
//     if let Err(e) = socket.bind(1234) {
//         warn!("accept error: {:?}", e);
//         continue;
//     }

//     control.gpio_set(0, true).await; // this is necessary!

//     let mut button = Input::new(peripherals.PIN_15, Pull::Up);
///////////////////////////////////////////////////
    loop {



            
        //senzor luminozitate
        let luminozitate = adc.read(&mut luminosity_sensor).await.unwrap();
        info!("-------------------------------------");
        info!("Luminosity: {}", luminozitate);
        Timer::after_secs(1).await;
        if luminozitate < LUM_MIN {
            info!("LUMINOSITY IS BELOW MINIMUM! HELP THE PLANT!");
        } else if luminozitate > LUM_MAX {
            info!("LUMINOSITY IS ABOVE MAXIMUM! GIVE THE PLANT SOME SUNGLASSES AND SUNSCREEN!");
        }
        else {
            info!("Good job, the plant is happily lit!");
        }


        //senzor umiditate sol
        let level = adc.read(&mut soil_moist_sensor).await.unwrap();
        info!("-------------------------------------");
        info!("Soil moisture: {}", level);
        Timer::after_secs(1).await;
        if level > UMID_MIN {
            info!("SOIL MOISTURE IS BELOW MINIMUM! WATER THE PLANT!");
        } else if level < UMID_MAX{
            info!("SOIL MOISTURE IS ABOVE MAXIMUM! deWATER THE PLANT!");
        }else {
            info!("Good job, the plant is better hydrated than yor skin!");
        }



        // senzor temperatura si presiune
        let tx_buf = [REG_ADDR_CTRL_MEAS,  0b001_001_11];    

        i2c.write(BMP280_ADDR, &tx_buf).await.unwrap();
        let tx_buf_press = [REG_ADDR_PRESS_MSB];
        let mut rx_buf_press = [0x00u8; 3];

        i2c.write_read(BMP280_ADDR, &tx_buf_press, &mut rx_buf_press).await.unwrap();

        let press_msb = rx_buf_press[0] as u32;
        let press_lsb = rx_buf_press[1] as u32;
        let press_xlsb = rx_buf_press[2] as u32;

        let pressure_raw = (press_msb << 12) + (press_lsb << 4) + (press_xlsb >> 4);
        info!("-------------------------------------");
        info!("Raw pressure: {pressure_raw}");

        let tx_buf_temp = [REG_ADDR_TEMP_MSB];
        let mut rx_buf_temp = [0x00u8; 3];

        i2c.write_read(BMP280_ADDR, &tx_buf_temp, &mut rx_buf_temp).await.unwrap();
        let temp_msb = rx_buf_temp[0] as u32;
        let temp_lsb = rx_buf_temp[1] as u32;
        let temp_xlsb = rx_buf_temp[2] as u32;

        let temperature_raw = (temp_msb << 12) + (temp_lsb << 4) + (temp_xlsb >> 4);// modify

        let temperature = calculate_temperature(temperature_raw as u32);
        let real_temp=temperature/100;
        let real_temp_dec=temperature%100;
        info!("Temperature: {}.{}",
        real_temp, real_temp_dec);

        if real_temp < TEMP_MIN {
            info!("TEMPERATURE IS BELOW MINIMUM! TURN ON THE RADIATOR!");
        } else if real_temp > TEMP_MAX {
            info!("TEMPERATURE IS ABOVE MAXIMUM! TURN ON THE AC!");
        }
        else {
            info!("Good job, the plant is in a cozy environment!");
        }



        // if uri
        if real_temp < TEMP_MIN || real_temp > TEMP_MAX {
            ledg.set_high();
            ledv2.set_low();
        } else {
            ledg.set_low();
            ledv2.set_high();
        }


        if luminozitate < LUM_MIN || luminozitate > LUM_MAX {
            ledv1.set_low();
            ledr.set_high();
        } else {
            ledv1.set_high();
            ledr.set_low();
        }


        if level < UMID_MIN   {
            buzzer.set_high();
            // config_pwm.compare_a=config_pwm.top/2;
            pompa.set_low();
            Timer::after_secs(1).await;
            buzzer.set_low();
        } else if level > UMID_MAX{
            pompa.set_high();
            buzzer.set_high();
            // config_pwm.compare_a=config_pwm.top/2;
            Timer::after_secs(1).await;
            buzzer.set_low();
        } else {
            buzzer.set_low();
            // config_pwm.compare_a=0;
            pompa.set_low();
        }

        // pwm_buzzer.set_config(&config_pwm);
        info!("");
        info!("");
        Timer::after_secs(4).await;


////////////////////////////////////////////////////

        // let select_result = select(
        //     button.wait_for_falling_edge(),
        //     socket.recv_from(&mut buf),
        // ).await;

        // match select_result {
        //     embassy_futures::select::Either::First(_) => {
        //         info!("Button pressed, sending message.");
        //         let msg = "Button pressed";
        //         let endpoint = IpEndpoint::new(IpAddress::v4(172, 20, 10, 10), 1234);
        //         if let Err(e) = socket.send_to(msg.as_bytes(), endpoint).await {
        //             warn!("Send error: {:?}", e);
        //         }
        //     }
        //     embassy_futures::select::Either::Second(recv) => {
        //         match recv {
        //             Ok((n, endpoint)) => {
        //                 let received_message = from_utf8(&buf[..n]).unwrap().trim();
        //                 info!("Received '{}' from {:?}", received_message, endpoint);

        //                 match received_message {
        //                     "ON" => {
        //                         control.gpio_set(0, true).await; // turn on LED
        //                     },
        //                     "OFF" => {
        //                         control.gpio_set(0, false).await; // turn off LED
        //                     },
        //                     _ => {
        //                         warn!("Unknown command: '{}'", received_message);
        //                     }
        //                 }
        //             },
        //             Err(e) => {
        //                 warn!("Recv error: {:?}", e);
        //             }
        //         }
        //     },
        // }
      
    //}
      ///////////////////////////////////////////////////////////
}
}

#[panic_handler]
fn panic(_info: &PanicInfo) -> ! {
    loop {}
}