//! Serial interface DMA RX transfer test

#![deny(unsafe_code)]
#![no_main]
#![no_std]

use panic_halt as _;

use cortex_m::{asm, singleton};

use cortex_m_rt::{
    entry
};
use stm32f1xx_hal::stm32::interrupt;
use stm32f1xx_hal::{
    pac,
    prelude::*,
    serial::{Config, Serial},
};
use nb::block;

use core::fmt::Write;

use scpi::command::Command;
use scpi::Device;
use scpi::tree::Node;
use scpi::tokenizer::Tokenizer;
use scpi::error::{Error, ErrorQueue, ArrayErrorQueue};
use scpi::response::{Formatter, ArrayVecFormatter};
use scpi::Context;
use scpi::ieee488::commands::*;
use scpi::scpi::commands::*;

use core::str;
use git_version::git_version;
const GIT_VERSION: &str = git_version!();

struct MyDevice;
impl Device for MyDevice {
    fn cls(&mut self) -> Result<(), Error> {
        unimplemented!()
    }

    fn rst(&mut self) -> Result<(), Error> {
        Ok(())
    }

}

#[interrupt]
fn USART1(){

}

#[entry]
fn main() -> ! {
    let p = pac::Peripherals::take().unwrap();

    let mut flash = p.FLASH.constrain();
    let mut rcc = p.RCC.constrain();

    let clocks = rcc.cfgr.freeze(&mut flash.acr);

    let mut afio = p.AFIO.constrain(&mut rcc.apb2);
    let channels = p.DMA1.split(&mut rcc.ahb);

    let mut gpioa = p.GPIOA.split(&mut rcc.apb2);
    // let mut gpiob = p.GPIOB.split(&mut rcc.apb2);

    // USART1
    let tx = gpioa.pa9.into_alternate_push_pull(&mut gpioa.crh);
    let rx = gpioa.pa10;

    let serial = Serial::usart1(
        p.USART1,
        (tx, rx),
        &mut afio.mapr,
        Config::default().baudrate(9_600.bps()),
        clocks,
        &mut rcc.apb2,
    );

    // Split the serial struct into a receiving and a transmitting part
    let (mut tx, mut rx) = serial.split();

    let mut my_device = MyDevice { };


    let mut tree = Node {name: b"ROOT", optional: true, handler: None, sub: Some(&[
        // Create default IEEE488 mandated commands
        Node {name: b"*IDN", optional: false,
            handler: Some(&IdnCommand{
                manufacturer: b"GPA-Robotics",
                model: b"T800",
                serial: b"0",
                firmware: GIT_VERSION.as_bytes()
            }),
            sub: None
        },
        scpi::ieee488_cls!(),
        scpi::ieee488_ese!(),
        scpi::ieee488_esr!(),
        scpi::ieee488_opc!(),
        scpi::ieee488_rst!(),
        scpi::ieee488_sre!(),
        scpi::ieee488_stb!(),
        scpi::ieee488_tst!(),
        scpi::ieee488_wai!(),
        // Create default SCPI mandated STATus subsystem
        scpi::scpi_status!(),
        // Create default SCPI mandated SYSTem subsystem
        scpi::scpi_system!()
    ])};



    let mut errors = ArrayErrorQueue::<[Error; 10]>::new();

    let mut context = Context::new(&mut my_device, &mut errors, &mut tree);

    let mut buffer: [u8; 100] = [0; 100];
    let mut i = 0;

    loop {
        if let Ok(c) = rx.read() {
            buffer[i] = c;
            block!(tx.write(c)).unwrap();
            i += 1;
            if c == b'\n' {
                let s = &buffer[0..i];
                i = 0;

                let mut buf = ArrayVecFormatter::<[u8; 256]>::new();

                //SCPI tokenizer
                let mut tokenizer = Tokenizer::from_str(s);

                //Result
                let result = context.exec(&mut tokenizer, &mut buf);

                if let Err(err) = result {
                    writeln!(tx, "Error {}", str::from_utf8(err.get_message().unwrap()).unwrap()).unwrap();
                } else if !buf.is_empty() {
                    writeln!(tx, "{}", str::from_utf8(buf.as_slice()).unwrap()).unwrap();
                }
            }
        }

    }
}