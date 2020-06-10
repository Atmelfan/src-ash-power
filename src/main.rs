//! Serial interface DMA RX transfer test

#![feature(try_blocks)]

#![allow(unsafe_code)]
#![no_main]
#![no_std]

use panic_halt as _;

use cortex_m::{asm, singleton};
use core::cell::Cell;
use cortex_m::interrupt::Mutex;

use cortex_m_rt::entry;
use stm32f1xx_hal::stm32::{
    I2C2,
    interrupt
};
use embedded_hal::{
    blocking,
    digital::v2::OutputPin
};
use stm32f1xx_hal::{
    pac,
    prelude::*,
    gpio::{Output, OpenDrain, Alternate},
    gpio::gpiob::*,
    serial::{Config, Serial},
    i2c::{BlockingI2c, Mode}
};
use lazy_static::lazy_static;
use nb::block;

use core::fmt::Write;

//
use shared_bus::BusManager;
use ina219::{INA219, INA219_ADDR};
use ina3221::{INA3221, INA3221_ADDR};
use xca9548a::{SlaveAddr, Xca9543a};

//Default commands
use scpi::prelude::*;
use scpi::ieee488::commands::*;
use scpi::scpi::commands::*;
use scpi::{
    ieee488_cls,
    ieee488_ese,
    ieee488_esr,
    ieee488_idn,
    ieee488_opc,
    ieee488_rst,
    ieee488_sre,
    ieee488_stb,
    ieee488_tst,
    ieee488_wai,
    scpi_status,
    scpi_system,

    //Helpers
    qonly
};
use scpi::suffix::SuffixUnitElement;
use scpi::tokenizer::NumericValues;
use scpi::response::{ArrayVecFormatter, Formatter};

use core::str;
use git_version::git_version;
use stm32f1xx_hal::qei::SlaveMode::ResetMode;
use stm32f1xx_hal::gpio::State;

const GIT_VERSION: &[u8] = git_version!().as_bytes();

//***********************************************************************************
/// # SCPI code

struct MyDevice;
impl Device for MyDevice {
    fn cls(&mut self) -> Result<(), Error> {
        unimplemented!()
    }

    fn rst(&mut self) -> Result<(), Error> {
        Ok(())
    }

}

#[derive(Copy, Clone)]
pub struct AuxPowers {
    pub voltage: u16,
    pub current: i16,
    pub power: i16
}
impl AuxPowers {
    pub fn new() -> Self {
        AuxPowers {
            voltage: 0,
            current: 0,
            power: 0
        }
    }
}

lazy_static! {
    static ref AUX_POWERS: Mutex<Cell<AuxPowers>> =
        Mutex::new(Cell::new(AuxPowers::new()));
}
struct SensVoltDcCommand;
impl Command for SensVoltDcCommand { qonly!();
    fn query(&self, _context: &mut Context, args: &mut Tokenizer, response: &mut dyn Formatter)
        -> Result<(), Error> {
        cortex_m::interrupt::free(|cs| {
            let aux = AUX_POWERS.borrow(cs).get();
            response.ascii_data(b"VOLT:DC ")?;
            response.f32_data((aux.voltage as f32) / 1000f32)
        })
    }
}
struct SensCurrDcCommand;
impl Command for SensCurrDcCommand { qonly!();
    fn query(&self, _context: &mut Context, args: &mut Tokenizer, response: &mut dyn Formatter)
             -> Result<(), Error> {
        cortex_m::interrupt::free(|cs| {
            let aux = AUX_POWERS.borrow(cs).get();
            response.ascii_data(b"CURR:DC ")?;
            response.f32_data((aux.current as f32) / 32768f32)
        })
    }
}
struct SensPwrDcCommand;
impl Command for SensPwrDcCommand { qonly!();
    fn query(&self, _context: &mut Context, args: &mut Tokenizer, response: &mut dyn Formatter)
             -> Result<(), Error> {
        cortex_m::interrupt::free(|cs| {
            let aux = AUX_POWERS.borrow(cs).get();
            response.ascii_data(b"WATT ")?;
            response.f32_data((aux.power as f32) / 1000f32)
        })
    }
}

//***********************************************************************************
/// # Main code

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
    let mut gpiob = p.GPIOB.split(&mut rcc.apb2);

    // *** USART1
    let tx = gpioa.pa9.into_alternate_push_pull(&mut gpioa.crh);
    let rx = gpioa.pa10;

    let serial = Serial::usart1(
        p.USART1,
        (tx, rx),
        &mut afio.mapr,
        Config::default().baudrate(115_200.bps()),
        clocks,
        &mut rcc.apb2,
    );

    // Split the serial struct into a receiving and a transmitting part
    let (mut tx, mut rx) = serial.split();

    // *** I2C
    let mut i2c_rst = gpioa.pa1.into_push_pull_output(&mut gpioa.crl);
    if let Err(_) = i2c_rst.set_high() {
        writeln!(tx, "err").unwrap();
    }
    let sda = gpiob.pb11.into_alternate_open_drain(&mut gpiob.crh);
    let scl = gpiob.pb10.into_alternate_open_drain(&mut gpiob.crh);

    let i2c_bus = BlockingI2c::i2c2(
        p.I2C2,
        (scl,sda),
        Mode::standard(100.khz()),
        clocks,
        &mut rcc.apb1,
        1000,
        1,
        1000,
        1000
    );

    //I2C_POWER
    let i2c_pwr_manager = shared_bus::BusManager::<Mutex<_>, _>::new(i2c_bus);

    //let mut aux_ina219 = INA219::new(i2c_pwr_manager.acquire(),
    //                                 0x45);
    //if let Ok(()) = aux_ina219.calibrate(13421) {
    //    let voltage = aux_ina219.voltage().unwrap();
    //    let current = aux_ina219.current().unwrap();
    //    writeln!(tx, "cal ok, voltage: {}, current: {}", voltage, current).unwrap();
    //}

    let pca9543a = Xca9543a::new(i2c_pwr_manager.acquire(),
                                     SlaveAddr::Alternative(false, false, false));
    let parts = pca9543a.split();

    //R_I2C
    let i2c_right_manager = shared_bus::BusManager::<Mutex<_>, _>::new(parts.i2c0);
    let mut r_ina219 = INA219::new(i2c_right_manager.acquire(),
                                   0x40);
    if let Ok(()) = r_ina219.calibrate(13421) {
        let voltage = r_ina219.voltage().unwrap();
        let current = r_ina219.current().unwrap();
        writeln!(tx, "cal ok, voltage: {}, current: {}", voltage, current).unwrap();
    }

    let mut r_ina3221 = INA3221::new(i2c_right_manager.acquire(),
                                     0x40);

    //L_I2C
    let i2c_left_manager = shared_bus::BusManager::<Mutex<_>, _>::new(parts.i2c1);
    let mut l_ina219 = INA219::new(i2c_left_manager.acquire(),
                                     0x40);
    if let Ok(()) = l_ina219.calibrate(13421) {
        let voltage = l_ina219.voltage().unwrap();
        let current = l_ina219.current().unwrap();
        writeln!(tx, "cal ok, voltage: {}, current: {}", voltage, current).unwrap();
    }

    let mut l_ina3221 = INA3221::new(i2c_left_manager.acquire(),
                                     0x40);


    let mut my_device = MyDevice { };

    let mut tree = Node {name: b"ROOT", optional: true, handler: None, sub: Some(&[
        // Create default IEEE488 mandated commands
        Node {name: b"*IDN", optional: false,
            handler: Some(&IdnCommand{
                manufacturer: b"GPA-Robotics",
                model: b"ash-power",
                serial: b"0",
                firmware: GIT_VERSION
            }),
            sub: None
        },
        ieee488_cls!(),
        ieee488_ese!(),
        ieee488_esr!(),
        ieee488_opc!(),
        ieee488_rst!(),
        ieee488_sre!(),
        ieee488_stb!(),
        ieee488_tst!(),
        ieee488_wai!(),
        // Create default SCPI mandated STATus subsystem
        scpi_status!(),
        // Create default SCPI mandated SYSTem subsystem
        scpi_system!(),

        Node {
            name: b"SENSe",
            optional: true,
            handler: None,
            sub: Some(&[
                Node {
                    name: b"VOLTage",
                    optional: false,
                    handler: None,
                    sub: Some(&[
                        Node {
                            name: b"DC",
                            optional: true,
                            handler: Some(&SensVoltDcCommand {}),
                            sub: None
                        }
                    ])
                },
                Node {
                    name: b"CURRent",
                    optional: false,
                    handler: None,
                    sub: Some(&[
                        Node {
                            name: b"DC",
                            optional: true,
                            handler: Some(&SensCurrDcCommand {}),
                            sub: None
                        }
                    ])
                },
                Node {
                    name: b"POWer",
                    optional: false,
                    handler: None,
                    sub: Some(&[
                        Node {
                            name: b"DC",
                            optional: true,
                            handler: Some(&SensPwrDcCommand {}),
                            sub: None
                        }
                    ])
                }
            ])
        }
    ])};

    let mut errors = ArrayErrorQueue::<[Error; 10]>::new();

    let mut context = Context::new(&mut my_device, &mut errors, &mut tree);

    let mut buffer: [u8; 100] = [0; 100];
    let mut i = 0;



    loop {
        // Update sensors
        if let Ok(a) = r_ina3221.read_channel(0) {
            writeln!(tx, "cal ok, voltage: {}, current: {}", a.voltage(), a.current(0.01f32)).unwrap();
        }

        if let Ok(a) = l_ina3221.read_channel(0) {
            writeln!(tx, "cal ok, voltage: {}, current: {}", a.voltage(), a.current(0.01f32)).unwrap();
        }




    }
}