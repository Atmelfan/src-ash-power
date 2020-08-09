use ina3221::{INA3221, INA3221_ADDR};
use shared_bus::{BusManager, BusProxy};
use cortex_m::interrupt::Mutex;

struct Leg<'a, I2C> {
    i2c: BusManager<Mutex<_>, I2C>,
    cs: INA3221<BusProxy<'a, Mutex<_>, I2C>>,
    io: INA3221<BusProxy<'a, Mutex<_>, I2C>>
}


impl<'a, I2C> Leg<'a, I2C> {
    pub fn new(i2c: I2C, offs: u8) -> Leg<'a, I2C>{
        let manager = BusManager::new(i2c);
        Leg {
            i2c: manager,
            cs: INA3221::new(manager.acquire(), INA3221_ADDR + offs),
            io: INA3221::new(manager.acquire(), INA3221_ADDR + offs)
        }
    }
}



