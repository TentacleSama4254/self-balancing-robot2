use embedded_io::Write;
use embedded_hal_0_2 as hal02;
use esp_hal::gpio::Output;
use esp_hal::{uart, Blocking};
use tmc2209::{self, reg};

/// Wrapper to implement the blocking serial write trait required by the
/// `tmc2209` crate for any type implementing [`embedded_io::Write`].
pub struct SerialWriteWrapper<'a, W: Write + 'a>(pub &'a mut W);

impl<'a, W> hal02::blocking::serial::Write<u8> for SerialWriteWrapper<'a, W>
where
    W: Write,
{
    type Error = W::Error;

    fn bwrite_all(&mut self, mut buffer: &[u8]) -> Result<(), Self::Error> {
        while !buffer.is_empty() {
            let written = self.0.write(buffer)?;
            buffer = &buffer[written..];
        }
        Ok(())
    }

    fn bflush(&mut self) -> Result<(), Self::Error> {
        self.0.flush()
    }
}

/// Minimal TMC2209 driver used for this project.
pub struct Tmc2209Driver<W> {
    tx: W,
    slave_addr: u8,
    rsense: f32,
}

impl<W> Tmc2209Driver<W> {
    /// Create a new driver instance.
    pub fn new(tx: W, slave_addr: u8, rsense: f32) -> Self {
        Self { tx, slave_addr, rsense }
    }

    /// Configure the driver with sane defaults similar to the C++ example.
    pub fn init(&mut self) -> Result<(), W::Error>
    where
        W: Write,
    {
        let mut tx = SerialWriteWrapper(&mut self.tx);

        // Configure chopper with TOFF=5 and 1/16 microsteps.
        let mut chop = reg::CHOPCONF::default();
        chop.set_toff(5);
        chop.set_mres(4); // 1/16 microsteps

        // Set current control.
        let (vsense, cs) = tmc2209::rms_current_to_vsense_cs(self.rsense, 600.0);
        chop.set_vsense(vsense);
        tmc2209::send_write_request(self.slave_addr, chop, &mut tx)?;

        let mut ihold_irun = reg::IHOLD_IRUN::default();
        ihold_irun.set_irun(cs.into());
        ihold_irun.set_ihold(0);
        ihold_irun.set_ihold_delay(1);
        tmc2209::send_write_request(self.slave_addr, ihold_irun, &mut tx)?;

        // Enable pwm autoscale for stealthChop.
        let mut pwmconf = reg::PWMCONF::default();
        pwmconf.set_pwm_autoscale(true);
        tmc2209::send_write_request(self.slave_addr, pwmconf, &mut tx)?;

        Ok(())
    }

    /// Change the shaft direction bit in GCONF.
    pub fn set_direction(&mut self, dir: bool) -> Result<(), W::Error>
    where
        W: Write,
    {
        let mut tx = SerialWriteWrapper(&mut self.tx);
        let mut gconf = reg::GCONF::default();
        gconf.set_shaft(dir);
        tmc2209::send_write_request(self.slave_addr, gconf, &mut tx)
    }
}


use embassy_time::{Timer, Duration};

/// Asynchronous task that toggles the stepper in alternating directions.
#[embassy_executor::task]
pub async fn run_stepper_task(
    mut step_pin: Output<'static>,
    mut dir_pin: Output<'static>,
    mut driver: Tmc2209Driver<uart::UartTx<'static, Blocking>>,
) {
    if driver.init().is_err() {
        return;
    }
    let mut dir = false;
    loop {
        // Set direction
        if dir {
            dir_pin.set_high();
        } else {
            dir_pin.set_low();
        }
        let _ = driver.set_direction(dir);
        // Run 5000 steps
        for _ in 0..5000 {
            step_pin.set_high();
            Timer::after(Duration::from_micros(160)).await;
            step_pin.set_low();
            Timer::after(Duration::from_micros(160)).await;
        }
        dir = !dir;
    }
}

