use esp_hal::gpio::Output;
use esp_hal::uart::{self, Async, TxError, UartTx};
use tmc2209::{self, reg, WriteRequest};

/// Minimal asynchronous TMC2209 driver used for this project.
pub struct Tmc2209Driver {
    tx: UartTx<'static, Async>,
    slave_addr: u8,
    rsense: f32,
}

impl Tmc2209Driver {
    /// Create a new driver instance.
    pub fn new(tx: UartTx<'static, Async>, slave_addr: u8, rsense: f32) -> Self {
        Self { tx, slave_addr, rsense }
    }

    async fn send_write_request<R>(&mut self, reg: R) -> Result<(), TxError>
    where
        R: reg::WritableRegister,
    {
        let req: WriteRequest = WriteRequest::new::<R>(self.slave_addr, reg);
        let bytes = req.bytes();
        let mut sent = 0;
        while sent < bytes.len() {
            let written = self.tx.write_async(&bytes[sent..]).await?;
            sent += written;
        }
        self.tx.flush_async().await
    }

    /// Configure the driver with sane defaults similar to the C++ example.
    pub async fn init(&mut self) -> Result<(), TxError> {

        // Configure chopper with TOFF=5 and 1/16 microsteps.
        let mut chop = reg::CHOPCONF::default();
        chop.set_toff(5);
        chop.set_mres(4); // 1/16 microsteps

        // Set current control.
        let (vsense, cs) = tmc2209::rms_current_to_vsense_cs(self.rsense, 600.0);
        chop.set_vsense(vsense);
        self.send_write_request(chop).await?;

        let mut ihold_irun = reg::IHOLD_IRUN::default();
        ihold_irun.set_irun(cs.into());
        ihold_irun.set_ihold(0);
        ihold_irun.set_ihold_delay(1);
        self.send_write_request(ihold_irun).await?;

        // Enable pwm autoscale for stealthChop.
        let mut pwmconf = reg::PWMCONF::default();
        pwmconf.set_pwm_autoscale(true);
        self.send_write_request(pwmconf).await?;

        Ok(())
    }

    /// Change the shaft direction bit in GCONF.
    pub async fn set_direction(&mut self, dir: bool) -> Result<(), TxError> {
        let mut gconf = reg::GCONF::default();
        gconf.set_shaft(dir);
        self.send_write_request(gconf).await
    }
}


use embassy_time::{Timer, Duration};

/// Asynchronous task that toggles the stepper in alternating directions.
#[embassy_executor::task]
pub async fn run_stepper_task(
    mut step_pin: Output<'static>,
    mut dir_pin: Output<'static>,
    mut driver: Tmc2209Driver,
) {
    if driver.init().await.is_err() {
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
        let _ = driver.set_direction(dir).await;
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

