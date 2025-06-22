use esp_hal::gpio::{Output, Level};

/// Extension trait to add toggle functionality to esp_hal::gpio::Output
pub trait OutputToggle {
    /// Toggle the output pin from high to low or low to high
    fn toggle(&mut self);
}

impl<'a> OutputToggle for Output<'a> {
    fn toggle(&mut self) {
        if self.get_output_level() == Level::High {
            self.set_low();
        } else {
            self.set_high();
        }
    }
}
