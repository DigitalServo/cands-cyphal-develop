#[cfg(any(feature="usb-ftdi", feature="raspberrypi"))]
use std::{thread, time};

#[cfg(any(feature="usb-ftdi", feature="raspberrypi"))]
impl crate::CANInterface {

    pub fn drive_enable(&mut self, channel: u8) -> Result<(), Box<dyn std::error::Error>> {

        self.send_digitalservo_response(channel, "cmdval", &[0.0])?;
        thread::sleep(time::Duration::from_millis(50));

        self.send_digitalservo_response(channel, "drive", &[1.0])?;
        thread::sleep(time::Duration::from_millis(50));

        Ok(())
    }

    pub fn drive_enable_all(&mut self) -> Result<(), Box<dyn std::error::Error>> {

        self.send_digitalservo_message("cmdval", &[0.0])?;
        thread::sleep(time::Duration::from_millis(50));

        self.send_digitalservo_message("drive", &[1.0])?;
        thread::sleep(time::Duration::from_millis(50));

        Ok(())
    }

    pub fn drive_disable(&mut self, channel: u8) -> Result<(), Box<dyn std::error::Error>> {

        self.send_digitalservo_response(channel, "drive", &[0.0])?;
        thread::sleep(time::Duration::from_millis(100));

        self.send_digitalservo_response(channel, "cmdval", &[0.0])?;
        thread::sleep(time::Duration::from_millis(50));

        Ok(())
    }

    pub fn drive_disable_all(&mut self) -> Result<(), Box<dyn std::error::Error>> {
        
        self.send_digitalservo_message("drive", &[0.0])?;
        thread::sleep(time::Duration::from_millis(50));

        self.send_digitalservo_message("cmdval", &[0.0])?;
        thread::sleep(time::Duration::from_millis(50));

        Ok(())
    }

    pub fn send_velocity_reference(&mut self, channel: u8, value: f64) -> Result<(), Box<dyn std::error::Error>> {
        self.send_digitalservo_response(channel, "cmdval", &[value])
    }

    pub fn send_motion_reference(&mut self, channel: u8, value: &[f64; 4]) -> Result<(), Box<dyn std::error::Error>> {
        self.send_digitalservo_response(channel, "cmdarray", value)
    }
}