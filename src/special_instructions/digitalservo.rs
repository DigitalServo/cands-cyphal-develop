#[cfg(any(feature="usb-ftdi", feature="raspberrypi"))]
use std::{thread, time};

#[cfg(any(feature="usb-ftdi", feature="raspberrypi"))]
use cands_transport::cyphal::CyphalRxData;

#[cfg(any(feature="usb-ftdi", feature="raspberrypi"))]
use cands_presentation::cyphal::digitalservo::{
    dictionary::Dict,
    traits::{DigitalServoPrimitiveData, IntoDigitalServoDataType}
};

#[cfg(any(feature="usb-ftdi", feature="raspberrypi"))]
use crate::CANInterface;

#[cfg(any(feature="usb-ftdi", feature="raspberrypi"))]
impl CANInterface {

    pub fn send_digitalservo_message<T: Clone + IntoDigitalServoDataType + Into<DigitalServoPrimitiveData>>(&mut self, key: &str, value: &[T]) -> Result<(), Box<dyn std::error::Error>> {
        const SUBJECT_ID: u16 = 1160;
        let payload:Vec<u8> = Dict::serialize(key, value);
        self.send_message(SUBJECT_ID, &payload)
    }

    pub fn send_digitalservo_response<T: Clone + IntoDigitalServoDataType + Into<DigitalServoPrimitiveData>>(&mut self, channel: u8, key: &str, value: &[T]) -> Result<(), Box<dyn std::error::Error>> {
        const SERVICE_ID: u16 = 129;
        let payload:Vec<u8> = Dict::serialize(key, &value);
        self.send_response(SERVICE_ID, channel, &payload)
    }

    pub fn send_digitalservo_request(&mut self, channel: u8, key: &str) -> Result<(), Box<dyn std::error::Error>> {
        const SERVICE_ID: u16 = 128;
        let payload:Vec<u8> = Dict::serialize(key, &[0.0]);
        self.send_request(SERVICE_ID, channel, &payload)
    }
    
    pub fn drive_enable(&mut self, channel: u8) -> Result<(), Box<dyn std::error::Error>> {

        self.send_digitalservo_response(channel, "cmdval", &[0.0])?;
        thread::sleep(time::Duration::from_millis(50));

        self.send_digitalservo_response(channel, "drive", &[true])?;
        thread::sleep(time::Duration::from_millis(50));

        Ok(())
    }

    pub fn drive_enable_all(&mut self) -> Result<(), Box<dyn std::error::Error>> {

        self.send_digitalservo_message("cmdval", &[0.0])?;
        thread::sleep(time::Duration::from_millis(50));

        self.send_digitalservo_message("drive", &[true])?;
        thread::sleep(time::Duration::from_millis(50));

        Ok(())
    }


    pub fn drive_disable(&mut self, channel: u8) -> Result<(), Box<dyn std::error::Error>> {

        self.send_digitalservo_response(channel, "drive", &[false])?;
        thread::sleep(time::Duration::from_millis(100));

        self.send_digitalservo_response(channel, "cmdval", &[0.0])?;
        thread::sleep(time::Duration::from_millis(50));

        Ok(())
    }

    pub fn drive_disable_all(&mut self) -> Result<(), Box<dyn std::error::Error>> {
        
        self.send_digitalservo_message("drive", &[false])?;
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

    pub fn get_key_value(&mut self) -> Result<Option<Vec<CyphalRxData<Dict>>>, Box<dyn std::error::Error>> {
        const TARGET_PORT_ID: [u16; 3] = [128, 129, 1160];
        
        let mut v: Vec<CyphalRxData<Dict>> = Vec::new();

        // Filter data which are to be processed
        let mut target_ids: Vec<usize> = vec![];
        for i in 0..self.rx_complete_fifo.len() {
            let port_id: u16 = self.rx_complete_fifo[i].props.port_id;
            if (port_id == TARGET_PORT_ID[0]) | (port_id == TARGET_PORT_ID[1]) | (port_id == TARGET_PORT_ID[2]) {
                target_ids.push(i);
            }
        }

        // Process target data
        for process_target_id in &target_ids {
            let packet = &self.rx_complete_fifo[*process_target_id];
            match Dict::deserialize(&packet.payload) {
                Ok(data) => v.push(CyphalRxData{data, props: packet.props}),
                Err(err) => return Err(err)
            }
        }

        // Delete processed data from rx_fifo
        for remove_target_id in target_ids.iter().rev() {
            self.rx_complete_fifo.remove(*remove_target_id);
        }

        match v.len() {
            0 => Ok(None),
            _ => Ok(Some(v)) 
        }

    }

}
