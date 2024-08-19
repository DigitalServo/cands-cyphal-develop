#[cfg(any(feature="usb-ftdi", feature="raspberrypi"))]
use cands_transport::cyphal::CyphalRxData;

#[cfg(any(feature="usb-ftdi", feature="raspberrypi"))]
use cands_presentation::cyphal::digitalservo::{
    dictionary::Dict,
    string::Str,
    traits::{DigitalServoPrimitiveData, IntoDigitalServoDataType}
};

mod shorthand;

#[cfg(any(feature="usb-ftdi", feature="raspberrypi"))]
impl crate::CANInterface {

    pub fn send_digitalservo_message<T: Clone + IntoDigitalServoDataType + Into<DigitalServoPrimitiveData>>(&mut self, key: &str, value: &[T]) -> Result<(), Box<dyn std::error::Error>> {
        const SUBJECT_ID: u16 = 0x488;
        let payload:Vec<u8> = Dict::serialize(key, value);
        self.send_message(SUBJECT_ID, &payload)
    }

    pub fn send_digitalservo_response<T: Clone + IntoDigitalServoDataType + Into<DigitalServoPrimitiveData>>(&mut self, channel: u8, key: &str, value: &[T]) -> Result<(), Box<dyn std::error::Error>> {
        const SERVICE_ID: u16 = 0x80;
        let payload:Vec<u8> = Dict::serialize(key, &value);
        self.send_response(SERVICE_ID, channel, &payload)
    }

    pub fn send_digitalservo_request(&mut self, channel: u8, key: &str) -> Result<(), Box<dyn std::error::Error>> {
        const SERVICE_ID: u16 = 0x80;
        let payload:Vec<u8> = Dict::serialize(key, &[0.0]);
        self.send_request(SERVICE_ID, channel, &payload)
    }

    pub fn send_digitalservo_set_value<T: Clone + IntoDigitalServoDataType + Into<DigitalServoPrimitiveData>>(&mut self, channel: u8, key: &str, value: &[T]) -> Result<(), Box<dyn std::error::Error>> {
        const SERVICE_ID: u16 = 0x81;
        let payload:Vec<u8> = Dict::serialize(key, &value);
        self.send_request(SERVICE_ID, channel, &payload)
    }

    pub fn send_digitalservo_get_value(&mut self, channel: u8, key: &str) -> Result<(), Box<dyn std::error::Error>> {
        const SERVICE_ID: u16 = 0x82;
        let payload:Vec<u8> = Str::serialize(key);
        self.send_request(SERVICE_ID, channel, &payload)
    }

    pub fn get_digitalservo_general_status(&mut self) -> u8 {
        const TARGET_PORT_ID: u16 = 0x87;
        // Filter data which are to be processed
        let mut target_ids: Vec<usize> = vec![];
        for i in 0..self.rx_complete_fifo.len() {
            let port_id: u16 = self.rx_complete_fifo[i].props.port_id;
            if (port_id == TARGET_PORT_ID) {
                target_ids.push(i);
            }
        }

        let mut result :u8 = 0xFF;

        // Process target data
        for process_target_id in &target_ids {
            let packet = &self.rx_complete_fifo[*process_target_id];
            // match Dict::deserialize(&packet.payload) {
            //     Ok(data) => v.push(CyphalRxData{data, props: packet.props}),
            //     Err(err) => return Err(err)
            // }
            // v.push(&packet.payload);
            match packet.payload.first() {
                Some(val) => result = *val,
                None => ()
            }

        }

        // Delete processed data from rx_fifo
        for remove_target_id in target_ids.iter().rev() {
            self.rx_complete_fifo.remove(*remove_target_id);
        }

        result
    }

    pub fn get_key_value(&mut self) -> Result<Option<Vec<CyphalRxData<Dict>>>, Box<dyn std::error::Error>> {
        const TARGET_PORT_ID: [u16; 3] = [128, 129, 1160];

        let mut v: Vec<CyphalRxData<Dict>> = Vec::new();

        // Load data from a device FIFO and put RxFrames on a user-space FIFO
        self.load_frames()?;
        
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
