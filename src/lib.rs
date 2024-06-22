pub use cands_interface::{TCAN455xTranceiver, RxData, SIDFCONFIG, XIDFCONFIG};
pub use cands_transport::cyphal::{CyphalMiddleware, CyphalRxFrame, CyphalRxPacketType, CRC_SIZE_BYTES};

mod special_instructions;
pub use special_instructions::digitalservo;

const MTU_CAN_FD: usize = 64;
const NODE_ID: u8 = 127;

const SIDF1: SIDFCONFIG = SIDFCONFIG { sft: 3, sfec: 0, sidf1: 0x123, sidf2: 0x456 };
const SIDF2: SIDFCONFIG = SIDFCONFIG { sft: 3, sfec: 5, sidf1: 0x123, sidf2: 0x456 };
const XIDF1: XIDFCONFIG = XIDFCONFIG { eft: 0, efec: 0, eidf1: 0x55555, eidf2: 0x77777 };
const SIDF: [SIDFCONFIG; 2] = [SIDF1, SIDF2];
const XIDF: [XIDFCONFIG; 1] = [XIDF1];

pub struct CANInterface {
    pub middleware: CyphalMiddleware<MTU_CAN_FD>,
    pub driver: TCAN455xTranceiver,
    pub rx_complete_fifo: Vec<CyphalRxFrame>,
    pub rx_incomplete_fifo: Vec<CyphalRxFrame>
}

impl CANInterface {
    pub fn new() -> Result<Self, Box<dyn std::error::Error>> {
        let mut middleware: CyphalMiddleware<MTU_CAN_FD> = CyphalMiddleware::<MTU_CAN_FD>::new(NODE_ID);
        let mut driver: TCAN455xTranceiver = TCAN455xTranceiver::new()?;
        
        //Initialize: middleware
        let now: u128 = std::time::SystemTime::now()
            .duration_since(std::time::UNIX_EPOCH).unwrap()
            .as_millis();
        let id_init: u8 = (now % 32) as u8;
        middleware.transfer_id = id_init;

        // Initialize: driver
        driver.setup(&SIDF, &XIDF)?;
        
        Ok(Self { middleware, driver, rx_complete_fifo: vec![], rx_incomplete_fifo: vec![] })
    }

    pub fn init(&mut self) -> Result<(), Box<dyn std::error::Error>> {
 
        self.driver.setup(&SIDF, &XIDF)?;
        self.reset_rx_fifo();

        // Message: dummy transfer_id to make intentional missmatch of current_transfer_id in slaves and that in this system.
        let now: u128 = std::time::SystemTime::now()
            .duration_since(std::time::UNIX_EPOCH).unwrap()
            .as_millis();
        let id_init: u8 = (now % 32) as u8;
        self.middleware.transfer_id = id_init;
      
        Ok(())
    }

    pub fn reset_rx_fifo(&mut self) {
        self.rx_complete_fifo.clear();
        self.rx_incomplete_fifo.clear();
    }

    pub fn send_message(&mut self, subject_id: u16, payload: &[u8]) -> Result<(), Box<dyn std::error::Error>> {
        match self.middleware.create_message_data(subject_id, &payload, payload.len()) {
            Ok(packets) => {
                for packet in packets {
                    self.driver.transmit(packet.xid, &packet.payload, packet.payload_size)?
                }
            },
            Err(err) => return Err(err)
        }
        Ok(())
    }

    pub fn send_response(&mut self, service_id: u16, channel: u8, payload: &[u8]) -> Result<(), Box<dyn std::error::Error>> {
        match self.middleware.create_response_data(channel, service_id, &payload, payload.len()) {
            Ok(packets) => {
                for packet in packets {
                    self.driver.transmit(packet.xid, &packet.payload, packet.payload_size)?
                }
            },
            Err(err) => return Err(err)
        }
        Ok(())
    }

    pub fn send_request(&mut self, service_id: u16, channel: u8, payload: &[u8]) -> Result<(), Box<dyn std::error::Error>> {
        match self.middleware.create_request_data(channel, service_id, &payload, payload.len()) {
            Ok(packets) => {
                for packet in packets {
                    self.driver.transmit(packet.xid, &packet.payload, packet.payload_size)?
                }
            },
            Err(err) => return Err(err)
        }
        Ok(())
    }

    pub fn read_driver_rx_fifo(&mut self) -> std::io::Result<Option<RxData>>{
        match self.driver.receive() {
            Ok(rx_data) => Ok(rx_data),
            Err(err) => Err(err)
        }
    }

    pub fn load_frames_from_buffer(&mut self, buffer: &[u8]) -> Result<(), Box<dyn std::error::Error>> {
        match self.middleware.try_read(buffer) {
            Ok(packets) => {
                for packet in packets {
                    match packet.status.frame_type {
                        CyphalRxPacketType::SignleFrame => {
                            self.rx_complete_fifo.push(CyphalRxFrame {
                                xid: packet.xid,
                                payload: packet.payload.to_vec(),
                                payload_size: packet.payload_size,
                                props: packet.props
                            });
                        },
                        CyphalRxPacketType::MultiFrameStart => {
                            self.rx_incomplete_fifo.push(CyphalRxFrame {
                                xid: packet.xid,
                                payload: Vec::from(&packet.payload[..packet.payload_size]),
                                payload_size: packet.payload_size,
                                props: packet.props
                            });
                        },
                        CyphalRxPacketType::MultiFrameInProcess => {
                            let target_frame_position: Option<usize> = self.rx_incomplete_fifo
                                .iter()
                                .position(|frame| (frame.xid == packet.xid) & (frame.props.port_id == packet.props.port_id));
                            if let Some(position) = target_frame_position {
                                self.rx_incomplete_fifo[position].payload.extend(&packet.payload[..packet.payload_size]);
                                self.rx_incomplete_fifo[position].payload_size += packet.payload_size;
                            }
                        },
                        CyphalRxPacketType::MultiFrameEnd => {
                            let target_frame_position: Option<usize> = self.rx_incomplete_fifo
                                .iter()
                                .position(|frame: &CyphalRxFrame| (frame.xid == packet.xid) & (frame.props.port_id == packet.props.port_id));

                            if let Some(position) = target_frame_position {
                                self.rx_incomplete_fifo[position].payload.extend(&packet.payload[..(packet.payload_size - CRC_SIZE_BYTES as usize)]);
                                self.rx_incomplete_fifo[position].payload_size += packet.payload_size - CRC_SIZE_BYTES as usize;

                                let crc_bytes: [u8; 2] = self.rx_incomplete_fifo[position].calculate_crc()?;
                                let crc_bytes_expected: [u8; 2] = [packet.payload[packet.payload_size - CRC_SIZE_BYTES as usize], packet.payload[packet.payload_size - CRC_SIZE_BYTES as usize + 1]];

                                if crc_bytes == crc_bytes_expected {
                                    self.rx_complete_fifo.push(self.rx_incomplete_fifo[position].clone());
                                    self.rx_incomplete_fifo.remove(position);
                                }
                                else {
                                    self.rx_incomplete_fifo.remove(position);
                                    return Err("INVALID DATA EXIST: CRC ERROR AT MULTIFRAME CONSTRUCTION".into());
                                }
                            }
                        }
                    }
                }
            },
            Err(err) => return Err(err)
        };

        Ok(())
    }

    pub fn load_frames(&mut self) -> Result<(), Box<dyn std::error::Error>> {
        let rx_data: Option<RxData> = self.read_driver_rx_fifo()?;

        if let Some(rx_data) = rx_data {
            self.load_frames_from_buffer(&rx_data.fifo1)?
        }
        
        Ok(())
    }

}