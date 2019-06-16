use heapless::{consts::U128, Vec};

pub const SYNC_1: u8 = 0xB5;
pub const SYNC_2: u8 = 0x62;

const LEN_HEADER = 4; 	// num of bytes in header minus the sync chars
const LEN_CHECKSUM = 2; // number of checksum bytes


pub struct Ubx {
	buffer: Vec<u8, U128>,
    prev_byte: u8,
    in_msg: bool,
    payload_len: u16,
    count: u8,
}

impl Ubx {
	pub fn new() -> Ubx {
		Ubx {
			buffer: Vec::new(),
			prev_byte: 0,
			in_msg: false,
			payload_len: 0,
			count: 0,
		}
	}

	pub fn push(&mut self, byte: u8){

		if self.in_msg {
			self.buffer.push(byte);
			self.count+=1;

			if self.count == 3 {
				self.payload_len = byte.into();
			} else if self.count == 4 {
				self.payload_len |= (byte as u16) <<8;
			}

			// msg is over
			if count - LEN_HEADER == payload_len + LEN_CHECKSUM {
				
			}

        } else {
            // the beginning of a msg has started
            if self.prev_byte == SYNC_1 && byte == SYNC_2 {
                self.in_msg = true;
                self.prev_byte = 0;
            } else {
            	self.prev_byte = byte;
            }
        }
	}
}


        /*
        //                  HEADER1       HEADER2           CLASS              ID   
        let mut set_ubx = [ubx::SYNC_1, ubx::SYNC_2, ubx::ClassId::Cfg as u8, 0x00, 
            20, 0x00,                   // length 
        */

       

        

        // write!(resources.DEBUG_UART, "{:X} ", byte);

        // *PREV = byte;
/*
 CK_A = 0, CK_B = 0
 For(I=0;I<N;I++)
 {
 	CK_A = CK_A + Buffer[I]
 	CK_B = CK_B + CK_A
 }
 */

pub fn set_checksum(buf: &mut [u8]){

	let mut ck_a: u8 = 0;
	let mut ck_b: u8 = 0;

	let len = buf.len() - 2;

	for i in  2..len {
		ck_a = ck_a.wrapping_add(buf[i]);
		ck_b = ck_b.wrapping_add(ck_a);
	}
	buf[len] = ck_a;
	buf[len + 1] = ck_b;
}

use num_traits::cast::FromPrimitive;

enum_from_primitive! {
#[derive(Debug, PartialEq)]
pub enum ClassId {
Nav = 0x01, // Navigation Results: Position, Speed, Time, Acc, Heading, DOP, SVs used
Rxm = 0x02, // Receiver Manager Messages: Satellite Status, RTC Status
Inf = 0x04, // Information Messages: Printf-Style Messages, with IDs such as Error, Warning, Notice
Ack = 0x05, // Ack/Nack Messages: as replies to CFG Input Messages
Cfg = 0x06, // Configuration Input Messages: Set Dynamic Model, Set DOP Mask, Set Baud Rate, etc.
Mon = 0x0A, // Monitoring Messages: Comunication Status, CPU Load, Stack Usage, Task Status
Aid = 0x0B, // AssistNow Aiding Messages: Ephemeris, Almanac, other A-GPS data input
Tim = 0x0D, // Timing Messages: Timepulse Output, Timemark Results
Esf = 0x10, // External Sensor Fusion Messages: External sensor measurements and status information
}
}