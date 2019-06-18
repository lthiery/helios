use heapless::{consts::U128, Vec};

pub const SYNC_1: u8 = 0xB5;
pub const SYNC_2: u8 = 0x62;

const LEN_HEADER: u16 = 4; 	// num of bytes in header minus the sync chars
const LEN_CHECKSUM: u16 = 2; // number of checksum bytes


pub struct Ubx {
	pub buffer: Vec<u8, U128>,
    pub prev_byte: u8,
    pub in_msg: bool,
    pub payload_len: u16,
    pub count: u16,
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

	fn reset(&mut self){
		self.in_msg = false;
		self.count = 0;
		self.payload_len = 0;
	}

	pub fn push(&mut self, byte: u8) -> (bool, u16) {

		if self.in_msg {
			self.buffer.push(byte);
			self.count+=1;

			if self.count == 3 {
				self.payload_len = byte.into();
			} else if self.count == 4 {
				self.payload_len |= (byte as u16) <<8;
			}

			if self.count >= 4 {
				// msg is over
				if self.count - LEN_HEADER == self.payload_len + LEN_CHECKSUM {

					// buffer starts after the sync words, so checksum has no offset
					let (ck_a, ck_b) = calculate_checksum(self.buffer.as_ref(), 0);
					// make copy of count
					let count = self.count;
					self.reset();

					if ck_a == self.buffer[count as usize -2] && ck_b == self.buffer[count as usize -1] {
						return (true, count);
					} else {
						self.buffer.clear();
					}
				} 
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
        (false, 0)
	}
}

pub fn calculate_checksum(buf: &[u8], offset: usize) -> (u8, u8){
	let mut ck_a: u8 = 0;
	let mut ck_b: u8 = 0;

	let len = buf.len() - 2;

	for i in  offset..len {
		ck_a = ck_a.wrapping_add(buf[i]);
		ck_b = ck_b.wrapping_add(ck_a);
	}
	(ck_a, ck_b)
}

pub fn set_checksum(buf: &mut [u8]){
	// sync word needs to be ignored, so provide offset
	let (ck_a, ck_b) = calculate_checksum(buf, 2);

	let len = buf.len() - 2;

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