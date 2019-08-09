#![no_std]
#[macro_use]
extern crate nb;
use nb::block;

#[macro_use]
extern crate enum_primitive;

use heapless::{consts::U128, Vec};

pub const SYNC_1: u8 = 0xB5;
pub const SYNC_2: u8 = 0x62;

const LEN_HEADER: u16 = 4; // num of bytes in header minus the sync chars
const LEN_CHECKSUM: u16 = 2; // number of checksum bytes

pub enum Message {
    NP(NavPvt),
}

#[derive(Copy, Clone)]
pub enum Mode {
    HighPower,
    LowPower
}

pub struct Ubx {
    pub buffer: Vec<u8, U128>,
    pub prev_byte: u8,
    pub in_msg: bool,
    pub payload_len: u16,
    pub count: u16,
    pub mode: Mode,
}

enum_from_primitive! {
#[derive(Debug, PartialEq)]
pub enum FixType {
    NoFix = 0,
    DeadReckoningOnly = 1,
    _2D= 2,
    _3D= 3,
    GNSSandDeadReckoning = 4,
    TimeOnly = 5
}
}

pub struct NavPvt {
    pub fix_type: FixType,
    pub year: u16,
    pub month: u8,
    pub day: u8,
    pub hour: u8,
    pub mins: u8,
    pub secs: u8,
    pub num_sats: u8,
    pub lat: i32,
    pub lon: i32,
    pub alt: i32,
    pub speed: i32,
}

impl core::fmt::Display for NavPvt {
    fn fmt(&self, f: &mut core::fmt::Formatter) -> core::fmt::Result {
        write!(
            f,
            "{:>02}:{:>02}:{:>02}\t{}/{}/{}\r\nSats: {}, Lat: {}, Lon: {}, Alt: {}, Speed: {}\r\n",
            self.hour,
            self.mins,
            self.secs,
            self.day,
            self.month,
            self.year,
            self.num_sats,
            (self.lat as f64) / 10000000.0,
            (self.lon as f64) / 10000000.0,
            self.alt,
            self.speed
        )
    }
}

impl NavPvt {
    fn new(buf: &[u8]) -> NavPvt {
        let fix_type = FixType::from_u8(buf[24]).expect("Invalid fix_type value");

        let year = (buf[8] as u16) | ((buf[9] as u16) << 8);
        let month = buf[10];
        let day = buf[11];
        let hour = buf[12];
        let mins = buf[13];
        let secs = buf[14];
        let num_sats = buf[27];

        let lon =
            unsafe { core::mem::transmute::<[u8; 4], i32>([buf[28], buf[29], buf[30], buf[31]]) };
        let lat =
            unsafe { core::mem::transmute::<[u8; 4], i32>([buf[32], buf[33], buf[34], buf[35]]) };
        let mut alt =
            unsafe { core::mem::transmute::<[u8; 4], i32>([buf[40], buf[41], buf[42], buf[43]]) };
        alt /= 1000;
        let mut speed =
            unsafe { core::mem::transmute::<[u8; 4], i32>([buf[64], buf[65], buf[66], buf[67]]) };
        speed /= 1000;
        NavPvt {
            fix_type,
            year,
            month,
            day,
            hour,
            mins,
            secs,
            num_sats,
            lat,
            lon,
            alt,
            speed,
        }
    }
}

impl Ubx {
    pub fn new() -> Ubx {
        Ubx {
            buffer: Vec::new(),
            prev_byte: 0,
            in_msg: false,
            payload_len: 0,
            count: 0,
            mode: Mode::HighPower,
        }
    }

    fn reset(&mut self) {
        self.in_msg = false;
        self.count = 0;
        self.payload_len = 0;
    }

    pub fn init<TX, DELAY>(&self, gps_tx: &mut TX, delay: &mut DELAY)
    where
        TX: embedded_hal::serial::Write<u8>,
        DELAY: embedded_hal::blocking::delay::DelayMs<u32>,
    {
        self.enable_ubx_protocol(gps_tx);
        delay.delay_ms(25);
        self.enable_ubx_protocol(gps_tx);
        delay.delay_ms(25);
        self.enable_nav_pvt(gps_tx);
        delay.delay_ms(25);
        self.enable_ext_ant(gps_tx);
    }

    pub fn push(&mut self, byte: u8) -> Option<Message> {
        if self.in_msg {
            self.buffer.push(byte);
            self.count += 1;

            if self.count == 3 {
                self.payload_len = byte.into();
            } else if self.count == 4 {
                self.payload_len |= (byte as u16) << 8;
            }

            if self.count >= 4 {
                // msg is over
                if self.count - LEN_HEADER == self.payload_len + LEN_CHECKSUM {
                    // buffer starts after the sync words, so checksum has no offset
                    let (ck_a, ck_b) = calculate_checksum(self.buffer.as_ref(), 0);
                    // make copy of count
                    let count = self.count;
                    self.reset();

                    if ck_a == self.buffer[count as usize - 2]
                        && ck_b == self.buffer[count as usize - 1]
                    {
                        let mut ret = None;
                        if self.buffer[0] == (ClassId::Nav as u8) && self.buffer[1] == 0x07 {
                            ret = Some(Message::NP(NavPvt::new(&self.buffer.as_ref())));
                        }
                        self.buffer.clear();
                        return ret;
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
        None
    }

    pub fn enable_ubx_protocol<TX>(&self, gps_tx: &mut TX)
    where
        TX: embedded_hal::serial::Write<u8>,
    {
        //          HEADER1       HEADER2           CLASS      ID
        let mut set_ubx = [
            SYNC_1,
            SYNC_2,
            ClassId::Cfg as u8,
            0x00,
            20,
            0x00, // length
            0x01, // port ID
            0x00, // reserved
            0x00,
            0x00, // tx ready
            0xD0,
            0x08,
            0x00,
            0x00, // MODE (8 bit len, no parity, 1 stop bit)
            0x80,
            0x25,
            0x00,
            0x00, // BAUD (9600)
            0b01,
            0x00, // inProtoMask (UBX only)
            0b01,
            0x00, // outProtoMask (UBX only)
            0x00,
            0x00,
            0x00,
            0x00, // reserved
            0x00,
            0x00, // checksum bytes
        ];
        self.set_checksum_and_write(gps_tx, &mut set_ubx)
    }

    pub fn enable_nav_pvt<TX>(&self, gps_tx: &mut TX)
    where
        TX: embedded_hal::serial::Write<u8>,
    {
        //       HEADER1       HEADER2                  CLASS     ID
        let mut enable_nav = [
            SYNC_1,
            SYNC_2,
            ClassId::Cfg as u8,
            0x01,
            8,
            0x00, // length

            ClassId::Nav as u8,
            0x07,
            0x00, // port 0
            0x01, // port 1
            0x00, // port 2
            0x00, // port 3
            0x00, // port 4
            0x00, // port 5
            0x00,
            0x00, // checksum bytes
        ];
        self.set_checksum_and_write(gps_tx, &mut enable_nav)
    }

    pub fn enable_high_power<TX>(&self, gps_tx: &mut TX)
    where
        TX: embedded_hal::serial::Write<u8>,
    {
        //       HEADER1       HEADER2                  CLASS     ID
        let mut cfg_rate = [
            SYNC_1,
            SYNC_2,
            ClassId::Cfg as u8,
            0x08,
            6,
            0x00, // length LSB

            0xe8, // [0] measure rate LSB
            0x3, // [1] measure rate MSB
            1,    // [2] nav rate LSB
            0,    // [3] nav rate MSB
            0,    // [4] time ref LSB 
            0,    // [5] time ref MSB

            0x00,
            0x00, // checksum bytes
        ];
        self.set_checksum_and_write(gps_tx, &mut cfg_rate);

        //       HEADER1       HEADER2                  CLASS     ID
        let mut cfg_pms = [
            SYNC_1,
            SYNC_2,
            ClassId::Cfg as u8,
            0x86,
            8,
            0x00, // length LSB

            0x00, // [0] version = 0
            0x03, // [1] power setup
            1,    // [2] on period LSB
            0x00, // [3] on period MSB
            1,    // [4] on time LSB (smaller than period)
            0x00, // [5] on time MSB
            0x00, // [6] reserved
            0x00, // [7] reserved

            0x00,
            0x00, // checksum bytes
        ];
        self.set_checksum_and_write(gps_tx, &mut cfg_pms)
    }

    pub fn enable_low_power<TX>(&self, gps_tx: &mut TX)
    where
        TX: embedded_hal::serial::Write<u8>,
    {
        //       HEADER1       HEADER2                  CLASS     ID
        let mut cfg_rate = [
            SYNC_1,
            SYNC_2,
            ClassId::Cfg as u8,
            0x08,
            6,
            0x00, // length LSB

            0x88, // [0] measure rate LSB
            0x13, // [1] measure rate MSB
            1,    // [2] nav rate LSB
            0,    // [3] nav rate MSB
            0,    // [4] time ref LSB 
            0,    // [5] time ref MSB

            0x00,
            0x00, // checksum bytes
        ];
        self.set_checksum_and_write(gps_tx, &mut cfg_rate);

        //       HEADER1       HEADER2                  CLASS     ID
        let mut cfg_pms = [
            SYNC_1,
            SYNC_2,
            ClassId::Cfg as u8,
            0x86,
            8,
            0x00, // length LSB

            0x00, // [0] version = 0
            0x05, // [1] power setup
            5,    // [2] on period LSB
            0x00, // [3] on period MSB
            5,    // [4] on time LSB (smaller than period)
            0x00, // [5] on time MSB
            0x00, // [6] reserved
            0x00, // [7] reserved

            0x00,
            0x00, // checksum bytes
        ];
        self.set_checksum_and_write(gps_tx, &mut cfg_pms)
    }



   

    pub fn enable_ext_ant<TX>(&self, gps_tx: &mut TX)
    where
        TX: embedded_hal::serial::Write<u8>,
    {
        let mut enable_ext_ant = [
            SYNC_1,
            SYNC_2,
            ClassId::Cfg as u8,
            0x13, /* CLASS, ID                 */
            0x04,
            0x00, /* LENGTH                    */
            0x00,
            0x00, /* FLAGS                     */
            0xf0,
            0xb9, /* PINS                      */
            0x00,
            0x00, /* CK_A, CK_B                */
        ];
        self.set_checksum_and_write(gps_tx, &mut enable_ext_ant)
    }

    fn set_checksum_and_write<TX>(&self, gps_tx: &mut TX, mut payload: &mut [u8])
    where
        TX: embedded_hal::serial::Write<u8>,
    {
        set_checksum(&mut payload);
        for byte in payload.iter() {
            block!(gps_tx.write(*byte));
        }
    }
}

pub fn calculate_checksum(buf: &[u8], offset: usize) -> (u8, u8) {
    let mut ck_a: u8 = 0;
    let mut ck_b: u8 = 0;

    let len = buf.len() - 2;

    for i in offset..len {
        ck_a = ck_a.wrapping_add(buf[i]);
        ck_b = ck_b.wrapping_add(ck_a);
    }
    (ck_a, ck_b)
}

pub fn set_checksum(buf: &mut [u8]) {
    // sync word needs to be ignored, so provide offset
    let (ck_a, ck_b) = calculate_checksum(buf, 2);

    let len = buf.len() - 2;

    buf[len] = ck_a;
    buf[len + 1] = ck_b;
}

use num_traits::cast::FromPrimitive;
use num_traits::cast::ToPrimitive;

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
