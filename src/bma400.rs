use embedded_hal::blocking::i2c::{Read, Write, WriteRead};

pub struct Bma400 {
	addr: u8
}

impl Bma400 {
	pub fn new(sdo_position: bool) -> Bma400 {
		Bma400 {
			// lowest bit of address is set by sdo being low or high
			addr: 0b0010100 | (sdo_position as u8),
		}
	}

	pub fn read_addr<I2C>(&self, i2c: &mut I2C, addr: Register) -> Result<u8, <I2C as WriteRead>::Error>
	where I2C: WriteRead
	{
		let mut buf = [0; 1];
        i2c.write_read(self.addr, &[addr as u8], &mut buf)?;
        Ok(buf[0])
	}

	pub fn write_addr<I2C>(&self, i2c: &mut I2C, addr: Register, value: u8) -> Result<(), <I2C as Write>::Error>
	where I2C: Write
	{
        i2c.write(self.addr, &[addr as u8, value])?;
        Ok(())
	}


	pub fn wake<I2C>(&self, i2c: &mut I2C) -> Result<(), <I2C as Write>::Error>
	where I2C: Write
	{
        self.write_addr(i2c, Register::AccelConfig0, 0x02)?;
        Ok(())
	}

	pub fn get_int_status<I2C>(&self, i2c: &mut I2C) -> Result<u8, <I2C as WriteRead>::Error>
	where I2C: WriteRead
	{
        self.read_addr(i2c, Register::IntStat0)
	}

	pub fn configure_inactivity_interupt<I2C>(&self, i2c: &mut I2C, delay: u16) -> Result<(), <I2C as Write>::Error>
	where I2C: Write
	{
		// map gen1 to pin1
		self.write_addr(i2c, Register::IntMap1, 0x04)?;
		// non-latching
		self.write_addr(i2c, Register::IntConf1, 0x00)?;
		// int1 pin interrupt active high
		self.write_addr(i2c, Register::Int12IoCtrl, 0x02)?;
		// enable x, y, z axis data source = acc_filt2 (fixed 100 Hz)
		// update reference every time, hysteresis = 48 mg
		self.write_addr(i2c, Register::Gen1IntConfig0, 0xFA)?;
		// or all the things, inactivity triggers
		self.write_addr(i2c, Register::Gen1IntConfig1, 0x00)?;
		// threshold is 8mb/lsb
		self.write_addr(i2c, Register::Gen1IntConfig2, 0x01)?;
		// set min duration
		self.write_addr(i2c, Register::Gen1IntConfig3, (delay >> 8) as u8)?;
		self.write_addr(i2c, Register::Gen1IntConfig31, delay as u8)?;
		// enable gen1 interrupt in normal mode
		self.write_addr(i2c, Register::IntConf0, 0x04)?;
		Ok(())
	}

}

use num_traits::cast::FromPrimitive;
use num_traits::cast::ToPrimitive;

enum_from_primitive! {
#[derive(Debug, PartialEq)]
pub enum Register {
    ChipId = 0x00,
	Status = 0x03,
	AccelData = 0x04,
	IntStat0 = 0x0E,
	TempData = 0x11,
	FifoLength = 0x12,
	FifoData = 0x14,
	StepCnt0 = 0x15,
	AccelConfig0 = 0x19,
	AccelConfig1 = 0x1A,
	AccelConfig2 = 0x1B,
	IntConf0 = 0x1F,
	IntConf1 = 0x20,
	Int12IoCtrl = 0x24,
	IntMap1 = 0x21,
	IntMap2 = 0x22,
	FifoConfig0 = 0x26,
	FifoReadEn = 0x29,
	AutoLowPow0 = 0x2A,
	AutoLowPow1 = 0x2B,
	AutoWakeup0 = 0x2C,
	AutoWakeup1 = 0x2D,
	WakeupIntConf0 = 0x2F,
	Gen1IntConfig0 = 0x3F,
	Gen1IntConfig1 = 0x40,
	Gen1IntConfig2 = 0x41,
	Gen1IntConfig3 = 0x42,
	Gen1IntConfig31 = 0x43,
	Gen1IntConfig4 = 0x44,
	Gen1IntConfig5 = 0x45,
	Gen1IntConfig6 = 0x46,
	Gen1IntConfig7 = 0x47,
	Gen1IntConfig8 = 0x48,
	Gen1IntConfig9 = 0x49,
	Gen2IntConfig0 = 0x4A,
	Gen2IntConfig1 = 0x4B,
	Gen2IntConfig2 = 0x4C,
	Gen2IntConfig3 = 0x4D,
	Gen2IntConfig31 = 0x4E,
	Gen2IntConfig4 = 0x4F,
	Gen2IntConfig5 = 0x50,
	Gen2IntConfig6 = 0x51,
	Gen2IntConfig7 = 0x52,
	Gen2IntConfig8 = 0x53,
	Gen2IntConfig9 = 0x54,
	ActChConfig0 = 0x55,
	ActChConfig1 = 0x56,
	TapConfig0 = 0x57,
	TapConfig1 = 0x58,
	IfConf = 0x7C,			//serial interface settings
	SelfTest = 0x7D,
	CommandReg = 0x7E,
}
}