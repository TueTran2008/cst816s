#![allow(clippy::doc_markdown)]
#![allow(dead_code)]
use embedded_hal::i2c::{I2c, SevenBitAddress};

pub mod constants {
    /// Duration of reset pin low (in ms)
    pub const CST328_RESET_DURATION_LOW_MS: u32 = 20;
    /// Duration of reset pin high (in ms)
    pub const CST328_RESET_DURATION_HIGH_MS: u32 = 50;
    /// Register Touch Data
    pub const CST328_REG_DATA: u16 = 0xD000;
    /// Command deep sleep
    pub const CST328_CMD_DEEP_SLEEP: u16 = 0xD105;
}

/// CST816s, CST816D, CST816T
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u16)]
pub enum TouchRegister {
    NumberReg    = 0xD005,
    XYReg        = 0xD000,
    ChecksumReg  = 0x80FF,
    ConfigReg    = 0x8047,   
    FirstX       = 0xD001,
    FirstY       = 0xD002,
    FirstXY      = 0xD003,
}


impl TouchRegister {

    pub fn to_bytes_le(self) -> [u8; 2] {
        (self as u16).to_le_bytes()
    }

    pub fn to_bytes_be(self) -> [u8; 2] {
        (self as u16).to_be_bytes()
    }
    /// Returns the register
    /// # Errors
    ///
    /// This method may return an error if there are communication issues with the sensor.
    pub fn read<I>(&self, interface: &mut I, addr: u8) -> Result<u8, I::Error>
    where
        I: I2c<SevenBitAddress>,
    {
        let read = self.to_bytes_be();
        let mut buffer = [0; 1];
        interface.write_read(addr, &read, &mut buffer)?;
        Ok(buffer[0])
    }

    /// Writes the register
    /// # Errors
    ///
    /// This method may return an error if there are communication issues with the sensor.
    pub fn write<I>(&self, interface: &mut I, addr: u8, value: u8) -> Result<(), I::Error>
    where
        I: I2c<SevenBitAddress>,
    {
        let reg = self.to_bytes_be();
        let buffer = [reg[0], reg[1], value];
        interface.write(addr, &buffer)?;
        Ok(())
    }

    // pub fn read_xy<I>(&self, interface: &mut I, addr: u8) -> Result<[u8; 27], I::Error>
    // where
    //     I: I2c<SevenBitAddress>,
    // {
    //     let read = self.to_bytes_be();
    //     let mut buffer = [0; 27];
    //     interface.write_read(addr, &read, &mut buffer)?;
    //     Ok(buffer)
    // }
}
// /// CST816s, CST816D, CST816T
// #[derive(Debug, Clone, Copy, PartialEq, Eq)]
// #[repr(u8)]
// pub enum Register {
//     /// Gesture ID
//     GestureId = 0x01,
//     /// Finger Numbers
//     FingerNumber = 0x02,
//     /// High 4 bits of the X coordinate
//     XposH = 0x03,
//     /// Low 8 bits of the X coordinate
//     XposL = 0x04,
//     /// High 4 bits of the Y coordinate
//     YposH = 0x05,
//     /// Low 8 bits of the Y coordinate
//     YposL = 0x06,
//     /// Touch XY POS
//     XYpos = 0x07,
//     /// Misc
//     Misc = 0x08,
//     /// High 8 bits of the BPC0H value
//     BPC0H = 0xB0,
//     /// Low 8 bits of the BPC0L value
//     BPC0L = 0xB1,
//     /// High 8 bits of the BPC1H value
//     BPC1H = 0xB2,
//     /// Low 8 bits of the BPC1L value
//     BPC1L = 0xB3,
//     /// Chip Id
//     ChipId = 0xA7,
//     /// Project Id
//     ProjectId = 0xA8,
//     /// Firmware Version
//     FirmwareVersion = 0xA9,
//     /// Deep Sleep
//     DeepSleep = 0xE5,
//     /// Motion Mask
//     MotionMask = 0xEC,
//     /// IrqPulseWidth
//     IrqPluseWidth = 0xED,
//     /// NorScanPer
//     NorScanPer = 0xEE,
//     /// MotionSlAngle
//     MotionSlAngle = 0xEF,
//     /// LpScanRaw1H
//     LpScanRaw1H = 0xF0,
//     /// LpScanRaw1L
//     LpScanRaw1L = 0xF1,
//     /// LpScanRaw2H
//     LpScanRaw2H = 0xF2,
//     /// LpScanRaw2L
//     LpScanRaw2L = 0xF3,
//     /// LpAutoWakeTime
//     LpAutoWakeTime = 0xF4,
//     /// LpScanTH
//     LpScanTH = 0xF5,
//     /// LpScanWin
//     LpScanWin = 0xF6,
//     /// LpScanFreq
//     LpScanFreq = 0xF7,
//     /// LpScanIdac
//     LpScanIdac = 0xF8,
//     /// AutoSleepTime
//     AutoSleepTime = 0xF9,
//     /// IrqCtl
//     IrqCtl = 0xFA,
//     /// AutoReset
//     AutoReset = 0xFB,
//     /// LongPressTime
//     LongPressTime = 0xFC,
//     /// IOCtl
//     IOCtl = 0xFD,
//     /// DisAutoSleep
//     DisAutoSleep = 0xFE,
// }

// impl Register {
//     /// Returns the register
//     /// # Errors
//     ///
//     /// This method may return an error if there are communication issues with the sensor.
//     pub fn read<I>(&self, interface: &mut I, addr: u8) -> Result<u8, I::Error>
//     where
//         I: I2c<SevenBitAddress>,
//     {
//         let read = [*self as u8; 1];
//         let mut buffer = [0; 1];
//         interface.write_read(addr, &read, &mut buffer)?;
//         Ok(buffer[0])
//     }

//     /// Writes the register
//     /// # Errors
//     ///
//     /// This method may return an error if there are communication issues with the sensor.
//     pub fn write<I>(&self, interface: &mut I, addr: u8, value: u8) -> Result<(), I::Error>
//     where
//         I: I2c<SevenBitAddress>,
//     {
//         let buffer = [*self as u8, value];
//         interface.write(addr, &buffer)?;
//         Ok(())
//     }
// }

// /// Read chip ID
// ///
// /// # Notes
// ///
// /// The chip is blank when it leaves the factory, IIC is not connected, there is no chip ID, and
// /// the value can only be read after burning and upgrading the firmeware.
// #[derive(Debug, PartialEq, Eq, Clone, Copy)]
// pub enum ChipId {
//     Factory = 0x00,
//     Cst716 = 0x20,
//     Cst816s = 0xB4,
//     Cst816t = 0xB5,
//     Cst816d = 0xB6,
// }

// #[derive(Debug, PartialEq, Eq, Default, Clone, Copy)]
// /// CST816S Touch event touch state
// pub enum Touch {
//     #[default]
//     Down = 0x00,
//     Up = 0x01,
//     Contact = 0x02,
// }

// impl From<u8> for Touch {
//     fn from(value: u8) -> Self {
//         if value & 0b0100_0000 > 0 {
//             Self::Up
//         } else if value & 0b1000_0000 > 0 {
//             Self::Contact
//         } else {
//             Self::Down
//         }
//     }
// }

// #[derive(Debug, PartialEq, Eq, Default, Clone, Copy)]
// /// CST816S Gesture types
// pub enum Gesture {
//     /// No gesture detected
//     #[default]
//     None = 0x00,
//     /// Downward slide detected
//     SlideDown = 0x01,
//     /// Upward slide detected
//     SlideUp = 0x02,
//     /// Left slide detected
//     SlideLeft = 0x03,
//     /// Right slide detected
//     SlideRight = 0x04,
//     /// Single click detected
//     SingleClick = 0x05,
//     /// Double click detected
//     DoubleClick = 0x0b,
//     /// Long press detected
//     LongPress = 0x0c,
// }

// impl From<u8> for Gesture {
//     fn from(value: u8) -> Self {
//         match value {
//             0x01 => Self::SlideDown,
//             0x02 => Self::SlideUp,
//             0x03 => Self::SlideLeft,
//             0x04 => Self::SlideRight,
//             0x05 => Self::SingleClick,
//             0x0b => Self::DoubleClick,
//             0x0c => Self::LongPress,
//             _ => Self::None,
//         }
//     }
// }

// #[derive(Default, Clone, Copy)]
// pub struct TouchEvents {
//     pub points: [TouchEvent; 10],
//     pub number_points: u8,
//     pub finger_number: u8,
// }

// impl core::fmt::Debug for TouchEvents {
//     fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
//         f.debug_struct("TouchEvents")
//             .field("finger_number", &self.finger_number)
//             .field("number_points", &self.number_points)
//             // Display only the points up to `number_points`
//             .field("points", &&self.points[..self.number_points as usize])
//             .finish()
//     }
// }

// impl TouchEvents {
//     /// Returns the registers
//     ///
//     /// # Notes
//     ///
//     /// From [`TouchEvents`] you must compute some **blackmagic** to have [`KeyEvent`].
//     /// A [`KeyEvent`] must be compute with `TouchEvents::report_key`.
//     ///
//     /// # Errors
//     ///
//     /// This method may return an error if there are communication issues with the sensor.
//     pub fn read<I>(interface: &mut I, addr: u8) -> Result<Self, I::Error>
//     where
//         I: I2c<SevenBitAddress>,
//     {
//         let read = [constants::CST328_REG_DATA; 1];
//         let mut data = Self::default();
//         let mut buffer = [0; 3 + 6 * 10];
//         interface.write_read(addr, &read, &mut buffer)?;

//         data.finger_number = buffer[Register::FingerNumber as usize] & 0x0f;

//         for i in 0..5 {
//             let point_id = buffer[(Register::YposH as usize) + 6 * i] >> 4;

//             if point_id > 0x0F {
//                 break;
//             }

//             data.number_points += 1;

//             let x_high = buffer[Register::XposH as usize + 6 * i] & 0x0f;
//             let x_low = buffer[Register::XposL as usize + 6 * i];

//             let y_high = buffer[Register::YposH as usize + 6 * i] & 0x0f;
//             let y_low = buffer[Register::YposL as usize + 6 * i];

//             let x: u16 = (u16::from(x_high) << 8) | u16::from(x_low);
//             let y: u16 = (u16::from(y_high) << 8) | u16::from(y_low);

//             data.points[i].x = x;
//             data.points[i].y = y;
//             data.points[i].gesture_type =
//                 Gesture::from(buffer[Register::GestureId as usize + 6 * i]);
//             data.points[i].touch_type = Touch::from(buffer[Register::XposH as usize + 6 * i]);
//             data.points[i].touch_id = buffer[Register::YposH as usize + 6 * i] >> 4;
//             data.points[i].pressure = buffer[Register::XYpos as usize + 6 * i];
//             data.points[i].area = buffer[Register::Misc as usize + 6 * i] >> 4;
//             data.points[i].finger_number = data.finger_number;

//             if (data.points[i].touch_type == Touch::Down
//                 || data.points[i].touch_type == Touch::Contact)
//                 && data.finger_number == 0
//             {
//                 break;
//             }
//         }

//         Ok(data)
//     }

//     /// Returns a [`KeyEvent`]
//     ///
//     /// # Notes
//     ///
//     /// This is an attempt to support the IC / firmware logic...
//     ///
//     /// It seems like this IC is badly implemented at firmware level and the documentation
//     /// is badly written too.
//     ///
//     #[must_use]
//     pub fn report_key<const W: u16, const H: u16>(&self) -> Option<KeyEvent> {
//         let mut i = 0;

//         while i < 5 {
//             if self.points[i].y <= H && self.points[i].x <= W {
//                 break;
//             }
//             i += 1;
//         }

//         if self.points[i].touch_type == Touch::Down || self.points[i].touch_type == Touch::Contact {
//             Some(KeyEvent {
//                 x: self.points[i].x,
//                 y: self.points[i].y,
//                 state: KeyState::Down,
//             })
//         } else {
//             Some(KeyEvent {
//                 x: self.points[i].x,
//                 y: self.points[i].y,
//                 state: KeyState::Up,
//             })
//         }
//     }
// }

// #[derive(Debug, Clone, Copy, Default)]
// pub enum KeyState {
//     #[default]
//     Down = 0,
//     Up = 1,
// }

// #[derive(Debug, Clone, Copy, Default)]
// pub struct KeyEvent {
//     /// X coordinate
//     pub x: u16,
//     /// Y coordinate
//     pub y: u16,
//     /// Press or release event
//     pub state: KeyState,
// }

// #[derive(Debug, Default, Clone, Copy)]
// pub struct TouchEvent {
//     /// Touch id
//     pub touch_id: u8,
//     // Finger number
//     pub finger_number: u8,
//     /// Detected gesture
//     pub gesture_type: Gesture,
//     /// Press or release event
//     pub touch_type: Touch,
//     /// X coordinate
//     pub x: u16,
//     /// Y coordinate
//     pub y: u16,
//     /// Pressure
//     pub pressure: u8,
//     /// Area
//     pub area: u8,
// }

// impl TouchEvent {
//     /// Returns the registers
//     ///
//     /// # Errors
//     ///
//     /// This method may return an error if there are communication issues with the sensor.
//     pub fn read<I>(interface: &mut I, addr: u8) -> Result<Self, I::Error>
//     where
//         I: I2c<SevenBitAddress>,
//     {
//         let read = [constants::CST328_REG_DATA; 1];
//         let mut buffer = [0; 7];
//         interface.write_read(addr, &read, &mut buffer)?;
//         Ok(Self::from_buffer(&buffer))
//     }

//     /// Returns [`TouchEvent`] from a buffer of length 7
//     #[allow(clippy::trivially_copy_pass_by_ref)]
//     fn from_buffer(buffer: &[u8; 7]) -> Self {
//         let gesture_type = Gesture::from(buffer[Register::GestureId as usize] & 0x0f);
//         let finger_number = buffer[Register::FingerNumber as usize];

//         let touch_id = (buffer[Register::YposH as usize]) >> 4;

//         let x_high = buffer[Register::XposH as usize] & 0x0f;
//         let x_low = buffer[Register::XposL as usize];

//         let y_high = buffer[Register::YposH as usize] & 0x0f;
//         let y_low = buffer[Register::YposL as usize];

//         let touch_type = Touch::from(buffer[Register::XposH as usize]); /* 0 = Touch Down, 1 = Touch Up, 2 = Contact */

//         let x: u16 = (u16::from(x_high) << 8) | u16::from(x_low);
//         let y: u16 = (u16::from(y_high) << 8) | u16::from(y_low);

//         Self {
//             touch_id,
//             finger_number,
//             gesture_type,
//             touch_type,
//             x,
//             y,
//             pressure: 0,
//             area: 0,
//         }
//     }
// }
