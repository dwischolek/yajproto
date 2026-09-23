//! Parser for YAJ-Proto channel-data frames.
//!
//! Frame layout:
//!
//! - Total frame length, including the checksum
//! - Protocol version
//! - Payload ID
//! - Number of digital channels
//! - Number of analog channels
//! - Variable-length packed digital-channel data
//! - Variable-length little-endian analog `u16` values
//! - Little-endian checksum, calculated as `0xffff` with the frame bytes
//!   subtracted in order

#![deny(clippy::unwrap_used)]
#![deny(clippy::expect_used)]
#![deny(clippy::panic)]
#![deny(unused_must_use)]

use ParseError::{
    ChecksumMismatch, InvalidLength, LengthMismatch, TooShort, UnknownPayloadId,
    UnsupportedProtocolVersion,
};
use std::fmt;

const LENGTH_WITHOUT_VARIABLE_FIELDS: usize = 7;
const VARIABLE_FIELDS_START_OFFSET: usize = 5;
const CHECKSUM_LENGTH: usize = 2;
const PROTOCOL_VERSION: u8 = 0x01;
const CHANNEL_DATA_FRAME_PAYLOAD_ID: u8 = 0x01;

#[derive(Debug, PartialEq, Eq)]
pub struct YajProtoHeader {
    /// Total frame length, including the two checksum bytes.
    pub length: u8,
    /// Protocol version used by the frame.
    pub version: u8,
    /// Payload identifying the frame payload format.
    pub payload_id: u8,
}

#[derive(Debug, PartialEq, Eq)]
pub struct YajProtoChannelDataFrame {
    pub header: YajProtoHeader,
    pub number_of_digital_channels: u8,
    pub number_of_analog_channels: u8,
    /// Digital channel values in channel-number order.
    pub digital_channel_data: Vec<bool>,
    /// Analog channel values in channel-number order.
    pub analog_channel_data: Vec<u16>,
    /// Checksum received in the frame.
    pub checksum: u16,
}

fn calculate_digital_channel_data_length(number_of_digital_channels: u8) -> usize {
    usize::from(number_of_digital_channels).div_ceil(8)
}

fn calculate_analog_channel_data_length(number_of_analog_channels: u8) -> usize {
    2 * usize::from(number_of_analog_channels)
}

fn calculate_total_length(
    digital_channel_data_length: usize,
    analog_channel_data_length: usize,
) -> usize {
    LENGTH_WITHOUT_VARIABLE_FIELDS + digital_channel_data_length + analog_channel_data_length
}

fn parse_digital_channel_data(data: &[u8], number_of_channels: u8) -> Vec<bool> {
    let mut channel_data = Vec::with_capacity(usize::from(number_of_channels));

    for channel in 0..number_of_channels {
        // Digital channels are packed LSB-first; unused bits in the
        // final byte are intentionally ignored by this channel-bounded loop.
        let bit = usize::from(channel) % 8;
        channel_data
            .push(data[VARIABLE_FIELDS_START_OFFSET + usize::from(channel) / 8] >> bit & 0x01 != 0);
    }

    log::debug!("Digital channel data: {:?}", channel_data);
    channel_data
}

fn parse_analog_channel_data(data: &[u8], start: usize, number_of_channels: u8) -> Vec<u16> {
    let mut channel_data = Vec::with_capacity(usize::from(number_of_channels));

    for channel in 0..usize::from(number_of_channels) {
        let offset = start + 2 * channel;
        channel_data.push(u16::from_le_bytes([data[offset], data[offset + 1]]));
    }

    log::debug!("Analog channel data: {:?}", channel_data);
    channel_data
}

fn validate_checksum(data: &[u8]) -> Result<u16, ParseError> {
    let checksum_start = data.len() - CHECKSUM_LENGTH;
    let received_checksum = u16::from_le_bytes([data[checksum_start], data[checksum_start + 1]]);
    // Start at 0xffff and subtract every covered byte with wrapping arithmetic.
    let mut calculated_checksum: u16 = 0xFFFF;
    // The checksum covers every byte excluding the final checksum bytes.
    for &byte in data.iter().take(checksum_start) {
        calculated_checksum = calculated_checksum.wrapping_sub(u16::from(byte));
    }

    if received_checksum != calculated_checksum {
        Err(ChecksumMismatch)
    } else {
        Ok(received_checksum)
    }
}

#[derive(Debug, PartialEq, Eq)]
pub enum ParseError {
    TooShort {
        expected_length: usize,
        actual_length: usize,
    },
    LengthMismatch {
        expected_length_by_header: usize,
        actual_length: usize,
    },
    InvalidLength {
        digital_channel_data_length: usize,
        analog_channel_data_length: usize,
        actual_total_length: usize,
    },
    UnknownPayloadId,
    UnsupportedProtocolVersion,
    ChecksumMismatch,
}

impl fmt::Display for ParseError {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        match self {
            TooShort {
                expected_length,
                actual_length,
            } => write!(
                f,
                "Too short. Expected {} bytes, got {}.",
                expected_length, actual_length
            ),
            LengthMismatch {
                expected_length_by_header,
                actual_length,
            } => write!(
                f,
                "Length mismatch. Expected {} bytes, got {}.",
                expected_length_by_header, actual_length
            ),
            InvalidLength {
                digital_channel_data_length,
                analog_channel_data_length,
                actual_total_length,
            } => write!(
                f,
                "Invalid length. Expected {} bytes of fixed size data, {} bytes for digital channel data and {} bytes for analog channel data, got {}.",
                LENGTH_WITHOUT_VARIABLE_FIELDS,
                digital_channel_data_length,
                analog_channel_data_length,
                actual_total_length
            ),
            UnknownPayloadId => write!(f, "Unknown payload ID."),
            UnsupportedProtocolVersion => write!(f, "Unsupported protocol version."),
            ChecksumMismatch => write!(f, "Checksum mismatch."),
        }
    }
}

impl std::error::Error for ParseError {}

impl TryFrom<&[u8]> for YajProtoChannelDataFrame {
    type Error = ParseError;

    /// Parses one complete, decoded YAJ-Proto channel-data frame.
    fn try_from(data: &[u8]) -> Result<Self, Self::Error> {
        let data_len = data.len();

        if data_len < LENGTH_WITHOUT_VARIABLE_FIELDS {
            return Err(TooShort {
                expected_length: LENGTH_WITHOUT_VARIABLE_FIELDS,
                actual_length: data_len,
            });
        }

        let header = YajProtoHeader {
            length: data[0],
            version: data[1],
            payload_id: data[2],
        };

        if usize::from(header.length) != data_len {
            return Err(LengthMismatch {
                expected_length_by_header: usize::from(header.length),
                actual_length: data_len,
            });
        } else if header.payload_id != CHANNEL_DATA_FRAME_PAYLOAD_ID {
            //TODO: Allow for other payloads.
            return Err(UnknownPayloadId);
        } else if header.version != PROTOCOL_VERSION {
            return Err(UnsupportedProtocolVersion);
        }

        let number_of_digital_channels = data[3];
        let digital_channel_data_length =
            calculate_digital_channel_data_length(number_of_digital_channels);
        let number_of_analog_channels = data[4];
        let analog_channel_data_length =
            calculate_analog_channel_data_length(number_of_analog_channels);
        let analog_channel_data_start = VARIABLE_FIELDS_START_OFFSET + digital_channel_data_length;

        if calculate_total_length(digital_channel_data_length, analog_channel_data_length)
            != data_len
        {
            return Err(InvalidLength {
                digital_channel_data_length,
                analog_channel_data_length,
                actual_total_length: data_len,
            });
        }

        let digital_channel_data = parse_digital_channel_data(data, number_of_digital_channels);
        let analog_channel_data =
            parse_analog_channel_data(data, analog_channel_data_start, number_of_analog_channels);
        let checksum = validate_checksum(data)?;

        Ok(YajProtoChannelDataFrame {
            header,
            number_of_digital_channels,
            number_of_analog_channels,
            digital_channel_data,
            analog_channel_data,
            checksum,
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn with_checksum(mut data: Vec<u8>) -> Vec<u8> {
        // Test fixtures provide the frame without its checksum and with a
        // placeholder length in the first byte.
        data[0] = u8::try_from(data.len() + 2).unwrap();
        let checksum = data.iter().fold(0xffff_u16, |checksum, &byte| {
            checksum.wrapping_sub(u16::from(byte))
        });
        data.extend_from_slice(&checksum.to_le_bytes());
        data
    }

    #[test]
    fn parses_digital_and_analog_channel_data() {
        // A normal frame containing three packed digital and two analog channels.
        let data = with_checksum(vec![
            0,
            0x01,
            0x01,
            3,
            2,
            0b0000_0101,
            0x34,
            0x12,
            0xcd,
            0xab,
        ]);

        let frame = YajProtoChannelDataFrame::try_from(data.as_slice()).unwrap();

        assert_eq!(frame.header.length, 12);
        assert_eq!(frame.digital_channel_data, vec![true, false, true]);
        assert_eq!(frame.analog_channel_data, vec![0x1234, 0xabcd]);
        assert_eq!(frame.checksum, u16::from_le_bytes([data[10], data[11]]));
    }

    #[test]
    fn ignores_padding_bits_after_digital_channels() {
        // Nine channels occupy two bytes; only the first bit of the second
        // byte belongs to a channel, while the remaining bits are padding.
        let data = with_checksum(vec![0, 0x01, 0x01, 9, 0, 0b1000_0001, 0b1111_1111]);

        let frame = YajProtoChannelDataFrame::try_from(data.as_slice()).unwrap();

        assert_eq!(frame.digital_channel_data.len(), 9);
        assert!(frame.digital_channel_data[0]);
        assert!(frame.digital_channel_data[7]);
        assert!(frame.digital_channel_data[8]);
    }

    #[test]
    fn rejects_short_data() {
        // The fixed header, counts, and checksum require at least seven bytes.
        let error = YajProtoChannelDataFrame::try_from([0; 6].as_slice()).unwrap_err();

        assert_eq!(
            error,
            ParseError::TooShort {
                expected_length: 7,
                actual_length: 6,
            }
        );
    }

    #[test]
    fn rejects_header_length_mismatch() {
        // The declared frame length must match the supplied slice length.
        let error =
            YajProtoChannelDataFrame::try_from([6, 1, 1, 0, 0, 0, 0].as_slice()).unwrap_err();

        assert_eq!(
            error,
            ParseError::LengthMismatch {
                expected_length_by_header: 6,
                actual_length: 7,
            }
        );
    }

    #[test]
    fn rejects_length_inconsistent_with_channel_counts() {
        // Channel counts must account for all variable-length payload bytes.
        let data = with_checksum(vec![0, 1, 1, 8, 2, 0, 0, 0]);
        let error = YajProtoChannelDataFrame::try_from(data.as_slice()).unwrap_err();

        assert!(matches!(error, ParseError::InvalidLength { .. }));
    }

    #[test]
    fn rejects_unknown_payload_and_version() {
        // Only the currently supported payload and protocol version are accepted.
        let unknown_payload = with_checksum(vec![0, 1, 2, 0, 0]);
        let error = YajProtoChannelDataFrame::try_from(unknown_payload.as_slice()).unwrap_err();
        assert_eq!(error, ParseError::UnknownPayloadId);

        let unsupported_version = with_checksum(vec![0, 2, 1, 0, 0]);
        let error = YajProtoChannelDataFrame::try_from(unsupported_version.as_slice()).unwrap_err();
        assert_eq!(error, ParseError::UnsupportedProtocolVersion);
    }

    #[test]
    fn rejects_checksum_mismatch() {
        // Mutating a covered byte must invalidate an otherwise valid frame.
        let mut data = with_checksum(vec![0, 1, 1, 0, 0]);
        data[5] ^= 1;

        let error = YajProtoChannelDataFrame::try_from(data.as_slice()).unwrap_err();

        assert_eq!(error, ParseError::ChecksumMismatch);
    }
}
