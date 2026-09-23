//! Bridges a YAJ-Proto device on a serial port onto a Linux uinput gamepad.
//!
//! Frames arrive COBS-encoded and zero-delimited. Because COBS output is
//! zero-free, every `0x00` on the wire is a real frame boundary, so reading up
//! to the delimiter always lands on a frame edge. The one exception is the
//! *first* chunk after joining or interrupting the stream, which may be the tail of
//! a frame that was already in flight; see [`Resync`].

#![deny(clippy::unwrap_used)]
#![deny(clippy::expect_used)]
#![deny(clippy::panic)]
#![deny(unused_must_use)]

use std::env;
use std::fmt;
use std::io::{self, BufRead, BufReader};
use std::time::Duration;

use evdev::uinput::VirtualDevice;
use evdev::{
    AbsInfo, AbsoluteAxisCode, AbsoluteAxisEvent, AttributeSet, InputEvent, KeyCode, KeyEvent,
    UinputAbsSetup,
};
use yajproto::YajProtoChannelDataFrame;

const KEY_CODE_LIST: &[KeyCode] = &[
    KeyCode::BTN_TRIGGER_HAPPY1,
    KeyCode::BTN_TRIGGER_HAPPY2,
    KeyCode::BTN_TRIGGER_HAPPY3,
    KeyCode::BTN_TRIGGER_HAPPY4,
    KeyCode::BTN_TRIGGER_HAPPY5,
    KeyCode::BTN_TRIGGER_HAPPY6,
    KeyCode::BTN_TRIGGER_HAPPY7,
    KeyCode::BTN_TRIGGER_HAPPY8,
    KeyCode::BTN_TRIGGER_HAPPY9,
    KeyCode::BTN_TRIGGER_HAPPY10,
    KeyCode::BTN_TRIGGER_HAPPY11,
    KeyCode::BTN_TRIGGER_HAPPY12,
    KeyCode::BTN_TRIGGER_HAPPY13,
    KeyCode::BTN_TRIGGER_HAPPY14,
    KeyCode::BTN_TRIGGER_HAPPY15,
    KeyCode::BTN_TRIGGER_HAPPY16,
    KeyCode::BTN_TRIGGER_HAPPY17,
    KeyCode::BTN_TRIGGER_HAPPY18,
    KeyCode::BTN_TRIGGER_HAPPY19,
    KeyCode::BTN_TRIGGER_HAPPY20,
    KeyCode::BTN_TRIGGER_HAPPY21,
    KeyCode::BTN_TRIGGER_HAPPY22,
    KeyCode::BTN_TRIGGER_HAPPY23,
    KeyCode::BTN_TRIGGER_HAPPY24,
    KeyCode::BTN_TRIGGER_HAPPY25,
    KeyCode::BTN_TRIGGER_HAPPY26,
    KeyCode::BTN_TRIGGER_HAPPY27,
    KeyCode::BTN_TRIGGER_HAPPY28,
    KeyCode::BTN_TRIGGER_HAPPY29,
    KeyCode::BTN_TRIGGER_HAPPY30,
    KeyCode::BTN_TRIGGER_HAPPY31,
    KeyCode::BTN_TRIGGER_HAPPY32,
    KeyCode::BTN_TRIGGER_HAPPY33,
    KeyCode::BTN_TRIGGER_HAPPY34,
    KeyCode::BTN_TRIGGER_HAPPY35,
    KeyCode::BTN_TRIGGER_HAPPY36,
    KeyCode::BTN_TRIGGER_HAPPY37,
    KeyCode::BTN_TRIGGER_HAPPY38,
    KeyCode::BTN_TRIGGER_HAPPY39,
    KeyCode::BTN_TRIGGER_HAPPY40,
];

const AXIS_CODE_LIST: &[AbsoluteAxisCode] = &[
    AbsoluteAxisCode::ABS_X,
    AbsoluteAxisCode::ABS_Y,
    AbsoluteAxisCode::ABS_Z,
    AbsoluteAxisCode::ABS_RX,
    AbsoluteAxisCode::ABS_RY,
];

const DEFAULT_DEVICE_NAME: &str = "YAJ-Proto";
const DEFAULT_PORT_NAME: &str = "/dev/ttyACM0";
const BAUD_RATE: u32 = 115_200;
const READ_TIMEOUT: Duration = Duration::from_secs(10);

/// COBS frame delimiter. Encoded frames are zero-free, so this byte only ever
/// marks a frame boundary.
const FRAME_DELIMITER: u8 = 0x00;

/// The last two digital channels are reported as `BTN_0` and `BTN_1` rather
/// than from the `BTN_TRIGGER_HAPPY*` range.
const DEDICATED_BUTTON_COUNT: usize = 2;

/// Bytes allowed to queue up in the driver before the backlog is dropped to
/// catch up with the device.
const MAX_BYTES_WAITING: u32 = 2000;

/// Undecodable chunks tolerated in a row before the link itself is suspect.
/// One covers joining the stream mid-frame, one covers a dropped backlog.
const RESYNC_TOLERANCE: u32 = 2;

/// How often to repeat the warning while the stream stays undecodable.
const RESYNC_WARN_INTERVAL: u32 = 1000;

/// An error that ends the session; anything recoverable is handled in place.
#[derive(Debug)]
enum FatalError {
    Arguments(String),
    OpenPort {
        port_name: String,
        error: serialport::Error,
    },
    Serial(serialport::Error),
    Read {
        port_name: String,
        error: io::Error,
    },
    Disconnected(String),
    Uinput(io::Error),
    TooFewDigitalChannels {
        count: u8,
    },
    TooManyDigitalChannels {
        count: u8,
        supported: usize,
    },
    TooManyAnalogChannels {
        count: u8,
        supported: usize,
    },
    ChannelLayoutChanged {
        digital: u8,
        analog: u8,
        expected_digital: usize,
        expected_analog: usize,
    },
}

impl fmt::Display for FatalError {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        match self {
            FatalError::Arguments(message) => write!(f, "{message}"),
            FatalError::OpenPort { port_name, error } => {
                write!(f, "Failed to open \"{port_name}\". Error: {error}")
            }
            FatalError::Serial(error) => write!(f, "Serial port error: {error}"),
            FatalError::Read { port_name, error } => {
                write!(f, "Failed to read from \"{port_name}\": {error}")
            }
            FatalError::Disconnected(port_name) => {
                write!(f, "\"{port_name}\" closed the connection.")
            }
            FatalError::Uinput(error) => write!(f, "uinput device error: {error}"),
            FatalError::TooFewDigitalChannels { count } => write!(
                f,
                "Device reports {count} digital channels, but at least {DEDICATED_BUTTON_COUNT} are required."
            ),
            FatalError::TooManyDigitalChannels { count, supported } => write!(
                f,
                "Device reports {count} digital channels, but only {supported} can be mapped."
            ),
            FatalError::TooManyAnalogChannels { count, supported } => write!(
                f,
                "Device reports {count} analog channels, but only {supported} can be mapped."
            ),
            FatalError::ChannelLayoutChanged {
                digital,
                analog,
                expected_digital,
                expected_analog,
            } => write!(
                f,
                "Channel layout changed from {expected_digital} digital and {expected_analog} analog channels to {digital} and {analog}."
            ),
        }
    }
}

impl std::error::Error for FatalError {}

/// A chunk that could not be turned into a frame. Expected occasionally, see
/// [`Resync`].
#[derive(Debug)]
enum FrameError {
    Decode(cobs::DecodeError),
    Parse(yajproto::ParseError),
}

impl fmt::Display for FrameError {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        match self {
            FrameError::Decode(error) => write!(f, "COBS decoding failed: {error}"),
            FrameError::Parse(error) => write!(f, "{error}"),
        }
    }
}

fn hex(bytes: &[u8]) -> String {
    bytes
        .iter()
        .map(|byte| format!("{byte:02x}"))
        .collect::<Vec<_>>()
        .join(" ")
}

/// Decodes one zero-delimited COBS chunk into a channel-data frame.
fn decode_frame(chunk: &[u8]) -> Result<YajProtoChannelDataFrame, FrameError> {
    // Decoding never expands the input, so the chunk length is always a
    // sufficient destination size and the decoded length is reported back.
    let mut decoded = vec![0u8; chunk.len()];
    let report = cobs::decode(chunk, &mut decoded).map_err(FrameError::Decode)?;
    YajProtoChannelDataFrame::try_from(&decoded[..report.frame_size()]).map_err(FrameError::Parse)
}

/// Maps digital channels onto evdev key codes in channel order.
fn key_codes_for(number_of_channels: u8) -> Result<Vec<KeyCode>, FatalError> {
    let trigger_happy_count = usize::from(number_of_channels)
        .checked_sub(DEDICATED_BUTTON_COUNT)
        .ok_or(FatalError::TooFewDigitalChannels {
            count: number_of_channels,
        })?;

    if trigger_happy_count > KEY_CODE_LIST.len() {
        return Err(FatalError::TooManyDigitalChannels {
            count: number_of_channels,
            supported: KEY_CODE_LIST.len() + DEDICATED_BUTTON_COUNT,
        });
    }

    Ok(KEY_CODE_LIST
        .iter()
        .take(trigger_happy_count)
        .copied()
        .chain([KeyCode::BTN_0, KeyCode::BTN_1])
        .collect())
}

/// Maps analog channels onto evdev absolute axis codes in channel order.
fn axis_codes_for(number_of_channels: u8) -> Result<Vec<AbsoluteAxisCode>, FatalError> {
    if usize::from(number_of_channels) > AXIS_CODE_LIST.len() {
        return Err(FatalError::TooManyAnalogChannels {
            count: number_of_channels,
            supported: AXIS_CODE_LIST.len(),
        });
    }

    Ok(AXIS_CODE_LIST
        .iter()
        .take(usize::from(number_of_channels))
        .copied()
        .collect())
}

fn build_virtual_device(
    name: &str,
    keys: &[KeyCode],
    axes: &[AbsoluteAxisCode],
) -> io::Result<VirtualDevice> {
    let mut key_set = AttributeSet::<KeyCode>::new();
    for key in keys {
        key_set.insert(*key);
    }

    let mut builder = VirtualDevice::builder()?.name(name).with_keys(&key_set)?;
    let axis_setup = AbsInfo::new(512, 0, 1024, 2, 0, 1);
    for axis in axes {
        builder = builder.with_absolute_axis(&UinputAbsSetup::new(*axis, axis_setup))?;
    }

    builder.build()
}

/// A uinput device together with the channel-to-code mapping it was built for.
///
/// Holding the codes alongside the device keeps every emit loop a zip over two
/// equally long lists, so no channel count coming off the wire can index out of
/// bounds.
struct GameController {
    device: VirtualDevice,
    keys: Vec<KeyCode>,
    axes: Vec<AbsoluteAxisCode>,
}

impl GameController {
    fn new(frame: &YajProtoChannelDataFrame, device_name: &str) -> Result<Self, FatalError> {
        let keys = key_codes_for(frame.number_of_digital_channels)?;
        let axes = axis_codes_for(frame.number_of_analog_channels)?;
        log::info!(
            "Creating input device {device_name} with {} keys and {} axes",
            keys.len(),
            axes.len()
        );
        let device = build_virtual_device(device_name, &keys, &axes).map_err(FatalError::Uinput)?;

        Ok(GameController { device, keys, axes })
    }

    /// The device is built once from the first frame, so a later frame that
    /// declares a different layout cannot be mapped onto it.
    fn check_layout(&self, frame: &YajProtoChannelDataFrame) -> Result<(), FatalError> {
        if usize::from(frame.number_of_digital_channels) != self.keys.len()
            || usize::from(frame.number_of_analog_channels) != self.axes.len()
        {
            return Err(FatalError::ChannelLayoutChanged {
                digital: frame.number_of_digital_channels,
                analog: frame.number_of_analog_channels,
                expected_digital: self.keys.len(),
                expected_analog: self.axes.len(),
            });
        }

        Ok(())
    }

    /// Emits one event per channel whose value differs from `last`. Without a
    /// previous frame every channel counts as changed.
    fn emit_changes(
        &mut self,
        frame: &YajProtoChannelDataFrame,
        last: Option<&YajProtoChannelDataFrame>,
    ) -> Result<(), FatalError> {
        let mut events: Vec<InputEvent> = Vec::new();

        for (channel, (&key, &pressed)) in self
            .keys
            .iter()
            .zip(frame.digital_channel_data.iter())
            .enumerate()
        {
            let previous = last.and_then(|last| last.digital_channel_data.get(channel));
            if previous != Some(&pressed) {
                events.push(*KeyEvent::new(key, i32::from(pressed)));
            }
        }

        for (channel, (&axis, &value)) in self
            .axes
            .iter()
            .zip(frame.analog_channel_data.iter())
            .enumerate()
        {
            let previous = last.and_then(|last| last.analog_channel_data.get(channel));
            if previous != Some(&value) {
                events.push(*AbsoluteAxisEvent::new(axis, i32::from(value)));
            }
        }

        if events.is_empty() {
            return Ok(());
        }

        self.device.emit(&events).map_err(FatalError::Uinput)
    }
}

/// Tracks how long the stream has been unreadable.
///
/// Reading up to the delimiter is itself the resynchronisation mechanism: a
/// truncated chunk is discarded and the next read starts on a frame boundary by
/// construction. Retrying is therefore pointless, and flushing is harmful — it
/// cuts the backlog at an arbitrary byte and creates the very truncation it is
/// meant to clear. All this needs to do is tell a one-off truncation apart from
/// a link that is misconfigured.
#[derive(Default)]
struct Resync {
    consecutive_rejections: u32,
}

impl Resync {
    fn on_frame_decoded(&mut self) {
        if self.consecutive_rejections > 0 {
            log::debug!(
                "Resynchronised after discarding {} chunks",
                self.consecutive_rejections
            );
            self.consecutive_rejections = 0;
        }
    }

    fn on_frame_rejected(&mut self, error: &FrameError, chunk: &[u8]) {
        self.consecutive_rejections += 1;
        let rejections = self.consecutive_rejections;

        if rejections <= RESYNC_TOLERANCE {
            log::debug!(
                "Discarding chunk and resynchronising: {error} [{}]",
                hex(chunk)
            );
        } else if rejections == RESYNC_TOLERANCE + 1 {
            log::warn!(
                "{rejections} consecutive undecodable chunks: {error}. Check the baud rate and the firmware protocol version. [{}]",
                hex(chunk)
            );
        } else if rejections.is_multiple_of(RESYNC_WARN_INTERVAL) {
            log::warn!("Still undecodable after {rejections} chunks: {error}");
        }
    }
}

struct GameControllerSession {
    device_name: String,
    game_controller: Option<GameController>,
    last_frame: Option<YajProtoChannelDataFrame>,
    resync: Resync,
}

impl GameControllerSession {
    fn new(device_name: String) -> Self {
        GameControllerSession {
            device_name,
            game_controller: None,
            last_frame: None,
            resync: Resync::default(),
        }
    }

    fn handle_chunk(&mut self, chunk: &[u8]) -> Result<(), FatalError> {
        // A chunk holding nothing but the delimiter carries no frame.
        if chunk.len() <= 1 {
            return Ok(());
        }

        let frame = match decode_frame(chunk) {
            Ok(frame) => {
                self.resync.on_frame_decoded();
                frame
            }
            Err(error) => {
                self.resync.on_frame_rejected(&error, chunk);
                return Ok(());
            }
        };

        if self.last_frame.as_ref() == Some(&frame) {
            return Ok(());
        }
        log::debug!("YAJ: {frame:?}");

        if self.game_controller.is_none() {
            self.game_controller = Some(GameController::new(&frame, &self.device_name)?);
        }
        if let Some(game_controller) = self.game_controller.as_mut() {
            game_controller.check_layout(&frame)?;
            game_controller.emit_changes(&frame, self.last_frame.as_ref())?;
        }
        self.last_frame = Some(frame);

        Ok(())
    }
}

struct Arguments {
    device_name: String,
    port_name: String,
}

impl Arguments {
    fn parse() -> Result<Self, FatalError> {
        let mut device_name = DEFAULT_DEVICE_NAME.to_owned();
        let mut port_name = DEFAULT_PORT_NAME.to_owned();
        let mut arguments = env::args().skip(1);

        while let Some(argument) = arguments.next() {
            let (option, value) = match argument.split_once('=') {
                Some((option, value)) => (option, value.to_owned()),
                None => {
                    let value = arguments.next().ok_or_else(|| {
                        FatalError::Arguments(format!("Missing value for {argument}."))
                    })?;
                    (argument.as_str(), value)
                }
            };

            match option {
                "--device-name" => device_name = value,
                "--port-name" => port_name = value,
                _ => {
                    return Err(FatalError::Arguments(format!(
                        "Unknown option {option}. Use --device-name <name> and --port-name <path>."
                    )));
                }
            }
        }

        Ok(Self {
            device_name,
            port_name,
        })
    }
}

fn run(arguments: Arguments) -> Result<(), FatalError> {
    let port = serialport::new(&arguments.port_name, BAUD_RATE)
        .timeout(READ_TIMEOUT)
        .open()
        .map_err(|error| FatalError::OpenPort {
            port_name: arguments.port_name.clone(),
            error,
        })?;
    log::info!(
        "Receiving data on {} at {BAUD_RATE} baud:",
        arguments.port_name
    );

    let mut reader = BufReader::new(port);
    let mut session = GameControllerSession::new(arguments.device_name);
    // Kept across iterations so that a read timing out part way through a frame
    // does not truncate it.
    let mut chunk: Vec<u8> = Vec::new();

    loop {
        match reader.read_until(FRAME_DELIMITER, &mut chunk) {
            Ok(0) => return Err(FatalError::Disconnected(arguments.port_name.clone())),
            Ok(bytes) => log::debug!("Received {bytes} bytes: [{}]", hex(&chunk)),
            // Nothing arrived in time; keep whatever was already read.
            Err(ref error) if error.kind() == io::ErrorKind::TimedOut => continue,
            Err(error) => {
                return Err(FatalError::Read {
                    port_name: arguments.port_name.clone(),
                    error,
                });
            }
        }

        session.handle_chunk(&chunk)?;
        chunk.clear();

        let bytes_waiting = reader
            .get_ref()
            .bytes_to_read()
            .map_err(FatalError::Serial)?;
        log::debug!("Bytes waiting: {bytes_waiting}");
        if bytes_waiting > MAX_BYTES_WAITING {
            // This cuts the backlog at an arbitrary byte, so the next chunk is
            // expected to be a frame tail that fails to decode.
            log::debug!("Dropping {bytes_waiting} buffered bytes to catch up");
            reader
                .get_mut()
                .clear(serialport::ClearBuffer::Input)
                .map_err(FatalError::Serial)?;
        }
    }
}

fn main() {
    env_logger::builder()
        .filter_level(log::LevelFilter::Info)
        .format_target(false)
        .format_timestamp(None)
        .init();

    if let Err(error) = Arguments::parse().and_then(run) {
        log::error!("{error}");
        std::process::exit(1);
    }
}
