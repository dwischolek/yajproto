# YAJ-Proto - (WIP)

YAJ-Proto ("Yet Another Joystick Protocol") is a small controller protocol
created for an Arduino game-controller project. It is loosely based on the
IBus protocol. The arduino project will be released in a separate GitHub
repository.

`yajproto` is the host side library for YAJ-Proto. It provides a parser and a 
Linux serial-to-uinput example.

The library currently supports version 1 channel-data frames. It validates the
frame length, protocol version, payload ID, channel payload size, and checksum
before returning typed digital and analog channel values.

## Protocol

YAJ-Proto defines the decoded frame format below; it does not prescribe a
transport encoding or frame delimiter. A transport may, for example,
COBS-encode each frame and append a `0x00` delimiter. The included Linux
serial bridge uses that convention, then decodes COBS, and removes the
delimiter before passing the frame bytes to `YajProtoChannelDataFrame::try_from`.

### Frame structure

A decoded frame consists of a three-byte header followed by a payload selected
by `payload_id`. All multi-byte values are little-endian.

| Offset | Size   | Field        | Description                                         |
| ---    | ---:   | ---          | ---                                                 |
| 0      | 1 byte | `length`     | Total decoded frame length, including the checksum. |
| 1      | 1 byte | `version`    | Protocol version; currently `0x01`.                 |
| 2      | 1 byte | `payload_id` | Identifies the format of the payload that follows.  |

### Channel-data payload (`payload_id = 0x01`)

Version 1 currently defines payload `0x01`, whose payload contains controller
channel data. The following offsets are relative to the first byte after the
header:

| Offset   |                            Size | Field           | Description                                                                                                       |
| ---      | ---:                            | ---             | ---                                                                                                               |
| 0        |                          1 byte | `digital_count` | Number of digital channels.                                                                                       |
| 1        |                          1 byte | `analog_count`  | Number of analog channels.                                                                                        |
| 2        | `ceil(digital_count / 8)` bytes | Digital data    | Packed channel states, least-significant bit first. Bit 0 is channel 0. Unused bits in the last byte are ignored. |
| variable |        `analog_count * 2` bytes | Analog data     | One unsigned 16-bit value per channel, in channel order.                                                          |
| final 2  |                         2 bytes | `checksum`      | Checksum described below.                                                                                         |

The expected total decoded frame length is:

```text
7 + ceil(digital_count / 8) + (2 * analog_count)
```

The checksum covers every byte before the checksum field. Start with `0xffff`
and subtract each covered byte in order using wrapping 16-bit arithmetic. Store
the result as a little-endian `u16`.

For example, a frame with three digital channels (`true`, `false`, `true`) and
two analog channels (`0x1234`, `0xabcd`) has these decoded frame bytes before
its checksum:

```text
0c 01 01 03 02 05 34 12 cd ab
```

Its full frame is 12 bytes long; the two checksum bytes are appended after
`ab`.

## Build

Install the current stable Rust toolchain with Cargo, then run:

```sh
cargo build --release
cargo test
```

To build and run the Linux serial bridge:

```sh
cargo run --release --example yajproto_linux -- \
  --port-name /dev/ttyACM0 \
  --device-name YAJ-Proto
```

Both options are optional. The defaults are `/dev/ttyACM0` and `YAJ-Proto`.
The example opens the serial port at 115200 baud and requires Linux `uinput`
access. Ensure the `uinput` module is available and run with permissions to
open the serial device and create a virtual input device.

## Linux input mapping

The bridge creates its virtual controller from the first valid frame and
requires the channel counts to remain unchanged for the session.

Digital channels are assigned in order to `BTN_TRIGGER_HAPPY1` through
`BTN_TRIGGER_HAPPY40`, followed by `BTN_0` and `BTN_1`. At least two digital
channels are required, and the bridge supports at most 42. Analog channels are
assigned in order to `ABS_X`, `ABS_Y`, `ABS_Z`, `ABS_RX`, and `ABS_RY`, for a
maximum of five axes. The bridge emits events only when a channel value
changes.

## Acknowledgements

The project was, besides others, inspired by [vJoySerialFeeder](https://github.com/Cleric-K/vJoySerialFeeder).

## License

This project is licensed under either of [Apache License, Version
2.0](LICENSE-APACHE) or [MIT license](LICENSE-MIT), at your option.

### Contribution

Unless you explicitly state otherwise, any contribution intentionally
submitted for inclusion in the work by you, as defined in the Apache-2.0
license, shall be dual licensed as above, without any additional terms or
conditions.

