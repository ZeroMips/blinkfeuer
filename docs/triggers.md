# Triggers and patterns

Triggers are done on the host: the analyzers have none in hardware. A trigger is either level and edge
conditions on any channels, or a protocol trigger with a pattern. The same pattern language is used by the
decode table search.

## Pattern language

- UART text or bytes, SPI words, or I2C events (`S`, `Sr`, `P`, `@50W` addresses, data bytes, `!` for NAK).
- `X` for free nibbles or bits (`4X`, `XX`, `0b1010XXXX`) and `*` for anything in between, e.g.
  `S @50W 01 XX P`.
- Optionally the trigger also fires on the protocol's error: UART framing/parity error, incomplete SPI word,
  I2C NAK.
- With Modbus RTU in the UART settings the pattern applies to whole frames from their first byte
  (`65 06 01 01`), or to frames with a bad CRC.

## Settings and timing

A protocol trigger can take its decoder settings from a decoder ("Settings from"), so they are set only once.
The trigger window shows an example of what it looks for, with the trigger point marked. The trigger point is
the start or the end of the match.

Pre-trigger and post-trigger are set separately in samples or time. Times can be shown relative to the
trigger. A triggered capture stops instead of dropping data when memory is full.

## Search

The decode table search takes text or a pattern in the language above: on characters, words or I2C events it
lists every item of a match, telegrams and Modbus frames are listed when they contain one.
