#!/bin/sh
# Make docs/screenshot.png: a triggered simulator capture with the decoder setup and the decode table.
# Needs a display; run from the repository root after `cargo build --release`.
exec target/release/blinkfeuer --demo --rate 20000000 --samples 600000 \
  --trigger-mode uart --pre-ms 5 --post-ms 25 \
  --decode uart,spi,i2c --table --table-lane 1 --search sim --decoder-setup 0 \
  --window 1800x1100 --decoder-setup-rect 700,150,1080,660 \
  --markers 119791,125000 --zoom-start 117749 --zoom-spp 30 \
  --screenshot docs/screenshot.png --after 3
