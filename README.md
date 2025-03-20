# paa5100je-pwm3901 Optical Flow Sensor Rust Driver

[![MegaLinter](https://github.com/dysonltd/paa5100je-pwm3901/actions/workflows/mega-linter.yaml/badge.svg)](https://github.com/dysonltd/paa5100je-pwm3901/actions/workflows/mega-linter.yaml) [![Continuous Build](https://github.com/dysonltd/paa5100je-pwm3901/actions/workflows/continuous-build.yaml/badge.svg)](https://github.com/dysonltd/paa5100je-pwm3901/actions/workflows/continuous-build.yaml)

## Summary

This is a platform agnostic Rust Driver for the paa5100je-pwm3901 Optical Flow Sensor by PixArt Imageing Inc. The driver is based on the [embedded-hal](https://docs.rs/embedded-hal/latest/embedded_hal/) traits. For more information it is recommended to look at the docs using `cargo doc`.

![Optical Flow Breakout Board](./docs/Optical%20Flow%20Breakout%20Board.png)

## Examples

For examples, there is an example for the [ESP32C6](https://unexpectedmaker.com/shop.html#!/TinyC6/p/602208790) which can be found [here](./examples/esp32c6/src/bin/async_main.rs).

## Useful Links

- [PiHut Sensor](https://thepihut.com/products/paa5100je-near-optical-flow-spi-breakout)
- [Python Library](https://github.com/pimoroni/pmw3901-python)
