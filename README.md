# Demos for the esb crate

## Description

Choose the microcontroller with one of the following features:
- 51
- 52810
- 52832
- 52840

This crate also has a `fast-ru` feature that can be enable to use a faster ramp-up time, this feature is not available on nRF51 devices.

Edit the `.cargo/config` files of each example with your microcontroller target and gdb command. Also, if using `cargo-embed`, change the `chip` and `protocol` fields in Embed.toml.

These demos use the [rtt-target](https://crates.io/crates/rtt-target) crate and serial for communication.

If using `cargo-embed`, just run

```console
$ cargo embed --release --features=52832
```

Replace `52832` with the correct feature for your microcontroller.

## License

Licensed under either of

- Apache License, Version 2.0 ([LICENSE-APACHE](LICENSE-APACHE) or
  http://www.apache.org/licenses/LICENSE-2.0)
- MIT license ([LICENSE-MIT](LICENSE-MIT) or http://opensource.org/licenses/MIT)

at your option.

## Contribution

Unless you explicitly state otherwise, any contribution intentionally submitted
for inclusion in the work by you, as defined in the Apache-2.0 license, shall be
dual licensed as above, without any additional terms or conditions.
