use rustic::bus::Bus;
use expectest::prelude::*;

#[test]
fn write_and_read() {
    let mut bus = Bus::new();
    bus.write(0x0000, 0xAD);

    let byte = bus.read(0x0000);
    expect!(byte).to(be_eq(0xAD));
}