use crate::config::Config;
use crate::transmitter::Transmitter;

pub struct DummyTransmitter;

impl DummyTransmitter {
    pub fn new(_: &Config) -> DummyTransmitter {
        warn!("Using dummy transmitter");
        DummyTransmitter {}
    }
}

impl Transmitter for DummyTransmitter {
    fn send(&mut self, gen: &mut dyn Iterator<Item = u32>) {
        for word in gen {
            info!("{:032b}", word);
        }
    }
}
