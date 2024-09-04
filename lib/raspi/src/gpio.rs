use model::Model;
use std::ptr::{read_volatile, write_volatile};
use std::sync::Arc;

pub struct GpioBase(*mut u32);

pub enum Gpio {
    MemGpio {
        base: Arc<GpioBase>,
        pin_mapping: Option<Vec<usize>>,
    },
    SysFsGpio {
        pin_mapping: Option<Vec<usize>>,
    },
}

impl Gpio {
    pub fn new() -> Option<Gpio> {
        let model = Model::get();
        let base = model.gpio_base();

        if base.is_none() {
            return Some(Gpio::SysFsGpio {
                pin_mapping: model.pin_mapping(),
            });
        }

        let mapped_base = unsafe {
            let mem = "/dev/mem\0".as_ptr() as *const libc::c_char;
            let mem_fd = libc::open(mem, libc::O_RDWR | libc::O_SYNC);
            if mem_fd < 0 {
                return None;
            }

            let mapped_base = libc::mmap(
                0 as *mut libc::c_void,
                0x1000,
                libc::PROT_READ | libc::PROT_WRITE,
                libc::MAP_SHARED,
                mem_fd,
                base.unwrap() as libc::off_t,
            );

            libc::close(mem_fd);

            if mapped_base == libc::MAP_FAILED {
                return None;
            }

            mapped_base
        };

        Some(Gpio::MemGpio {
            base: Arc::new(GpioBase(mapped_base as *mut u32)),
            pin_mapping: model.pin_mapping(),
        })
    }

    pub fn pin(&self, number: usize, direction: Direction) -> Box<dyn Pin> {
        match self {
            &Gpio::MemGpio {
                ref base,
                ref pin_mapping,
            } => {
                let number = pin_mapping
                    .as_ref()
                    .and_then(|mapping| mapping.get(number).map(|num| *num))
                    .unwrap_or(number);
                Box::new(MemGpioPin::new(base.clone(), number, direction))
            }
            &Gpio::SysFsGpio { ref pin_mapping } => {
                let number = pin_mapping
                    .as_ref()
                    .and_then(|mapping| mapping.get(number).map(|num| *num))
                    .unwrap_or(number);
                Box::new(SysFsGpioPin::new(number as u8, direction))
            }
        }
    }
}

impl Drop for GpioBase {
    fn drop(&mut self) {
        unsafe { libc::munmap(self.0 as *mut libc::c_void, 0x1000) };
    }
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub enum Direction {
    Input,
    Output,
}

pub trait Pin {
    fn set_direction(&mut self, direction: Direction);
    fn set(&self, value: bool);
    fn read(&self) -> bool;

    fn set_high(&self) {
        self.set(true);
    }

    fn set_low(&self) {
        self.set(false);
    }
}

pub struct SysFsGpioPin {
    direction: Direction,
    pin: u8,
}

impl SysFsGpioPin {
    pub fn new(number: u8, direction: Direction) -> SysFsGpioPin {
        println!("Attempting to export GPIO pin {}", number);

        set_gpio_pin(number, direction);

        let pin = SysFsGpioPin {
            pin: number,
            direction: direction,
        };

        pin
    }
}

pub fn get_gpio_pin(number: u8) -> rppal::gpio::Pin {
    rppal::gpio::Gpio::new()
        .expect("Failed to initialize GPIO")
        .get(number as u8)
        .expect("Failed to get the pin")
}

pub fn set_gpio_pin(number: u8, direction: Direction) {
    let gpio_pin = get_gpio_pin(number);

    if matches!(direction, Direction::Input) {
        gpio_pin.into_input();
    } else {
        gpio_pin.into_output();
    }
}

impl Pin for SysFsGpioPin {
    fn set_direction(&mut self, direction: Direction) {
        set_gpio_pin(self.pin, direction);
    }

    fn set(&self, value: bool) {
        assert_eq!(self.direction, Direction::Output);

        let gpio_pin = get_gpio_pin(self.pin);

        if value {
            gpio_pin.into_output_high();
        } else {
            gpio_pin.into_output_low();
        }
    }

    fn read(&self) -> bool {
        assert_eq!(self.direction, Direction::Input);
        
        match get_gpio_pin(self.pin).read() {
            rppal::gpio::Level::High => true,
            rppal::gpio::Level::Low => false,
        }
    }
}

pub struct MemGpioPin {
    base: Arc<GpioBase>,
    number: usize,
    direction: Direction,
}

impl MemGpioPin {
    pub fn new(
        base: Arc<GpioBase>,
        number: usize,
        direction: Direction,
    ) -> MemGpioPin {
        let mut pin = MemGpioPin {
            base: base,
            number: number,
            direction: direction,
        };

        pin.set_direction(direction);
        pin
    }
}

impl Pin for MemGpioPin {
    fn set_direction(&mut self, direction: Direction) {
        self.direction = direction;
        let number = self.number;

        match self.direction {
            Direction::Input => unsafe {
                let p =
                    (*self.base).0.offset((number / 10) as isize) as *mut u32;
                *p &= !(0b111 << ((number % 10) * 3));
            },
            Direction::Output => unsafe {
                let p =
                    (*self.base).0.offset((number / 10) as isize) as *mut u32;
                *p &= !(0b111 << ((number % 10) * 3));
                *p |= 0b1 << ((number % 10) * 3);
            },
        };
    }

    fn set(&self, value: bool) {
        assert_eq!(self.direction, Direction::Output);
        if value {
            unsafe {
                let gpio_set = (*self.base).0.offset(7) as *mut u32;
                write_volatile(gpio_set, 1 << self.number);
            }
        } else {
            unsafe {
                let gpio_clr = (*self.base).0.offset(10) as *mut u32;
                write_volatile(gpio_clr, 1 << self.number);
            }
        }
    }

    fn read(&self) -> bool {
        assert_eq!(self.direction, Direction::Input);
        unsafe {
            let gpio_val = read_volatile((*self.base).0.offset(13) as *mut u32);
            (gpio_val & (1 << self.number)) != 0
        }
    }
}

impl Drop for MemGpioPin {
    fn drop(&mut self) {
        match self.direction {
            Direction::Output => {
                self.set_low();
                self.set_direction(Direction::Input);
            }
            Direction::Input => {
                // nothing to clean up
            }
        };
    }
}
