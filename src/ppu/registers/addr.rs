
pub struct AddrRegister {
    value: (u8, u8),
    hi_ptr: bool,
}

impl AddrRegister {

    pub fn new() -> AddrRegister {
        AddrRegister {
            value: (0, 0),
            hi_ptr: true,
        }
    }

    fn set(&mut self, value: u16) {
        self.value.0 = (value >> 8) as u8;
        self.value.1 = (value & 0xff) as u8;
    }

    pub(crate) fn get(&self) -> u16 {
        (self.value.0 as u16) << 8 | self.value.1 as u16
    }

    pub fn update(&mut self, value: u8) {
        if self.hi_ptr {
            self.value.0 = value;
        } else {
            self.value.1 = value;
        }

        if self.get() > 0x3fff {
            self.set(self.get() & 0b11111111111111);
        }
    }

    pub fn increment(&mut self, inc: u8) {
        let low = self.value.1;
        self.value.1 = self.value.1.wrapping_add(inc);
        if low > self.value.1 {
            self.value.0 = self.value.0.wrapping_add(1);
        }

        if self.get() > 0x3fff {
            self.set(self.get() & 0b11111111111111);
        }
    }

    pub fn reset_latch(&mut self) {
        self.hi_ptr = true;
    }
}