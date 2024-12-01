pub struct CPU {
    pub register_a: u8,
    pub register_x: u8,
    pub status: u8,
    pub program_counter: u16
}

impl CPU {
    pub fn new() -> CPU {
        CPU {
            register_a: 0,
            register_x: 0,
            status: 0,
            program_counter: 0
        }
    }

    pub fn interpret(&mut self, program: Vec<u8>) {
        self.program_counter = 0;

        loop {
            let opcode = program[self.program_counter as usize];
            self.program_counter += 1;

            match opcode {
                0x00 => return,
                // LDA: Load Accumulator
                0xA9 => {
                    let param = program[self.program_counter as usize];
                    self.program_counter += 1;

                    self.lda(param);
                },
                // TAX: Transfer Accumulator to X
                0xAA => self.tax(),
                // INX: Increment X
                0xE8 => self.inx(),
                _ => {}
            }
        }
    }

    fn lda(&mut self, param: u8) {
        self.register_a = param;
        self.update_zero_and_negative_flags(self.register_a);
    }

    fn tax(&mut self) {
        self.register_x = self.register_a;
        self.update_zero_and_negative_flags(self.register_x);
    }

    fn inx(&mut self) {
        self.register_x = self.register_x.wrapping_add(1);
        self.update_zero_and_negative_flags(self.register_x);
    }

    fn update_zero_and_negative_flags(&mut self, rst: u8) {
        if rst == 0 {
            self.status |= 0b0000_0100;
        } else {
            self.status &= 0b1111_1011;
        }

        if rst & 0b1000_0000 != 0 {
            self.status |= 0b0000_0010;
        } else {
            self.status &= 0b1111_1101;
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn test_0xa9_lda_immediate_load_data() {
        let mut cpu = CPU::new();
        cpu.interpret(vec![0xa9, 0x05, 0x00]);
        assert_eq!(cpu.register_a, 0x05);
        assert!((cpu.status & 0b0000_0010) == 0b00);
        assert!((cpu.status & 0b0000_0100) == 0);
    }

    #[test]
    fn test_0xaa_tax_move_a_to_x() {
        let mut cpu = CPU::new();
        cpu.register_a = 10;
        cpu.interpret(vec![0xaa, 0x00]);

        assert_eq!(cpu.register_x, 10);
    }

    #[test]
    fn test_0xe8_inx_increment_x() {
        let mut cpu = CPU::new();
        cpu.register_x = 10;
        cpu.interpret(vec![0xe8, 0x00]);
        assert_eq!(cpu.register_x, 11);
    }

    #[test]
    fn test_0xe8_inx_overflow() {
        let mut cpu = CPU::new();
        cpu.register_x = 0xff;
        cpu.interpret(vec![0xe8, 0xe8, 0x00]);

        assert_eq!(cpu.register_x, 1);
    }

    #[test]
    fn test_0xa9_lda_zero_flag() {
        let mut cpu = CPU::new();
        cpu.interpret(vec![0xa9, 0x00, 0x00]);
        assert!((cpu.status & 0b0000_0010) == 0b00);
    }

    #[test]
    fn test_0xa9_lda_negative_flag() {
        let mut cpu = CPU::new();
        cpu.interpret(vec![0xa9, 0x80, 0x00]);
        assert!((cpu.status & 0b0000_0010) == 0b10);
    }

    #[test]
    fn test_ops_work_together() {
        let mut cpu = CPU::new();
        cpu.interpret(vec![0xa9, 0xc0, 0xaa, 0xe8, 0x00]);
        assert_eq!(cpu.register_x, 0xc1);
    }
}