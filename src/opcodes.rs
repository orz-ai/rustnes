use std::collections::HashMap;
use lazy_static::lazy_static;
use crate::cpu::AddressMode;

pub struct OpCode {
    pub code: u8,
    pub mnemonic: &'static str,
    pub len: u8,
    pub cycles: u8,
    pub mode: AddressMode
}

impl OpCode {
    pub fn new(code: u8, mnemonic: &'static str, len: u8, cycles: u8, mode: AddressMode) -> OpCode {
        OpCode { code, mnemonic, len, cycles, mode }
    }
}

lazy_static! {
    pub static ref CPU_OPS_CODES: Vec<OpCode> = vec![
        OpCode::new(0x00, "BRK", 1, 7, AddressMode::NoneAddressing),
        OpCode::new(0xAA, "TAX", 1, 2, AddressMode::NoneAddressing),
        OpCode::new(0xE8, "INX", 1, 2, AddressMode::NoneAddressing),
        OpCode::new(0xA9, "LDA", 2, 2, AddressMode::Immediate),
        OpCode::new(0xA2, "LDX", 2, 3, AddressMode::ZeroPage),
        OpCode::new(0xB5, "LDA", 2, 4, AddressMode::ZeroPage_X),
        OpCode::new(0xAD, "LDA", 3, 4, AddressMode::Absolute),
        OpCode::new(0xBD, "LDA", 3, 4, AddressMode::Absolute_X),
        OpCode::new(0xB9, "LDA", 3, 4, AddressMode::Absolute_Y),
        OpCode::new(0xA1, "LDA", 2, 6, AddressMode::Indirect_X),
        OpCode::new(0xB1, "LDA", 2, 5, AddressMode::Indirect_Y),

        OpCode::new(0x85, "STA", 2, 3, AddressMode::ZeroPage),
        OpCode::new(0x95, "STA", 2, 4, AddressMode::ZeroPage_X),
        OpCode::new(0x8D, "STA", 3, 4, AddressMode::Absolute),
        OpCode::new(0x9D, "STA", 3, 5, AddressMode::Absolute_X),
        OpCode::new(0x99, "STA", 3, 5, AddressMode::Absolute_Y),
        OpCode::new(0x81, "STA", 2, 6, AddressMode::Indirect_X),
        OpCode::new(0x91, "STA", 2, 6, AddressMode::Indirect_Y),
    ];

    pub static ref OPCODES_MAP: HashMap<u8, &'static OpCode> = {
        let mut map = HashMap::new();
        for op in &*CPU_OPS_CODES {
            map.insert(op.code, op);
        }
        map
    };
}

