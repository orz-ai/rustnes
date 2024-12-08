use sdl2::pixels::Palette;
use crate::cartridge::Mirroring;
use crate::ppu::registers::addr::AddrRegister;
use crate::ppu::registers::control::ControlRegister;
use crate::ppu::registers::mask::MaskRegister;
use crate::ppu::registers::scroll::ScrollRegister;
use crate::ppu::registers::status::StatusRegister;

pub mod registers;



pub struct NesPPU {
    pub chr_rom: Vec<u8>,
    pub mirroring: Mirroring,
    pub ctrl: ControlRegister,
    pub mask: MaskRegister,
    pub status: StatusRegister,
    pub scroll: ScrollRegister,
    pub addr: AddrRegister,
    pub vram: [u8; 2048],

    pub oam_addr: u8,
    pub oam_data: [u8; 256],
    pub palette_table: [u8; 32],

    internal_data_buf: u8
}

pub trait PPU {
    fn write_to_ctrl(&mut self, value: u8);
    fn write_to_mask(&mut self, value: u8);
    fn read_status(&mut self) -> u8;

    fn write_to_oam_addr(&mut self, value: u8);

    fn write_to_oam_data(&mut self, value: u8);

    fn read_oam_data(&mut self) -> u8;

    fn write_to_scroll(&mut self, value: u8);

    fn write_to_ppu_addr(&mut self, value: u8);

    fn write_to_data(&mut self, value: u8);

    fn read_data(&mut self) -> u8;

    fn write_oam_dma(&mut self, value: &[u8; 256]);
}

impl NesPPU {
    pub fn new_empty_rom() -> Self {
        NesPPU::new(vec![0; 2048], Mirroring::Horizontal)
    }

    pub fn new(chr_rom: Vec<u8>, mirroring: Mirroring) -> Self {
        NesPPU {
            chr_rom,
            mirroring,
            ctrl: ControlRegister::new(),
            mask: MaskRegister::new(),
            status: StatusRegister::new(),
            oam_addr: 0,
            oam_data: [0; 64 * 4],
            palette_table: [0; 32],
            scroll: ScrollRegister::new(),
            addr: AddrRegister::new(),
            vram: [0; 2048],
            internal_data_buf: 0
        }
    }

    pub fn mirror_vram_addr(&self, addr: u16) -> u16 {
        let mirrored_vram = addr & 0b10111111111111;
        let vram_index = mirrored_vram - 0x2000;
        let name_table = vram_index / 0x400;

        match (&self.mirroring, name_table) {
            (Mirroring::Vertical, 2) | (Mirroring::Vertical, 3) => vram_index - 0x800,
            (Mirroring::Horizontal, 2) => vram_index - 0x400,
            (Mirroring::Horizontal, 1) => vram_index - 0x400,
            (Mirroring::Horizontal, 3) => vram_index - 0x800,
            _ => vram_index
        }
    }


    fn increment_vram_addr(&mut self) {
        self.addr.increment(self.ctrl.vram_addr_increment());
    }
}

impl PPU for NesPPU {
    fn write_to_ctrl(&mut self, value: u8) {
        let before_nmi_status = self.ctrl.generate_vblank_nmi();
        self.ctrl.update(value);
    }

    fn write_to_mask(&mut self, value: u8) {
        self.mask.update(value);
    }

    fn read_status(&mut self) -> u8 {
        let data = self.status.snapshot();
        self.status.reset_vblank_status();
        self.addr.reset_latch();
        self.scroll.reset_latch();

        data
    }

    fn write_to_oam_addr(&mut self, value: u8) {
        self.oam_addr = value;
    }

    fn write_to_oam_data(&mut self, value: u8) {
        self.oam_data[self.oam_addr as usize] = value;
        self.oam_addr = self.oam_addr.wrapping_add(1);
    }

    fn read_oam_data(&mut self) -> u8 {
        self.oam_data[self.oam_addr as usize]
    }

    fn write_to_scroll(&mut self, value: u8) {
        self.scroll.write(value);
    }

    fn write_to_ppu_addr(&mut self, value: u8) {
        self.addr.update(value);
    }

    fn write_to_data(&mut self, value: u8) {
        let addr = self.addr.get();
        match addr {
            0..=0x1fff => println!("attempt to write to chr rom space {}", addr),
            0x2000..=0x2fff => self.vram[self.mirror_vram_addr(addr) as usize] = value,
            0x3000..=0x3eff => {
                unimplemented!("addr {} shouldn't be used in reality", addr);
            }
            0x3f10 | 0x3f14 | 0x3f18 | 0x3f1c => {
                let addr_mirror = addr - 0x10;
                self.palette_table[(addr_mirror - 0x3f00) as usize] = value;
            }
            0x3f00..=0x3fff => {
                self.palette_table[(addr - 0x3f00) as usize] = value;
            }
            _ => println!("Invalid PPU address: {}", addr)
        }
    }

    fn read_data(&mut self) -> u8 {
        let addr = self.addr.get();
        self.increment_vram_addr();

        match addr {
            0..=0x1fff => {
                let result = self.internal_data_buf;
                self.internal_data_buf = self.chr_rom[addr as usize];
                result
            },
            0x2000..=0x2fff => {
                let result = self.internal_data_buf;
                self.internal_data_buf = self.vram[self.mirror_vram_addr(addr) as usize];
                result
            },
            0x3000..=0x3eff => {
                unimplemented!("addr {} shouldn't be used in reality", addr);
            }
            0x3f10 | 0x3f14 | 0x3f18 | 0x3f1c => {
                let addr_mirror = addr - 0x10;
                self.palette_table[(addr_mirror - 0x3f00) as usize]
            }
            0x3f00..=0x3fff => {
                self.palette_table[(addr - 0x3f00) as usize]
            }

            _ => panic!("unexpected access to mirrored space {}", addr)
        }
    }

    fn write_oam_dma(&mut self, data:&[u8; 256]) {
        for x in data.iter() {
            self.oam_data[self.oam_addr as usize] = *x;
            self.oam_addr = self.oam_addr.wrapping_add(1);
        }
    }
}