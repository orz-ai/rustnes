mod cpu;
mod opcodes;
mod bus;
mod cartridge;
mod trace;
mod ppu;
mod render;

#[macro_use]
extern crate lazy_static;

#[macro_use]
extern crate bitflags;

use rand::Rng;
use sdl2::event::Event;
use sdl2::EventPump;
use sdl2::keyboard::Keycode;
use sdl2::pixels::{Color, PixelFormatEnum};
use crate::bus::Bus;
use crate::cartridge::Rom;
use crate::cpu::{Mem, CPU};
use crate::ppu::NesPPU;
use crate::render::frame::Frame;
use crate::trace::trace;

fn color(byte: u8) -> Color {
    match byte {
        0 => Color::BLACK,
        1 => Color::WHITE,
        2 | 9 => Color::GREY,
        3 | 10 => Color::RED,
        4 | 11 => Color::GREEN,
        5 | 12 => Color::BLUE,
        6 | 13 => Color::MAGENTA,
        7 | 14 => Color::YELLOW,
        _ => Color::CYAN,
    }
}

fn read_screen_state(cpu: &mut CPU, frame: &mut [u8; 32 * 3 * 32]) -> bool {
    let mut frame_idx = 0;
    let mut update = false;
    for i in 0x0200..0x600 {
        let color_idx = cpu.mem_read(i as u16);
        let (r, g, b) = color(color_idx).rgb();

        if frame[frame_idx] != r || frame[frame_idx + 1] != g || frame[frame_idx + 2] != b {
            frame[frame_idx] = r;
            frame[frame_idx + 1] = g;
            frame[frame_idx + 2] = b;
            update = true
        }
        frame_idx += 3;
    }

    update
}

fn handle_user_input(cpu: &mut CPU, event_pump: &mut EventPump) {
    for event in event_pump.poll_iter() {
        match event {
            Event::Quit { .. } | Event::KeyDown { keycode: Some(sdl2::keyboard::Keycode::Escape), .. } => {
                std::process::exit(0)
            },

            Event::KeyDown { keycode: Some(sdl2::keyboard::Keycode::W), .. } => {
                cpu.mem_write(0xff, 0x77);
            },
            Event::KeyDown { keycode: Some(sdl2::keyboard::Keycode::S), .. } => {
                cpu.mem_write(0xff, 0x73);
            },
            Event::KeyDown { keycode: Some(sdl2::keyboard::Keycode::A), .. } => {
                cpu.mem_write(0xff, 0x61);
            },
            Event::KeyDown { keycode: Some(sdl2::keyboard::Keycode::D), .. } => {
                cpu.mem_write(0xff, 0x64);
            }

            _ => {
                // do nothing
            }
        }
    }
}

fn main() {
    // init sdl2
    let sdl_context = sdl2::init().unwrap();
    let video_subsystem = sdl_context.video().unwrap();
    let window = video_subsystem
        .window("Tile viewer", (256.0 * 3.0) as u32, (240.0 * 3.0) as u32)
        .position_centered()
        .build()
        .unwrap();

    let mut canvas = window.into_canvas().present_vsync().build().unwrap();
    let mut event_pump = sdl_context.event_pump().unwrap();
    canvas.set_scale(10.0, 10.0).unwrap();

    let creator = canvas.texture_creator();
    let mut texture = creator
        .create_texture_target(PixelFormatEnum::RGB24, 256, 240).unwrap();


    // load the game
    let bytes: Vec<u8> = std::fs::read("pacman.nes").unwrap();
    let rom = Rom::new(&bytes).unwrap();

    let mut frame = Frame::new();

    let bus = Bus::new(rom, move |ppu: &NesPPU| {
        render::render(ppu, &mut frame);
        texture.update(None, &frame.data, 256 * 3).unwrap();

        canvas.copy(&texture, None, None).unwrap();
        canvas.present();

        for event in event_pump.poll_iter() {
            match event {
                Event::Quit {..} | Event::KeyDown {
                    keycode: Some(Keycode::Escape),
                    ..
                } => std::process::exit(0),

                _ => {
                    // do nothing
                }
            }
        }

    });

    let mut cpu = CPU::new(bus);
    cpu.reset();
    cpu.run();
}
