# NES Emulator

A simple, fast NES emulator written in Rust. This project aims to emulate the Nintendo Entertainment System (NES) hardware and run NES games. It is a work in progress and is being developed with a focus on accuracy, performance, and maintainability.

## Features

- **CPU Emulation**: Accurate 6502 CPU emulation.
- **PPU (Picture Processing Unit)**: Basic graphical rendering of NES games.
- **APU (Audio Processing Unit)**: Sound emulation for NES games.
- **Input Handling**: Support for NES controllers (keyboard or joystick).
- **Save States**: Ability to save and load game states.
- **ROM Support**: Load and run NES game ROMs.
- **Cross-platform**: Works on major platforms, including Linux, Windows, and macOS.

## Getting Started

### Prerequisites

- [Rust](https://www.rust-lang.org/) (version 1.XX or higher)
- A compatible NES ROM file.

### Installation

1. Clone the repository:

   ```bash
   git clone https://github.com/yourusername/nes-emulator.git
   cd nes-emulator
   ```

2. Build the project using Cargo:

   ```bash
   cargo build --release
   ```

3. Run the emulator with a NES ROM:

   ```bash
   cargo run -- path/to/your/rom.nes
   ```

### Running Tests

To run the tests for the emulator:

```bash
cargo test
```

## Usage

- Use arrow keys for direction, `Z` for A button, and `X` for B button (customizable in settings).
- Save and load game states with `F5` (save) and `F7` (load).

## Contributing

We welcome contributions! Please feel free to open issues and pull requests. When contributing, be sure to follow these guidelines:

1. Fork the repository and clone your fork.
2. Create a new branch for your feature or bugfix.
3. Write tests for your code if applicable.
4. Submit a pull request describing the changes you’ve made.

### Code of Conduct

Please note that this project adheres to a [Code of Conduct](CODE_OF_CONDUCT.md). By participating, you are expected to honor this code.

## License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

## Acknowledgments

- Thanks to the Rust community for providing a great programming language.
- Special thanks to [NESDev](https://www.nesdev.org/) for resources on NES hardware.
- Any other libraries or contributions that were used or cited.

## Roadmap

- [ ] Complete APU sound emulation
- [ ] Improve PPU rendering accuracy
- [ ] Add support for save states
- [ ] Optimize performance
- [ ] Add GUI for configuration and ROM loading
