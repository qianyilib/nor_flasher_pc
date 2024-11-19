# NOR Flash Programmer PC

## Overview
The NOR Flash Programmer is a Python tool designed for programming NOR flash memory devices via serial communication. It allows users to write, read, verify, and erase firmware on compatible devices.

## Features
- Write firmware to the device
- Read firmware from the device
- Verify firmware integrity
- Erase the entire chip or specific blocks
- Test write functionality
- Restart the device

## Requirements
To run this tool, you need to have Python 3.x installed on your system. Additionally, you will need to install the following Python packages:

## Installation
You can install the required packages using `pip`. Run the following command in your terminal:
```bash
pip install pyserial tqdm click psutil crcmod
```

## Usage
1. Connect your NOR flash device to your computer via a serial port.
2. Ensure that the [NOR Flash STM32 Firmware](https://github.com/qianyilib/nor_flasher_stm32.git) is properly installed and running on your STM32F407 microcontroller.
3. Run the `norflasher.py` script:
   ```bash
   python norflasher.py
   ```
4. Follow the on-screen prompts to select the desired operation.

## Available Operations
1. Write Firmware
2. Read Firmware
3. Verify Firmware
4. Erase Chip
5. Erase Block
6. Test Write
7. Restart Device
8. Exit Program

## License
This project is licensed under the MIT License. See the LICENSE file for more details.

## Contributing
Contributions are welcome! Please feel free to submit a pull request or open an issue for any enhancements or bug fixes.

## Contact
If you have any questions, please open an issue on GitHub or contact [mutouman@gmail.com].