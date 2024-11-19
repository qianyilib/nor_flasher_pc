import serial
import struct
import time
import os
from enum import Enum
import crcmod.predefined
from tqdm import tqdm
import click
import sys
import atexit
import psutil  # Install first: pip install psutil
import signal
from serial.tools import list_ports

class PacketType:
    WRITE_FIRMWARE = 0x01
    READ_FIRMWARE = 0x02
    WRITE_DATA = 0x03
    READ_DATA = 0x04
    RESPONSE = 0x80
    MSG = 0x05
    ERASE_BLOCK = 0x06
    ERASE_ALL = 0x07
    TEST = 0x08
    RESET = 0x09
    
class Command:
    START = 0x01
    DATA = 0x02
    END = 0x03
    ERROR = 0xFF

class Response:
    OK = 0x00
    ERROR = 0x01

class NORFlashProgrammer:
    _instances = []  # Class variable to track all instances
    PACKET_SIZE = 1024  # Define packet size, must match the microcontroller's FIRMWARE_BUFFER_SIZE
    
    def __init__(self, port, baudrate=115200):
        self.port = port
        self.baudrate = baudrate
        self.serial = None
        try:
            self.open_serial()
            click.secho(f"Successfully opened port {port}", fg='green')
            NORFlashProgrammer._instances.append(self)  # Record instance
        except Exception as e:
            click.secho(f"Failed to open port: {str(e)}", fg='red')
            sys.exit(1)

    def open_serial(self):
        """Open serial connection"""
        try:
            if self.serial is None or not self.serial.is_open:
                self.serial = serial.Serial(
                    port=self.port,
                    baudrate=self.baudrate,
                    timeout=1,
                    write_timeout=1
                )
                if not self.serial.is_open:
                    self.serial.open()
        except Exception as e:
            raise Exception(f"Cannot open port {self.port}: {str(e)}")
        
    @classmethod
    def cleanup_all(cls):
        """Clean up all serial instances"""
        for instance in cls._instances:
            instance.close()
        cls._instances.clear()

    def close(self):
        """Safely close the serial port"""
        try:
            if self.serial and self.serial.is_open:
                self.serial.close()
                click.secho("Serial port safely closed", fg='green')
                if self in NORFlashProgrammer._instances:
                    NORFlashProgrammer._instances.remove(self)
        except Exception as e:
            click.secho(f"Error closing serial port: {str(e)}", fg='red')

    def calculate_crc16(self, data):
        crc = 0xFFFF
        
        for byte in data:
            crc ^= byte
            for _ in range(8):
                if crc & 0x0001:
                    crc = (crc >> 1) ^ 0xA001
                else:
                    crc = crc >> 1
        return crc

    def send_packet(self, packet_type, command, length, addr, data=None, wait_response=True):
        packet = struct.pack('<BBIII', 
                           packet_type,
                           command,
                           length,
                           addr,
                           0)
        
        crc = self.calculate_crc16(packet[:-4])
        packet = packet[:-4] + struct.pack('<H', crc)
        
        self.serial.write(packet)
        
        if data is not None:
            self.serial.write(data)
        if(wait_response):
            return self.wait_response()
        else:
            return True
            
    def read_from_serial(self, size, timeout=5):
        start_time = time.time()
        while (time.time() - start_time) < timeout:
            if self.serial.in_waiting >= size:
                return self.serial.read(size)
        return None
        
    def wait_response(self, timeout=5):
        response = self.read_from_serial(4, timeout)
        if(response == None):
            print(f"Response timeout {timeout}")
            return False
        else:
            packet_type, status, crc = struct.unpack('<BBH', response)
            return status == Response.OK
            
    def erase_chip(self):
        click.echo("Starting chip erase...")
        if not self.send_packet(PacketType.ERASE_ALL, Command.START, 0xFFFFFFFF, 0, None, False):
            click.secho("Failed to send erase command", fg='red')
            return False
    
        if not self.wait_response(60):
            click.secho("Erase failed", fg='red')
            return False
        click.secho("Erase completed", fg='green')
        return True
    
    def generate_sectors(self):
        sectors = []
        # First 8 sectors are 8KB
        for i in range(8):
            start_addr = i * 0x2000  # 8KB = 0x2000
            end_addr = start_addr + 0x1FFF  # End address
            sectors.append((start_addr, end_addr, 8))  # (Start address, End address, Size)

        # Remaining sectors are 64KB
        for i in range(8, 134):
            start_addr = 0x20000 + (i - 8) * 0x10000  # 64KB = 0x10000
            end_addr = start_addr + 0xFFFF  # End address
            sectors.append((start_addr, end_addr, 64))  # (Start address, End address, Size)

        return sectors

    def erase_block(self, base_addr, size):
        click.echo("Starting block erase...")
        
        sectors = self.generate_sectors()  # Generate sector list
        total_sectors = len(sectors)
        erased_sectors = 0

        with click.progressbar(length=total_sectors, label='Erase Progress') as bar:
            # Calculate sectors to erase
            for start, end, sector_size in sectors:
                if base_addr < end and base_addr + size > start:  # Check if within sector range
                    addr = max(base_addr, start)  # Determine start address for erase
                    erase_size = min(size, end - addr + 1)  # Determine size to erase
                    if not self.send_packet(PacketType.ERASE_BLOCK, Command.START, addr, erase_size):
                        click.secho("Erase failed", fg='red')
                        return False
                    
                    erased_sectors += 1
                    size -= erase_size  # Update remaining size to erase
                    bar.update(1)  # Update progress bar
                    if size <= 0:
                        break

        click.secho("All blocks erased successfully", fg='green')
        return True

    def test(self):
        click.echo("Starting write test...")
        if not self.send_packet(PacketType.TEST, Command.START, 0, 0, None, False):
            click.secho("Write test failed", fg='red')
            return False
        
        if not self.wait_response(600):
            click.secho("Write test failed", fg='red')
            return False
        click.secho("Write test completed", fg='green')
        return True
    
    def reset(self):
        click.echo("Restarting...")
        if not self.send_packet(PacketType.RESET, Command.START, 0, 0, None, False):
            click.secho("Failed to send restart command", fg='red')
            return False
        
        if not self.wait_response(600):
            click.secho("Restart failed", fg='red')
            return False
        click.secho("Restart completed", fg='green')
        return True
    
    def program_firmware(self, firmware_path, base_addr=0):
        """Write firmware to device"""
        try:
            with open(firmware_path, 'rb') as f:
                firmware_data = f.read()
        except Exception as e:
            click.secho(f"Failed to read firmware file: {str(e)}", fg='red')
            return False
        
        firmware_size = len(firmware_data)
        click.echo(f"Firmware size: {firmware_size:,} bytes")

        # Clear serial buffer
        self.serial.reset_input_buffer()
        self.serial.reset_output_buffer()

        # Send start command for writing firmware
        retry_count = 3
        while retry_count > 0:
            if self.send_packet(PacketType.WRITE_FIRMWARE, Command.START, 
                            firmware_size, base_addr):
                break
            retry_count -= 1
            click.secho(f"Failed to send write command, remaining retries: {retry_count}", fg='yellow')
            time.sleep(1)
        
        if retry_count == 0:
            click.secho("Failed to send write command, exceeded max retries", fg='red')
            return False

        # Send firmware data in chunks
        with tqdm(total=firmware_size, unit='B', unit_scale=True) as pbar:
            remaining_bytes = firmware_size
            current_addr = base_addr
            retry_count = 3
            
            while remaining_bytes > 0:
                try:
                    # Calculate current chunk size
                    block_size = min(self.PACKET_SIZE, remaining_bytes)
                    chunk = firmware_data[current_addr:current_addr + block_size]
                    
                    # Validate data chunk
                    if len(chunk) != block_size:
                        click.secho(f"Data chunk size error: expected {block_size}, actual {len(chunk)}", fg='red')
                        return False
                    
                    bytes_written = self.serial.write(chunk)
                    
                    if bytes_written != len(chunk):
                        click.secho(f"Data write incomplete: expected {len(chunk)}, actual {bytes_written}", fg='red')
                        return False
                    
                    # Wait for data to be sent
                    self.serial.flush()
                    
                    response_success = False
                    while retry_count > 0:
                        if self.wait_response(timeout=2.0):  # Increase timeout
                            response_success = True
                            break
                        retry_count -= 1
                        click.secho(f"Response timeout, remaining retries: {retry_count}", fg='yellow')
                        # Clear buffer before retry
                        self.serial.reset_input_buffer()
                        self.serial.reset_output_buffer()
                        time.sleep(0.5)  # Add retry delay
                    
                    if not response_success:
                        click.secho(f"Failed to write data chunk at addr 0x{current_addr:08X}, exceeded max retries", fg='red')
                        return False
                    
                    # Update progress
                    remaining_bytes -= block_size
                    current_addr += block_size
                    pbar.update(block_size)
                    
                except serial.SerialException as e:
                    click.secho(f"Serial communication error: {str(e)}", fg='red')
                    return False
                except Exception as e:
                    click.secho(f"Unknown error: {str(e)}", fg='red')
                    return False

        # Verify final status
        if remaining_bytes > 0:
            click.secho(f"Firmware write incomplete, remaining {remaining_bytes} bytes not written", fg='red')
            return False

        click.secho("Firmware write completed", fg='green')
        return True

    def read_firmware(self, save_path, size, base_addr=0):
        """Read firmware data"""
        click.echo(f"Starting to read firmware ({size:,} bytes)")

        # Send start command for reading firmware
        if not self.send_packet(PacketType.READ_FIRMWARE, Command.START, size, base_addr):
            click.secho("Failed to send read command", fg='red')
            return False

        try:
            with open(save_path, 'wb') as f:
                remaining_bytes = size
                current_addr = base_addr
                
                with tqdm(total=size, unit='B', unit_scale=True) as pbar:
                    while remaining_bytes > 0:
                        # Wait to receive data packet header
                        header = self.read_from_serial(10)  # UartPacket size without CRC
                        if len(header) != 10:
                            click.secho(f"Failed to read data packet header at addr 0x{current_addr:08X}", fg='red')
                            return False
                        
                        # Parse data packet header
                        packet_type, command, length, addr = struct.unpack('<BBII', header)
                        
                        # Read CRC
                        crc_bytes = self.read_from_serial(2)
                        if len(crc_bytes) != 2:
                            click.secho("Failed to read CRC", fg='red')
                            return False
                        
                        # Read actual data
                        data = self.read_from_serial(length)
                        if len(data) != length:
                            data_len = len(data)
                            click.secho(f"Failed to read data at addr 0x{current_addr:08X} packet_type {packet_type:02X} command:{command:02X} length:{length:04X} read_len:{data_len:04X}", fg='red')
                            return False
                        
                        # Write to file
                        f.write(data)
                        
                        # Send confirmation response
                        response_packet = struct.pack('<BBH', 
                                                PacketType.RESPONSE, 
                                                Response.OK,
                                                0)  # CRC will be added by send_packet
                        self.serial.write(response_packet)
                        self.serial.flush() 
                        
                        # Update progress
                        remaining_bytes -= length
                        current_addr += length
                        pbar.update(length)

        except Exception as e:
            click.secho(f"Failed to save firmware: {str(e)}", fg='red')
            return False

        click.secho(f"Firmware saved to: {save_path}", fg='green')
        return True

    def verify_firmware(self, firmware_path, base_addr=0):
        try:
            with open(firmware_path, 'rb') as f:
                firmware_data = f.read()
        except Exception as e:
            click.secho(f"Failed to read firmware file: {str(e)}", fg='red')
            return False
        
        firmware_size = len(firmware_data)
        click.echo(f"Starting firmware verification ({firmware_size:,} bytes)")

        if not self.send_packet(PacketType.READ_FIRMWARE, Command.START,
                              firmware_size, base_addr):
            click.secho("Failed to send verification command", fg='red')
            return False

        read_data = bytearray()
        with tqdm(total=firmware_size, unit='B', unit_scale=True) as pbar:
            for offset in range(0, firmware_size, self.PACKET_SIZE):
                chunk_size = min(self.PACKET_SIZE, firmware_size - offset)
                response = self.read_from_serial(chunk_size)
                
                if len(response) != chunk_size:
                    click.secho(f"Failed to read data at offset {offset}", fg='red')
                    return False
                
                read_data.extend(response)
                pbar.update(len(response))

        if read_data == firmware_data:
            click.secho("Firmware verification successful", fg='green')
            return True
        else:
            click.secho("Firmware verification failed", fg='red')
            return False

    def close(self):
        self.serial.close()

def cleanup_ports():
    """Cleanup function on program exit"""
    NORFlashProgrammer.cleanup_all()
    # Force cleanup of potentially occupied serial ports
    if sys.platform.startswith('win'):
        try:
            # Get all serial ports opened by the current process
            process = psutil.Process()
            for handler in process.open_files():
                if 'COM' in handler.path:
                    try:
                        os.system(f'mode {handler.path} BAUD=115200 PARITY=n DATA=8 STOP=1')
                    except:
                        pass
        except:
            pass

def force_release_port(port):
    """Force release occupied serial port"""
    if sys.platform.startswith('win'):
        try:
            # Check if other processes occupy the serial port
            for proc in psutil.process_iter(['pid', 'name', 'open_files']):
                try:
                    for file in proc.open_files():
                        if port in file.path:
                            click.secho(f"Port occupied by process {proc.name()}(PID:{proc.pid}), attempting to release...", fg='yellow')
                            proc.kill()
                            time.sleep(1)  # Wait for process to end
                except:
                    continue
            
            # Reset serial port
            os.system(f'mode {port} BAUD=115200 PARITY=n DATA=8 STOP=1')
        except Exception as e:
            click.secho(f"Failed to release port: {str(e)}", fg='red')
    else:
        try:
            os.system(f'fuser -k {port}')
        except:
            pass

# Register exit handler
atexit.register(cleanup_ports)

def list_com_ports():
    """List all available serial ports"""
    ports = list(serial.tools.list_ports.comports())
    if not ports:
        click.secho("No available serial ports found", fg='yellow')
        return None
    
    click.echo("Available serial ports:")
    for i, port in enumerate(ports):
        click.echo(f"{i + 1}. {port.device} - {port.description}")
    
    choice = click.prompt("Please select port number", type=int, default=1)
    if 1 <= choice <= len(ports):
        return ports[choice - 1].device
    return None

def show_menu():
    click.clear()
    click.secho("NOR Flash Programming Tool", fg='blue', bold=True)
    click.echo("=" * 50)
    click.echo("1. Write Firmware")
    click.echo("2. Read Firmware")
    click.echo("3. Verify Firmware")
    click.echo("4. Erase Chip")
    click.echo("5. Erase Block")
    click.echo("6. Test Write")
    click.echo("7. Restart Device")
    click.echo("8. Exit Program")
    click.echo("=" * 50)
    return click.prompt("Please select an operation", type=int, default=1)

def signal_handler(signum, frame):
    """Handle program termination signal"""
    click.secho("\nReceived termination signal, exiting safely...", fg='yellow')
    cleanup_ports()  # Clean up all serial ports
    sys.exit(0)

def main():
    # Existing signal handling remains unchanged
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)

    try:
        port = list_com_ports()
        if not port:
            return

        programmer = NORFlashProgrammer(port, 921600)

        while True:
            choice = show_menu()
            
            if choice == 1:
                firmware_path = click.prompt("Please enter firmware file path", type=str)
                if not os.path.exists(firmware_path):
                    click.secho("File does not exist!", fg='red')
                    continue
                base_addr = click.prompt("Please enter start address (hex)", type=str, default="0x00000000")
                programmer.program_firmware(firmware_path, int(base_addr, 16))
                
            elif choice == 2:
                save_path = click.prompt("Please enter save file path", type=str)
                size = click.prompt("Please enter read size (hex)", type=str, default="0x100000")
                base_addr = click.prompt("Please enter start address (hex)", type=str, default="0x00000000")
                programmer.read_firmware(save_path, int(size, 16), int(base_addr, 16))
                
            elif choice == 3:
                firmware_path = click.prompt("Please enter firmware file path", type=str)
                if not os.path.exists(firmware_path):
                    click.secho("File does not exist!", fg='red')
                    continue
                base_addr = click.prompt("Please enter start address (hex)", type=str, default="0x00000000")
                programmer.verify_firmware(firmware_path, int(base_addr, 16))
                
            elif choice == 4:
                if click.confirm("Are you sure you want to erase the entire chip?"):
                    programmer.erase_chip()
            elif choice == 5:
                base_addr = click.prompt("Please enter start address (hex)", type=str, default="0x00000000")
                size = click.prompt("Please enter size to erase", type=str, default="0x1000")
                programmer.erase_block(int(base_addr, 16), int(size, 16))
            elif choice == 6:
                programmer.test()
            elif choice == 7:
                programmer.reset()   
            elif choice == 8:
                break    
            click.prompt("Press Enter to continue...", default=True, show_default=False)

        programmer.close()
        click.secho("Program exited", fg='blue')
    except Exception as e:
        click.secho(f"Program exception: {str(e)}", fg='red')
    finally:
        if 'programmer' in locals():
            programmer.close()
        click.secho("Program exited", fg='blue')

if __name__ == "__main__":
    main()