#!/usr/bin/env python3
import hid_test
import time
import numpy
from subprocess import run
import usb.core
import uuid
import encode_usb_strings
import serial
import serial.tools.list_ports;
import shutil

# Locations for external utilities and files referenced by the test program
file_locations = {
    'iceprog':'tillitis-iceprog',
    'chprog':'chprog',
    'app_gateware':'binaries/top.bin',
    'ch552_firmware':'binaries/usb_device_cdc.bin',
    'ch552_firmware_injected':'/tmp/ch552_fw_injected.bin',
    'pico_bootloader_source':'binaries/main.uf2',
    'pico_bootloader_target':'/media/lab/RPI-RP2/main.uf2'
}

def enable_power():
    """Enable power to the TK-1"""
    d = hid_test.ice40_flasher()
    d.gpio_set_direction(7, True)
    d.gpio_put(7, True)
    d.close()
    time.sleep(.2)

    return True

def disable_power():
    """Disable power to the TK-1"""
    time.sleep(.1)
    d = hid_test.ice40_flasher()
    d.gpio_set_direction(7, True)
    d.gpio_put(7, False)
    d.close()

    return True

def measure_voltages(device, samples):
    """Measure the voltage levels of the three mta1 power rails multiple times, and return the average values"""
    adc_vals = numpy.array([0,0,0])
    for i in range(0,samples):
        adc_vals = adc_vals + device.adc_read_all()
    adc_vals = dict(zip(['1.2','2.5','3.3'],adc_vals/samples))
    return adc_vals

def voltage_test():
    """Measure 3.3V 2.5V, and 1.2V voltage rails on the TK-1"""
    enable_power()

    d = hid_test.ice40_flasher()
    vals = measure_voltages(d,20)

    d.close()
    disable_power()

    print('voltages:',', '.join('{:}V:{:.3f}'.format(val[0],val[1]) for val in vals.items()))
    if (
        (abs(vals['1.2'] - 1.2) > .2)
        | (abs(vals['2.5'] - 2.5) > .2)
        | (abs(vals['3.3'] - 3.3) > .2)
        ):
        return False

    return True

def flash_validate_id():
    """Read the ID from TK-1 SPI flash, and verify that it matches the expected value"""
    result = run([
        file_locations['iceprog'],
        '-t'
        ],
        capture_output=True)

    err = result.stderr.split(b'\n')
    for line in err:
        if line.startswith(b'flash ID:'):
            vals_b = line.split(b' 0x')[1:]
            flash_id = int(b''.join(vals_b),16)
            print(line, hex(flash_id))
 
            # Note: Flash IDs reported by iceprog are slightly wrong
            flash_types = {
                0xb40140b40140b40140b40140b4014: 'XT25F08BDFIGT-S (MTA1-USB-V1)',
                0xef401400: 'W25Q80DVUXIE (TP-1)'
            }

            flash_type = flash_types.get(flash_id)

            if flash_type == None:
                print('Flash ID invalid')
                return False
            print('Detected flash type: {:}'.format(flash_type))
            return True
            
    return (result.returncode == 0)

def flash_program():
    """Program and verify the TK-1 SPI flash with the application test gateware"""
    result = run([
        file_locations['iceprog'],
        file_locations['app_gateware']
        ])
    print(result)

    return (result.returncode == 0)

def flash_check():
    """Verify the TK-1 SPI flash is programmed with the application test gateware"""
    result = run([
        file_locations['iceprog'],
        '-c',
        file_locations['app_gateware']
        ])
    print(result)

    return (result.returncode == 0)

def test_extra_io():
    """Test the TK-1 RTS, CTS, and GPIO1-4 lines by measuring a test pattern generated by the app_test gateware"""
    enable_power()

    time.sleep(.1)
    d = hid_test.ice40_flasher()

    d.gpio_put(16, False)
    d.gpio_set_direction(16, True)

    expected_results = [1<<(i%5) for i in range(9,-1,-1)]

    results = []
    for i in range(0,10):
        vals = d.gpio_get_all()
        pattern = (vals >> 17) & 0b11111
        results.append(pattern)

        d.gpio_put(16, True)
        d.gpio_put(16, False)

    d.gpio_set_direction(16, False)
    d.close()

    disable_power()

    print(results,expected_results,results == expected_results)
    return results == expected_results


def test_found_bootloader():
    """Search for a CH552 in USB bootloader mode"""
    print('\n\n\nSearching for CH552 bootloader, plug in USB cable now (times out in 10 seconds)!')
    for trys in range(0,100): # retry every 0.1s, up to 10 seconds
        devices= usb.core.find(idVendor=0x4348, idProduct=0x55e0, find_all=True)
        count = len(list(devices))

        if count == 1:
            return True

        time.sleep(0.1)

    post = usb.core.find(idVendor=0x4348, idProduct=0x55e0, find_all=True)
    post_count = len(list(post))
    return (post_count == 1)

def inject_serial_number(infile, outfile, serial):
    """Inject a serial number into the specified CH552 firmware file"""
    magic = encode_usb_strings.string_to_descriptor("68de5d27-e223-4874-bc76-a54d6e84068f")
    replacement = encode_usb_strings.string_to_descriptor(serial)

    f = bytearray(open(infile, 'rb').read())
    
    pos = f.find(magic)
    
    if pos < 0:
        print('failed to find magic string')
        exit(1)
    
    f[pos:(pos+len(magic))] = replacement
    
    with open(outfile, 'wb') as of:
        of.write(f)
    
def flash_ch552(serial):
    """Flash an attached CH552 device with the USB CDC firmware, injected with the given serial number"""

    print(serial)
    inject_serial_number(
        file_locations['ch552_firmware'],
        file_locations['ch552_firmware_injected'],
        serial)
    
    # Program the CH552 using CHPROG
    result = run([
        file_locations['chprog'],
        file_locations['ch552_firmware_injected']
        ])
    print(result)
    return (result.returncode == 0)

def find_serial_device(desc):
    """Look for a serial device that has the given attributes"""

    for port in serial.tools.list_ports.comports():
        matched = True
        for key, value in desc.items():
            if not getattr(port, key) == value:
                matched = False

        if matched:
            print(port.device)
            return port.device

    return None

def find_ch552(serial):
    """Search all serial devices for one that has the correct description and serial number"""
    time.sleep(1)

    description = {
        'vid':0x1207,
        'pid':0x8887,
        'manufacturer':'Tillitis',
        'product':'MTA1-USB-V1',
        'serial_number':serial
    }
    
    if find_serial_device(description) == None:
        return False

    return True

def ch552_program():
    """Load the CDC ACM firmware onto a CH552 with a randomly generated serial number, and verify that it boots correctly"""
    if not test_found_bootloader():
        print('Error finding CH552!')
        return False
    
    serial = str(uuid.uuid4())
    
    if not flash_ch552(serial):
        print('Error flashing CH552!')
        return False
    
    if not find_ch552(serial):
        print('Error finding flashed CH552!')
        return False

    return True

def test_txrx_touchpad():
    """Test UART communication, RGB LED, and touchpad by asking the operator to interact with the touch pad"""
    description = {
        'vid':0x1207,
        'pid':0x8887,
        'manufacturer':'Tillitis',
        'product':'MTA1-USB-V1'
    }
    
    s = serial.Serial(find_serial_device(description),9600, timeout=.2)

    if not s.isOpen():
        print('couldn\'t find/open serial device')
        return False

    for i in range(0,5):
        # Attempt to clear any buffered data from the serial port
        s.write(b'0123')
        time.sleep(0.2)
        s.read(20)
    
        try:
            s.write(b'0')
            [count, touch_count] = s.read(2)
            print('read count:{:}, touch count:{:}'.format(count,touch_count))
        
            input('\n\n\nPress touch pad once and check LED, then press Enter')
            s.write(b'0')
            [count_post, touch_count_post] = s.read(2)
            print('read count:{:}, touch count:{:}'.format(count_post,touch_count_post))
        
            if (count_post - count != 1) or (touch_count_post - touch_count !=1):
                print('Unexpected values returned, trying again')
                continue
        
            return True
        except ValueError as e:
            print(e)
            continue

    print('Max retries exceeded, failure!')
    return False


def program_pico():
    """Load the ice40 flasher firmware onto the TP-1"""
    print('Attach test rig to USB (times out in 10 seconds)')
    for trys in range(0,100): # retry every 0.1s
        try:
            shutil.copyfile(
                file_locations['pico_bootloader_source'],
                file_locations['pico_bootloader_target']
                )

            # TODO: Test if the pico identifies as a USB-HID device after programming

            return True
        except FileNotFoundError:
            time.sleep(0.1)
        except PermissionError:
            time.sleep(0.1)

    return False


def sleep_2():
    """Sleep for 2 seconds"""
    time.sleep(2)
    return True

manual_tests = [
        program_pico,
        voltage_test,
        flash_validate_id,
        flash_program,
        flash_check,
        test_extra_io,
        ch552_program,
        test_txrx_touchpad,
        enable_power,
        disable_power
        ]

test_sequences = {
        'tk1_test_sequence' : [
            voltage_test,
            flash_validate_id,
            flash_program,
            sleep_2,
            test_extra_io,
            ch552_program,
            test_txrx_touchpad
        ],
        'tp1_test_sequence' : [
            program_pico,
            sleep_2,
            flash_validate_id
        ],
        'mta1_usb_v1_programmer_test_sequence' : [
            program_pico,
            sleep_2,
            voltage_test,
            flash_validate_id,
            sleep_2,
            test_extra_io
        ],
        }



pass_msg = '''
  _____                _____    _____ 
 |  __ \      /\      / ____|  / ____|
 | |__) |    /  \    | (___   | (___  
 |  ___/    / /\ \    \___ \   \___ \ 
 | |       / ____ \   ____) |  ____) |
 |_|      /_/    \_\ |_____/  |_____/ 
 '''

fail_msg = '''
  ______              _____   _      
 |  ____|     /\     |_   _| | |     
 | |__       /  \      | |   | |     
 |  __|     / /\ \     | |   | |     
 | |       / ____ \   _| |_  | |____ 
 |_|      /_/    \_\ |_____| |______|
 '''

ANSI = {
        'fg_black':"\u001b[30m",
        'fg_red':"\u001b[31m",
        'fg_green':"\u001b[32m",
        'fg_yellow':"\u001b[33m",
        'fg_blue':"\u001b[34m",
        'fg_magenta':"\u001b[35m",
        'fg_cyan':"\u001b[36m",
        'fg_white':"\u001b[37m",
        'bg_black':"\u001b[40m",
        'bg_red':"\u001b[41m",
        'bg_green':"\u001b[42m",
        'bg_yellow':"\u001b[43m",
        'bg_blue':"\u001b[44m",
        'bg_magenta':"\u001b[45m",
        'bg_cyan':"\u001b[46m",
        'bg_white':"\u001b[47m",
        'reset':"\u001b[0m",
        'bold':"\u001b[1m",
        'underline':"\u001b[4m"
        }

def run_tests(test_list):

    for test in test_list:
        print("\n{:}Test step: {:}{:} ({:})".format(ANSI['bold'],test.__name__, ANSI['reset'], test.__doc__))
        if not test():
            print('error running test step ' + test.__name__)
            return False

    return True

if __name__ == '__main__':
    last_a = 0

    print('\n\nTillitis TK-1 and TP-1 Production tests')

    while True:
        print('\n\n')

        options = []

        print('=== Test sequences ===')
        i = 1
        for name, tests in test_sequences.items():
            print('{:}{:}. {:}{:}: {:}'.format(ANSI['bold'], i, name, ANSI['reset'], ', '.join([test.__name__ for test in tests])))
            options.append(tests)
            i += 1

        print('\n=== Manual tests ===')
        for test in manual_tests:
            print('{:}{:}. {:}{:}: {:}'.format(ANSI['bold'], i, test.__name__, ANSI['reset'], test.__doc__))
            options.append([test])
            i += 1


        if(int(last_a) == 0):
            a = input('\n\n\nPlease type an option number and press return:')
        else:
            a = input('\n\n\nPress return to re-run test {:}, or type in a new option number and press return:'.format(last_a))
            if a == '':
                a = last_a

        try:
            test_sequence = options[int(a)-1]
        except IndexError as e:
            print('Invalid input')
            continue
        except ValueError as e:
            print('Invalid input')
            continue

        try:
            result = run_tests(test_sequence)
        except Exception as e:
            print(e)
            result = False

        if not result:
            print(ANSI['bg_red'] + fail_msg + ANSI['reset'])

            try: 
                disable_power()
            except AttributeError as e:
                pass
            except OSError as e:
                pass
        else:
            print(ANSI['bg_green'] + pass_msg + ANSI['reset'])

        last_a = a

