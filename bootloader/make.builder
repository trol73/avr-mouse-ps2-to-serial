name = 'avr-bootloader'

# --------[Bootloader configuration]------------------------

mcu = 'atmega8'

frequency = 16*1000000

#UART_BAUD_RATE = 57600
#UART_BAUD_RATE = 230400
#UART_BAUD_RATE = 153600

USE_SECOND_UART = 0

BOOTLOADER_SIZE = 0x0400

READ_PROTECT_BOOTLOADER=0
ENABLE_READ_FUSELOCK=0

# --------[Avrdude]-----------------------------------------

port = ''
baudrate = 1200
programmer = 'arduino'

# ---------[Devices] ---------------------------------------

devices = {
	'atmega8': {
		'page_size': 0x40,
		'boot_start': 0x1800,
		'boot_sizes': [0x0100, 0x0200, 0x0400, 0x0800]
	},
	'atmega128': {
		'page_size': 0x100,
		'boot_start': 0x1e000,
		'boot_sizes': [0x0400, 0x0800, 0x1000, 0x2000]
	},
	'atmega328': {
		'page_size': 0x80,
		'boot_start': 0x7000,
		'boot_sizes': [0x0200, 0x0400, 0x0800, 0x1000]
	},
}

# -----------------------------------------------------------
src = [
   '../../avr-bootloader/src/*.c',
   '../../avr-bootloader/src/asm/*.s'
]



def error(msg):
	print msg
	import sys
	sys.exit(-1)


if not mcu in devices.keys():
	error('Device is not supported: ' + mcu)

if not 'UART_BAUD_RATE' in globals():
	if frequency == 16e6:
		UART_BAUD_RATE = 153600
	elif frequency == 20e6:
		UART_BAUD_RATE = 230400
	else:
		print frequency
		error('UART_BAUD_RATE not defined')
	print 'Baudrate not specified and set as', UART_BAUD_RATE
	
dev = devices[mcu]
size_is_supported = False
for bs in dev['boot_sizes']:
	if BOOTLOADER_SIZE == bs:
		size_is_supported = True
		break
if not size_is_supported:
	error('Wrong bootloader size: ' + str(BOOTLOADER_SIZE) + ', must be one from ' + str(dev['boot_sizes']))
bootstart = dev['boot_start'] + dev['boot_sizes'][len(dev['boot_sizes'])-1] - BOOTLOADER_SIZE
print 'Bootloader start:', hex(bootstart)
print 'Bootloader size:',  BOOTLOADER_SIZE, 'bytes'
print
	
defines = [
   'UART_RX_BUFFER_SIZE=256',
   'UART_TX_BUFFER_SIZE=64',
   'UART_BAUD_RATE=' + str(UART_BAUD_RATE),
   'USE_SECOND_UART=' + str(USE_SECOND_UART),
   'BOOTLOADER_START=' + str(bootstart),
   'PAGE_SIZE=' + str(dev['page_size']),
   'BOOTLOADER_SIZE=' + str(BOOTLOADER_SIZE),
   'READ_PROTECT_BOOTLOADER=' + str(READ_PROTECT_BOOTLOADER),
   'ENABLE_READ_FUSELOCK=' + str(ENABLE_READ_FUSELOCK),
   'DEBUG=0'
]

compiler_options = ['-g2']

linker_options = [
	'-Wl,--section-start=.text=' + hex(bootstart),
	'-nostartfiles'
]


configurations = {
}




