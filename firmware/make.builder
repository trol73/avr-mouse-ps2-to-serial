
name = 'ps2-to-serial'

src = ['src/*.c']

mcu = 'atmega8'

frequency = 16*1000000

port = 'ft0'

baudrate = 1200

programmer = '2ftbb'

defines = ['F_CPU=16000000', 'DEBUG=0']

compiler_options = ['-g2']

linker_options = []

