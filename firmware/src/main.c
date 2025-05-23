#include <stdlib.h>
#include <stdbool.h>
#include <stdint.h>

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <avr/sleep.h>
#include <avr/eeprom.h>
#include <avr/wdt.h>

#include <util/delay.h>


#include "debug.h"
#include "stddef.h"

#define ENABLE_WHEEL					0
#define PS2_SAMPLES_PER_SEC		40	// 10..200


#define EEPROM_OFFSET_MULTIPLIER		0

#define led_enable()		PORTD |= _BV(5);
#define led_disable()		PORTD &= ~_BV(5);


//===========================================================================
// PS/2 порт
//===========================================================================

#define PS2_BUF_SIZE 256  // Размер приёмного буфера PS/2 порта

#define PS2_CLK_PORT  D // Ножка к которой подлючен тактовый сиг. PS/2
#define PS2_CLK_PIN   2
#define PS2_DATA_PORT D // Ножка к которой подлючен сигнал данных PS/2 
#define PS2_DATA_PIN  4

#define ps2_data()		(PIND & _BV(4))
#define get_jumper()	(PINB & _BV(2))
#define get_com_power()	(PIND & _BV(3))

enum ps_state_t { 
	ps2_state_error, 
	ps2_state_read, 
	ps2_state_write 
};

volatile uint8_t ps2_state;                // состояние порта (ps_state_t)
volatile uint8_t ps2_bitcount;             // счётчик битов обработчика
volatile uint8_t ps2_data;                 // буфер на байт
volatile uint8_t ps2_parity;
volatile uint8_t ps2_rx_buf[PS2_BUF_SIZE]; // Приёмный буфер PS/2 порта
volatile uint8_t ps2_rx_buf_w;
volatile uint8_t ps2_rx_buf_r;
volatile uint8_t ps2_rx_buf_count;


volatile bool send_rs232_flag;


//---------------------------------------------------------------------------
// Сохранить принятый байт в буфер приёма PS/2 порта. Вызывается только из обработчика прерывания.

void ps2_rx_push(uint8_t c) {
	// Если буфер переполнен и потерян байт, то программа не сможет правильно 
	// расшифровать все дальнейшие пакеты, поэтому перезагружаем контроллер.
	if (ps2_rx_buf_count >= sizeof(ps2_rx_buf)) {
		ps2_state = ps2_state_error;
		return;
	}
	// Сохраняем в буфер
	ps2_rx_buf[ps2_rx_buf_w] = c;
	ps2_rx_buf_count++;
	if (++ps2_rx_buf_w == sizeof(ps2_rx_buf)) {
		ps2_rx_buf_w = 0;
	}
}

//---------------------------------------------------------------------------
// Получить байт из приёмного буфера PS/2 порта

uint8_t ps2_read(void) {
	uint8_t d;
	
	cli();	// Выключаем прерывания, так как обработчик прерывания тоже модифицирует эти переменные.
	// Если буфер пуст, возвращаем ноль
	if (ps2_rx_buf_count == 0) {
		d = 0;
	} else {
		// Читаем байт из буфера
		d = ps2_rx_buf[ps2_rx_buf_r];
		ps2_rx_buf_count--;
		if (++ps2_rx_buf_r == sizeof(ps2_rx_buf)) {
			ps2_rx_buf_r = 0;
		}
    }
    
    sei();	// Включаем прерывания
    return d;
}


//---------------------------------------------------------------------------
// Вычисление бита чётности
uint8_t parity(uint8_t p) {
	p = p ^ (p >> 4 | p << 4);
	p = p ^ (p >> 2);
	p = p ^ (p >> 1);
	return (p ^ 1) & 1;
}

//---------------------------------------------------------------------------
// Изменение тактового сигнала PS/2
ISR (INT0_vect) {
	if (ps2_state == ps2_state_error) {
		return;
	}

	if (ps2_state == ps2_state_write) {
		switch (ps2_bitcount) {
			default: // Данные
				if (ps2_data & 1) {
					DDR(PS2_DATA_PORT) &= ~_BV(PS2_DATA_PIN);
				} else {
					DDR(PS2_DATA_PORT) |= _BV(PS2_DATA_PIN);
				}
				ps2_data >>= 1;
				break;
			case 3: // Бит чётности
				if (ps2_parity) {
					DDR(PS2_DATA_PORT) &= ~_BV(PS2_DATA_PIN);
				} else {
					DDR(PS2_DATA_PORT) |= _BV(PS2_DATA_PIN);
				}
				break;
			case 2: // Стоп бит
				DDR(PS2_DATA_PORT) &= ~_BV(PS2_DATA_PIN);
				break;
			case 1: // Подтверждение приёма
				if (ps2_data()) {
					ps2_state = ps2_state_error;
				} else {
					ps2_state = ps2_state_read; 
				}
				ps2_bitcount = 12;
				break;
					 
        } 
    } else {
		switch (ps2_bitcount) {
			case 11: // Старт бит
				if (ps2_data()) {
					ps2_state = ps2_state_error;
				}
				break;
			default: // Данные
				ps2_data >>= 1;
				if (ps2_data()) {
					ps2_data |= 0x80;
				}
				break;
			case 2: // Бит четности 
				if (parity(ps2_data) != (ps2_data() != 0)) {
					ps2_state = ps2_state_error;
				}
				break;
			case 1: // Стоп бит 
				if (ps2_data()) {
					ps2_rx_push(ps2_data);
				} else {
					ps2_state = ps2_state_error;
				}
				ps2_bitcount = 12;
		}
	}
	ps2_bitcount--;
}

//---------------------------------------------------------------------------
// Инициализация PS/2

void ps2_init(void) {
	// Переключаем PS/2 порт на приём
	DDR(PS2_CLK_PORT) &= ~_BV(PS2_CLK_PIN);
	DDR(PS2_DATA_PORT) &= ~_BV(PS2_DATA_PIN);
    
	// Очищаем приёмный буфер
	ps2_rx_buf_w = 0;
	ps2_rx_buf_r = 0;
	ps2_rx_buf_count = 0;
    
	// Устанавливаем переменные обработчика прерывания
	ps2_state = ps2_state_read;
	ps2_bitcount = 11;

	// Прерывание по срезу тактового сигнала
	GIFR = _BV(INTF0);
	GICR |= _BV(INT0);
	MCUCR = (MCUCR & 0xFC) | 2;
}

//---------------------------------------------------------------------------
// Отправка байта в PS/2 порт без подтверждения
void ps2_write(uint8_t a) {
    // Отключаем прерывание по изменению тактового сигнала PS/2
    GIFR = _BV(INTF0);
    GICR &= ~_BV(INT0);

    // Замыкаем тактовый сигнал PS/2 на землю
    PORT(PS2_CLK_PORT) &= ~_BV(PS2_CLK_PIN);
    DDR(PS2_CLK_PORT) |= _BV(PS2_CLK_PIN);

    // ждём в течение 100 мкс
    _delay_us(100);

    // Замыкаем линию данных PS/2 на землю
    PORT(PS2_DATA_PORT) &= ~_BV(PS2_DATA_PIN);
    DDR(PS2_DATA_PORT) |= _BV(PS2_DATA_PIN);

    // Освобождаем тактовый сигнал
    DDR(PS2_CLK_PORT) &= ~_BV(PS2_CLK_PIN);

    // Очищаем приёмный буфер
    ps2_rx_buf_count = 0;
    ps2_rx_buf_w = 0;
    ps2_rx_buf_r = 0;

    // Настраиваем переменные обработчика прерывания
    ps2_state = ps2_state_write;
    ps2_bitcount = 11;
    ps2_data = a;             
    ps2_parity = parity(a);

    // Включаем прерывание по срезу тактового сигнала PS/2
    GIFR = _BV(INTF0);
    GICR |= _BV(INT0);
    MCUCR = (MCUCR & 0xFC) | 2;    
}

//---------------------------------------------------------------------------
// Получение байта из PS/2 порта с ожиданием
uint8_t ps2_recv(void) {
    while (ps2_rx_buf_count == 0);
    return ps2_read();
}

//---------------------------------------------------------------------------
// Отправка байта в PS/2 порт с подтверждением

void ps2_send(uint8_t c) {
    ps2_write(c);
    if (ps2_recv() != 0xFA) {
        ps2_state = ps2_state_error;
    }
}

//===========================================================================
// RS232 порт
//===========================================================================

//#define RS232_TX_BUFFER_SIZE 512
#define RS232_TX_BUFFER_SIZE 256

uint8_t rs232_tx_buf[RS232_TX_BUFFER_SIZE];
volatile uint8_t rs232_tx_buf_w;
volatile uint8_t rs232_tx_buf_r;
volatile uint8_t rs232_tx_buf_count;
volatile bool rs232_reset;
volatile bool rs232_enabled;

void rs232_send(uint8_t c) {
    // Ожидание, если буфер переполнен
    while (rs232_tx_buf_count == sizeof(rs232_tx_buf));

    // Выключаем прерывания, так как обработчик прерывания тоже модифицирует эти переменные
    cli();

    // Если передача уже идёт или в буфере передачи что-то есть, то сохраняем в буфер значение
    if (rs232_tx_buf_count || ((UCSRA & _BV(UDRE)) == 0)) {
        rs232_tx_buf[rs232_tx_buf_w] = c;
        if (++rs232_tx_buf_w == sizeof(rs232_tx_buf)) {
            rs232_tx_buf_w = 0;
        }
        rs232_tx_buf_count++;
    } else {
        // Иначе выводим значение в порт
        UDR = c;
    }

    // Включаем прерывания
    sei();
}

//---------------------------------------------------------------------------
// COM-порт отправил байт
ISR (USART_TXC_vect) {
	// Если буфер пуст, то ничего не делаем
	if (!rs232_tx_buf_count) {
		return;
	}

	// Иначе отправляем байт из буфера
	UDR = rs232_tx_buf[rs232_tx_buf_r];
	rs232_tx_buf_count--;
	if (++rs232_tx_buf_r == sizeof(rs232_tx_buf)) {
		rs232_tx_buf_r = 0;
	}
}

//---------------------------------------------------------------------------
// Изменилось состояние линий DTR или RTS
ISR (INT1_vect) {
	// Сохраняем состояние в переменную
	//rs232_enabled = get_com_power() ? 0 : 1;
	rs232_enabled = !get_com_power();
	// Сохраняем признак сброса
	rs232_reset = true;          
}

//===========================================================================
//  Светодиод
//===========================================================================


// Включить светодиод
void flash_led() {
	led_enable();
	TCNT2 = 0x00;
	TCCR2 = _BV(CS22)|_BV(CS21)|_BV(CS20);	// делитель на 1024
}

//---------------------------------------------------------------------------
// Выключение светодиода через некоторое время
ISR (TIMER2_OVF_vect) {
    TCCR2 = 0;
    led_disable();
}

//===========================================================================
// Инициализация PS/2 мыши
//===========================================================================

#if ENABLE_WHEEL
bool  ps2m_whell;   // Используемый протокол: 0=без колеса, 1=с колесом
#endif
uint8_t  ps2m_multiplier; // Масштабирование координат
uint8_t  ps2m_b;          // Нажатые кнопки
int16_t  ps2m_x, ps2m_y, ps2m_z; // Координаты

//---------------------------------------------------------------------------
// Инициализация PS/2 мыши

static void ps2m_init() {
	// Посылаем команду "Сброс"
	ps2_send(0xFF);
	if (ps2_recv() != 0xAA) { 
		ps2_state = ps2_state_error; 
		return; 
	}
	if (ps2_recv() != 0x00) { 
		ps2_state = ps2_state_error; 
		return; 
	}
	
#if ENABLE_WHEEL
	// Включаем колесо и побочно устанавливаем 80 пакетов в секунду.    
	ps2_send(0xF3);
	ps2_send(0xC8);
	ps2_send(0xF3);
	ps2_send(0x64);
	ps2_send(0xF3);
	ps2_send(0x50);

	// Узнаём, получилось ли включить колесо
	ps2_send(0xF2);
	ps2m_whell = ps2_recv();
#endif

	// Разрешение 8 точек на мм
	ps2_send(0xE8);
	ps2_send(0x03);

	// Задаём количество сэмплов/сек
	ps2_send(0xF3);
	ps2_send(PS2_SAMPLES_PER_SEC);

	// Включаем потоковый режим.
	ps2_send(0xF4);
}

//---------------------------------------------------------------------------
// Обработка поступивших с PS/2 порта данных

void ps2m_process() {
#if ENABLE_WHEEL
//	while (ps2_rx_buf_count < (3 + (ps2m_whell ? 1 : 0))) {
//	}
	while (ps2_rx_buf_count >= (3 + (ps2m_whell ? 1 : 0))) {
		ps2m_b = ps2_read() & 7; //! Тут старшие биты!!!
		ps2m_x += (int8_t)ps2_read();
		ps2m_y -= (int8_t)ps2_read();
		if (ps2m_whell) {
			ps2m_z += (int8_t)ps2_read();
		}
	}
#else
	
	//while (ps2_rx_buf_count < 3) {
	//}
	while (ps2_rx_buf_count >= 3) {
		ps2m_b = ps2_read() & 7; //! Тут старшие биты!!!
		ps2m_x += (int8_t)ps2_read();
		ps2m_y -= (int8_t)ps2_read();
	}	
#endif
}          

//===========================================================================
// Наплатные кнопки
//===========================================================================

#define BUTTONS_TIMEOUT 50

uint8_t buttons_disabled = 0;    // Таймаут для предотвращения дребезга
uint8_t pressed_button   = 0xFF; // Последняя нажатая кнопка

// Периодический опрос джамперов скорости
// 80.13 Hz
ISR (TIMER0_OVF_vect) {
	TCNT0 = 0x3d;

	static uint8_t cnt = 0;
	send_rs232_flag = cnt & 1;
	cnt++;

	
	uint8_t b = PINC & 7;
	if (!buttons_disabled) {
		if (!(b & _BV(0))) {
			pressed_button = 0; 
		} else if (!(b & _BV(1))) {
			pressed_button = 1; 
		} else if (!(b & _BV(2))) {
			pressed_button = 2; 
		} else {
			return;
		}
	}
	if (b == 7) {
		buttons_disabled = BUTTONS_TIMEOUT;
	}
	buttons_disabled--;
}

//===========================================================================
// Интерфейс с компьютером
//===========================================================================

#define PROTOCOL_MICROSOFT		0
#define PROTOCOL_EM84520		1

uint8_t rs232m_protocol; // Используемый протокол: 0=MSMouse, 1=EM84520

const uint8_t EM84520_ID[61] = {
    0x4D, 0x5A, 0x40, 0x00, 0x00, 0x00, 0x08, 0x01, 0x24, 0x25, 0x2D, 0x23,
    0x10, 0x10, 0x10, 0x11, 0x3C, 0x3C, 0x2D, 0x2F, 0x35, 0x33, 0x25, 0x3C,
    0x30, 0x2E, 0x30, 0x10, 0x26, 0x10, 0x21, 0x3C, 0x25, 0x2D, 0x23, 0x00,
    0x33, 0x23, 0x32, 0x2F, 0x2C, 0x2C, 0x29, 0x2E, 0x27, 0x00, 0x33, 0x25,
    0x32, 0x29, 0x21, 0x2C, 0x00, 0x2D, 0x2F, 0x35, 0x33, 0x25, 0x21, 0x15,
    0x09
};

//---------------------------------------------------------------------------

static void rs232m_init() {
    // Протокол определяется перемычкой на плате
    rs232m_protocol = get_jumper() ? PROTOCOL_MICROSOFT : PROTOCOL_EM84520;

    // Настройка RS232: 1200 бод, 1 стоп бит, 7 бит, нет чётности
    UCSRA = 0;
    UCSRB = _BV(TXEN) | _BV(RXEN) | _BV(TXCIE);
	 // В режиме MS регистр USBS = 0 (1 стоп-бит), в режиме EM84520 - 2 стоп-бита
    UCSRC = rs232m_protocol == PROTOCOL_EM84520 ? (_BV(URSEL)|_BV(USBS)|_BV(UCSZ1)) : (_BV(URSEL)|_BV(UCSZ1));
    
    UBRRH = 0x03;	//0x02;
    UBRRL = 0x40;	//0xFF;
    
    // По умолчанию включён
    //rs232_enabled = true;

    // Вывести приветствие
    //rs232_reset = true;    
	 
    rs232_enabled = !get_com_power();
    if (rs232_enabled) {
        rs232_reset = true; 
    }
}

//---------------------------------------------------------------------------

void rs232m_send(int8_t x, int8_t y, int8_t z, uint8_t b) {
    uint8_t lb, rb, mb;
    static uint8_t mb1;
    
    // Обработка сброса      
    if (rs232_reset) {
        rs232_reset = false; 
        _delay_ms(14);
        if (rs232m_protocol == PROTOCOL_EM84520) {
            // Приветствие EM84520
            for (uint8_t i = 0; i < sizeof(EM84520_ID); i++) {
                rs232_send(EM84520_ID[i]);
            }
        } else {
            // Приветствие Logitech/Microsoft Plus
            rs232_send(0x4D);
            _delay_ms(63);
            rs232_send(0x33);
        }
    }
    
    // Клавиши мыши
    lb = b & 1;
    rb = (b >> 1) & 1;
    mb = (b >> 2) & 1;

    // Стандартная часть протокола 
    rs232_send((1 << 6) | (lb << 5) | (rb << 4) | ((y & 0xC0) >> 4) | ((x & 0xC0) >> 6));
    rs232_send(x & 0x3F);
    rs232_send(y & 0x3F);
    
    if (rs232m_protocol == PROTOCOL_EM84520) {
        // Расширение EM84520
        rs232_send((mb << 4) | (z & 0x0F));
    } else { 
        // Расширение Logitech/Microsoft Plus
        if (mb || mb1) {
            rs232_send(mb << 5);
            mb1 = mb;
        }
    }                    
}

//===========================================================================
// Программа
//===========================================================================

static void init(void) {
	// Настройка портов ввода-вывода и подтягивающих резисторов
	DDRB = 0;//_BV(3);//0x08; 
	PORTB = _BV(2);//_BV(0)|_BV(1)|_BV(2)|_BV(3)|_BV(4)|_BV(5);//0x3F;
DDRB = _BV(5);	 
	DDRC = 0;//0x00; 	 
	PORTC = _BV(0)|_BV(1)|_BV(2);//0x7F;
	 
	DDRD = _BV(1)|_BV(5);//0x02; 
	PORTD = 0;//0xE2;

	// Timer/Counter 0 initialization
	// Clock source: System Clock
	// Clock value: 57,600 kHz
//	TCCR0 = 4;
//	TCNT0 = 0;
	// ~ 80.13Hz
	TCCR0 = _BV(CS02) | _BV(CS00);	// 1024 divider
	TCNT0 = 0x3d;


	// Таймер 2
	ASSR = 0; 
	TCCR2 = _BV(CS22)|_BV(CS21)|_BV(CS20);	// CLK/1024
	TCNT2 = 0; 
	OCR2 = 0;

	// Включение прерывания по изменению на входе INT1
	GICR |= _BV(INT1);
	MCUCR = _BV(ISC10);	// Any logical change on INT1 generates an interrupt request.
	GIFR = _BV(INTF1);

	// Timer(s)/Counter(s) Interrupt(s) initialization
	TIMSK = 0; 
    
	// Включаем прерывания от таймеров 0 и 2
	TIMSK |= _BV(TOIE0)|_BV(TOIE2);//0x41;

	// Analog Comparator initialization
	// Analog Comparator: Off
	// Analog Comparator Input Capture by Timer/Counter 1: Off
	ACSR = _BV(ACD);
	SFIOR = 0;

	// Настройка Watchdog-таймера
	wdt_enable(WDTO_1S);	

	// Включение прерываний
	sei();
}

//static void (*jump_to_bootloader)(void) = (void*)0x1c00;
//static void (*jump_to_bootloader)(void) = (void*)0xe00;

//---------------------------------------------------------------------------

//eeprom uint8_t eeprom_ps2m_multiplier;


static void sentToRs232() {
	static uint8_t smb1 = 0;
	
	send_rs232_flag = false;
	if (!rs232_enabled) {
		return;
	}
	if (ps2m_b != smb1 || ps2m_x != 0 || ps2m_y != 0 || ps2m_z != 0 || rs232_reset) {
//				if (rs232_tx_buf_count == 0) {
		int8_t cx = ps2m_x < -128 ? -128 : (ps2m_x > 127 ? 127 : ps2m_x); 
		ps2m_x -= cx;
		int8_t cy = ps2m_y < -128 ? -128 : (ps2m_y > 127 ? 127 : ps2m_y); 
		ps2m_y -= cy;
		int8_t cz = ps2m_z < -8   ? -8   : (ps2m_z > 7   ?   7 : ps2m_z); 
		ps2m_z -= cz;
		
		smb1 = ps2m_b;
		rs232m_send(cx, cy, cz, ps2m_b);
		flash_led();
	}
}

static void checkBootloader() {
	// если мышь отключена, то проверяем команды конфигурации с перехода на загрузчик
	// '?' -> 'M'		команда определения типа устройства
	// 'tsbl'			команда перехода на загрузчик
	// иначе -> '!'
	
	static uint8_t goto_bootloader_cnt = 0;
	char ch = UDR;
	switch (ch) {
		case '?':
			rs232_send('M');
			goto_bootloader_cnt = 0;
			break;
		case 't':
			goto_bootloader_cnt = 1;
			break;
		case 's':
			if (goto_bootloader_cnt == 1) {
				goto_bootloader_cnt++;
			} else {
				goto_bootloader_cnt = 0;
			}
			break;
		case 'b':
			if (goto_bootloader_cnt == 2) {
				goto_bootloader_cnt++;
			} else {
				goto_bootloader_cnt = 0;
			}
			break;
		case 'l':
			if (goto_bootloader_cnt == 3) {
				wdt_reset();
				asm("ijmp" :: "z" (0x0E00)); // size = 512 words
			}
			goto_bootloader_cnt = 0;
			break;
		default:
			goto_bootloader_cnt = 0;
			rs232_send('!');
			break;
	}
}

static void checkJumpers() {
	if (pressed_button != 0xFF) {
		ps2m_multiplier = pressed_button;
		eeprom_update_byte(EEPROM_OFFSET_MULTIPLIER, ps2m_multiplier);
		//flash_led();
		pressed_button = 0xFF;
	}
}

void main(void) {
	// Восстанавливаем настройки	 
	ps2m_multiplier = eeprom_read_byte(EEPROM_OFFSET_MULTIPLIER);
	if (ps2m_multiplier > 2) {
		ps2m_multiplier = 1; 
	}
    
	init();
	ps2_init();
	ps2m_init();
	rs232m_init();
	flash_led();

	for(;;) {
		if (send_rs232_flag) {
			sentToRs232();
		}
		// читаем данные из PS/2
		ps2m_process();
        
		// Отправляем компьютеру пакет, если буфер отправки пуст, мышь включена, 
		// изменились нажатые кнопки или положение мыши
		if (send_rs232_flag) {
			TCNT0 = 0x3d;
			sentToRs232();
		} else if (UCSRA & _BV(RXC)) {	// Если байт готов
			checkBootloader();
		}		

		checkJumpers();
		
		// Регулирование скорости мыши прямо с мыши
		if (rs232m_protocol == PROTOCOL_MICROSOFT && (ps2m_b & 3) == 3) {
			if (ps2m_z < 0) { 
				if (ps2m_multiplier > 0) {
					ps2m_multiplier--; 
				}
				ps2m_z = 0; 
			} else if (ps2m_z > 0) { 
				if (ps2m_multiplier < 2) {
					ps2m_multiplier++;
				}
				ps2m_z = 0; 
			}
		}

                   
		// В случае ошибки перезагружаемся
		if (ps2_state != ps2_state_error) {
			wdt_reset();
		}
 
 
	}
}
