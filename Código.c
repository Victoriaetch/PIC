#define F_CPU 16000000UL
#include <avr/io.h>
#include <util/delay.h>
#include <avr/wdt.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <avr/sleep.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "TWI.h"   

// CONFIG 
#define MEASURE_INTERVAL_S 300UL   // segundos entre envíos 
#define WDT_INTERVAL_S 8UL         // WDT ~8s
#define TARGET_CYCLES ((MEASURE_INTERVAL_S + WDT_INTERVAL_S - 1) / WDT_INTERVAL_S)

#define BAUD 9600
#define UBRR_VALUE ((F_CPU/16/BAUD)-1)

#define BME280_ADDR 0x76

// Variables BME calib 
uint16_t dig_T1, dig_P1;
int16_t dig_T2, dig_T3;
int16_t dig_P2, dig_P3, dig_P4, dig_P5, dig_P6, dig_P7, dig_P8, dig_P9;
uint8_t dig_H1, dig_H3;
int16_t dig_H2, dig_H4, dig_H5, dig_H6;
int32_t t_fine;

// Watchdog
volatile uint16_t wdt_cycles = 0;
volatile uint8_t wdt_woke = 0;

// UART (Sigfox)
void UART_init_hw(void){
	UBRR0H = (uint8_t)(UBRR_VALUE >> 8);
	UBRR0L = (uint8_t)UBRR_VALUE;
	UCSR0B = 0x00; // TX/RX deshabilitado para ahorrar energia por default
	UCSR0C = (1<<UCSZ01)|(1<<UCSZ00);
}
void UART_enable_tx(void){ UCSR0B |= (1<<TXEN0); }
void UART_disable_tx(void){ UCSR0B &= ~(1<<TXEN0); }
void UART_enable_rx(void){ UCSR0B |= (1<<RXEN0); }
void UART_disable_rx(void){ UCSR0B &= ~(1<<RXEN0); }

void UART_send(char c){
	while(!(UCSR0A & (1<<UDRE0)));
	UDR0 = c;
}
void UART_print(const char* s){
	while(*s) UART_send(*s++);
}
void UART_print_n(const char* s, size_t n){
	for(size_t i=0;i<n;i++) UART_send(s[i]);
}

// SIGFOX helper (manda hex payload via AT$SF) 
void sigfox_send_hex_payload(const uint8_t *data, uint8_t len){
	// Build hex string (uppercase)
	char cmd[3*13]; // suficiente para 12 bytes -> 24 hex + "AT$SF="...
	char *p = cmd;
	strcpy(p, "AT$SF=");
	p += 6;
	for(uint8_t i=0;i<len;i++){
		uint8_t hi = (data[i]>>4) & 0x0F;
		uint8_t lo = data[i] & 0x0F;
		*p++ = (hi < 10) ? ('0'+hi) : ('A'+hi-10);
		*p++ = (lo < 10) ? ('0'+lo) : ('A'+lo-10);
	}
	*p = '\0';

	UART_enable_tx();
	UART_print(cmd);
	UART_print("\r"); // CR (módulo Wisol espera CR)
	// opcional: no bloqueante esperar respuesta breve (envío fire-and-forget)
	_delay_ms(200);
	UART_disable_tx();
}

// ADC (OPT101) 
void adc_init(void){
	ADMUX = (1<<REFS0) | 7; // AVcc ref
	ADCSRA = 0; // off
}
void adc_enable(void){
	ADCSRA = (1<<ADEN)|(1<<ADPS2)|(1<<ADPS1)|(1<<ADPS0); // prescaler 128
	_delay_ms(2);
}
void adc_disable(void){
	ADCSRA &= ~(1<<ADEN);
}
uint16_t adc_read(uint8_t ch){
	ADMUX = (1<<REFS0) | (ch & 0x07);
	ADCSRA |= (1<<ADSC);
	while(ADCSRA & (1<<ADSC));
	return ADC;
}

// BME280 I2C helpers 
void BME280_write(uint8_t reg, uint8_t data){
	twi_start();
	twi_write_cmd((BME280_ADDR<<1)|0);
	twi_write_dwr(reg);
	twi_write_dwr(data);
	twi_stop();
}
uint8_t BME280_read8(uint8_t reg){
	twi_start();
	twi_write_cmd((BME280_ADDR<<1)|0);
	twi_write_dwr(reg);
	twi_repeated_start();
	twi_write_cmd((BME280_ADDR<<1)|1);
	uint8_t v = twi_read_nack();
	twi_stop();
	return v;
}
uint16_t BME280_read16_LE(uint8_t reg){
	uint16_t v;
	twi_start();
	twi_write_cmd((BME280_ADDR<<1)|0);
	twi_write_dwr(reg);
	twi_repeated_start();
	twi_write_cmd((BME280_ADDR<<1)|1);
	uint8_t lo = twi_read_ack();
	uint8_t hi = twi_read_nack();
	twi_stop();
	v = ((uint16_t)hi<<8) | lo;
	return (v>>8) | (v<<8);
}
int16_t BME280_readS16_LE(uint8_t reg){ return (int16_t)BME280_read16_LE(reg); }
uint32_t BME280_read24(uint8_t reg){
	twi_start();
	twi_write_cmd((BME280_ADDR<<1)|0);
	twi_write_dwr(reg);
	twi_repeated_start();
	twi_write_cmd((BME280_ADDR<<1)|1);
	uint8_t msb = twi_read_ack();
	uint8_t lsb = twi_read_ack();
	uint8_t xlsb = twi_read_nack();
	twi_stop();
	return ((uint32_t)msb<<12) | ((uint32_t)lsb<<4) | (xlsb>>4);
}

// BME280 calibraciones y conversiones
void BME280_read_calibration(void){
	dig_T1 = BME280_read16_LE(0x88);
	dig_T2 = BME280_readS16_LE(0x8A);
	dig_T3 = BME280_readS16_LE(0x8C);
	dig_P1 = BME280_read16_LE(0x8E);
	dig_P2 = BME280_readS16_LE(0x90);
	dig_P3 = BME280_readS16_LE(0x92);
	dig_P4 = BME280_readS16_LE(0x94);
	dig_P5 = BME280_readS16_LE(0x96);
	dig_P6 = BME280_readS16_LE(0x98);
	dig_P7 = BME280_readS16_LE(0x9A);
	dig_P8 = BME280_readS16_LE(0x9C);
	dig_P9 = BME280_readS16_LE(0x9E);
	dig_H1 = BME280_read8(0xA1);
	dig_H2 = BME280_readS16_LE(0xE1);
	dig_H3 = BME280_read8(0xE3);
	uint8_t e4 = BME280_read8(0xE4);
	uint8_t e5 = BME280_read8(0xE5);
	uint8_t e6 = BME280_read8(0xE6);
	dig_H4 = (int16_t)((e4 << 4) | (e5 & 0x0F));
	dig_H5 = (int16_t)((e6 << 4) | (e5 >> 4));
	dig_H6 = (int8_t)BME280_read8(0xE7);
}

void BME280_init(void){
	// set ctrl_hum (write before ctrl_meas)
	twi_init();
	BME280_write(0xF2, 0x01); // osrs_h = x1
	BME280_write(0xF5, 0x00); // config: filter off, standby minimal
	BME280_read_calibration();
}

void BME280_start_forced(uint8_t osrs_t, uint8_t osrs_p){
	uint8_t ctrl = ((osrs_t & 0x07)<<5) | ((osrs_p & 0x07)<<2) | 0x01; // mode=01 forced
	BME280_write(0xF4, ctrl);
	// wait until measuring bit clears
	uint16_t to = 0;
	while((BME280_read8(0xF3) & 0x08) && (to++ < 1000)) _delay_ms(1);
}

float BME280_read_temperature(void){
	uint32_t adc_T = BME280_read24(0xFA);
	int32_t var1, var2, T;
	var1 = ((((adc_T>>3) - ((int32_t)dig_T1<<1))) * ((int32_t)dig_T2)) >> 11;
	var2 = (((((adc_T>>4) - ((int32_t)dig_T1)) * ((adc_T>>4) - ((int32_t)dig_T1))) >> 12) * ((int32_t)dig_T3)) >> 14;
	t_fine = var1 + var2;
	T = (t_fine * 5 + 128) >> 8;
	return T / 100.0f;
}

float BME280_read_pressure(void){
	uint32_t adc_P = BME280_read24(0xF7);
	int64_t var1, var2, p;
	var1 = ((int64_t)t_fine) - 128000;
	var2 = var1 * var1 * (int64_t)dig_P6;
	var2 = var2 + ((var1*(int64_t)dig_P5)<<17);
	var2 = var2 + (((int64_t)dig_P4)<<35);
	var1 = ((var1 * var1 * (int64_t)dig_P3)>>8) + ((var1 * (int64_t)dig_P2)<<12);
	var1 = (((((int64_t)1)<<47)+var1) * (int64_t)dig_P1) >> 33;
	if (var1 == 0) return 0.0;
	p = 1048576 - adc_P;
	p = (((p<<31) - var2) * 3125) / var1;
	var1 = (((int64_t)dig_P9) * (p>>13) * (p>>13)) >> 25;
	var2 = (((int64_t)dig_P8) * p) >> 19;
	p = ((p + var1 + var2)>>8) + (((int64_t)dig_P7)<<4);
	return (float)p / 256.0f;
}

float BME280_read_humidity(void){
	// read 16-bit humidity
	twi_start();
	twi_write_cmd((BME280_ADDR<<1)|0);
	twi_write_dwr(0xFD);
	twi_repeated_start();
	twi_write_cmd((BME280_ADDR<<1)|1);
	uint16_t raw = (twi_read_ack()<<8) | twi_read_nack();
	twi_stop();

	int32_t v_x1 = t_fine - 76800;
	int32_t v = (((((raw<<14) - (((int32_t)dig_H4)<<20) - ((int32_t)dig_H5)*v_x1) + 16384) >> 15) *
	((((((v_x1 * (int32_t)dig_H6) >> 10) * (((v_x1 * (int32_t)dig_H3) >> 11) + 32768)) >> 10) + 2097152) *
	(int32_t)dig_H2 + 8192) >> 14);
	v = v - (((((v >> 15) * (v >> 15)) >> 7) * (int32_t)dig_H1) >> 4);
	if (v < 0) v = 0;
	if (v > 419430400) v = 419430400;
	return (v >> 12) / 1024.0f;
}

// WDT interrupt setup
ISR(WDT_vect){
	wdt_cycles++;
	wdt_woke = 1;
}

void wdt_init_interrupt_8s(void){
	cli();
	MCUSR &= ~(1<<WDRF);
	WDTCSR |= (1<<WDCE)|(1<<WDE);
	WDTCSR = (1<<WDIE)|(1<<WDP3)|(1<<WDP0); // ~8s on ATmega328P
	sei();
}

// Sleep 
void enter_sleep(void){
	set_sleep_mode(SLEEP_MODE_PWR_DOWN);
	sleep_enable();
	sei(); // make sure interrupts enabled
	sleep_cpu();
	sleep_disable();
}

int main(void){
	// inicializacion
	UART_init_hw();    // leave TX off
	adc_init();
	BME280_init();     // calls twi_init internally
	wdt_init_interrupt_8s();

	// short debug message (enable tx briefly)
	UART_enable_tx();
	UART_print("Iniciado: Sigfox + BME280 + OPT101\r\n");
	UART_disable_tx();

	if (wdt_cycles >= TARGET_CYCLES) wdt_cycles = 0;
	while(1){
		wdt_woke = 0;
		enter_sleep(); // wake on WDT

		if (!wdt_woke) continue;
		if (wdt_cycles < TARGET_CYCLES) continue;

		// time to measure+send
		wdt_cycles = 0;

		// ADC (OPT101)
		adc_enable();
		_delay_ms(2);
		uint16_t luz_adc = adc_read(7); // ADC0 pin
		adc_disable();

		// BME forced measurement (osrs x1 for T,P,H)
		BME280_start_forced(1,1); // osrs_t=1, osrs_p=1 ; humidity oversampling already set
		float temp = BME280_read_temperature();
		float pres = BME280_read_pressure(); // Pa scale -> convert to hPa when packing
		float hum  = BME280_read_humidity();

		// Pack payload (8 bytes)
		// format: temp (int16, °C *100), hum (uint16, % *100), pres (uint16, hPa*10), luz (uint16 ADC)
		int16_t t10 = (int16_t)(temp * 100.0f);      // -327.67 .. +327.67°C safe
		uint16_t h10 = (uint16_t)(hum * 100.0f);     // 0..10000
		uint16_t p10 = (uint16_t)( (pres/100.0f) * 10.0f ); // Pa->hPa *10
		uint8_t payload[8];
		payload[0] = (uint8_t)( (t10 >> 8) & 0xFF );
		payload[1] = (uint8_t)( t10 & 0xFF );
		payload[2] = (uint8_t)( (h10 >> 8) & 0xFF );
		payload[3] = (uint8_t)( h10 & 0xFF );
		payload[4] = (uint8_t)( (p10 >> 8) & 0xFF );
		payload[5] = (uint8_t)( p10 & 0xFF );
		payload[6] = (uint8_t)( (luz_adc >> 8) & 0xFF );
		payload[7] = (uint8_t)( luz_adc & 0xFF );

		// Send via Sigfox AT$SF
		sigfox_send_hex_payload(payload, sizeof(payload));
		// optionally blink LED or set flag
	}
	return 0;
}
