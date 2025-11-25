ANDA SENSOR LUZ #define F_CPU 8000000UL
#include <avr/io.h>
#include <util/delay.h>
#include <stdint.h>
#include <string.h>
#include "SENSOR_LUZ.h"

#define SEND_INTERVAL_MIN 10// Intervalo entre envíos en minutos

// ---------------- UART ----------------
#define BAUD 9600
#define UBRR_VALUE ((F_CPU/16/BAUD)-1)

void UART_init(void){
	UBRR0H = (uint8_t)(UBRR_VALUE >> 8);
	UBRR0L = (uint8_t)(UBRR_VALUE & 0xFF);
	UCSR0C = (1<<UCSZ01)|(1<<UCSZ00); // 8N1
	UCSR0B = (1<<TXEN0);               // solo TX
}

void UART_send_char(char c){
	while(!(UCSR0A & (1<<UDRE0)));
	UDR0 = c;
}

void UART_send_str(const char *s){
	while(*s) UART_send_char(*s++);
}

// ---------------- Sigfox ----------------
void sigfox_send_hex_payload(const uint8_t *data, uint8_t len){
	char buf[64];
	char *p = buf;
	strcpy(p,"AT$SF=");
	p += 6;
	for(uint8_t i=0;i<len;i++){
		uint8_t hi = (data[i]>>4)&0x0F;
		uint8_t lo = data[i]&0x0F;
		*p++ = (hi<10)?('0'+hi):('A'+hi-10);
		*p++ = (lo<10)?('0'+lo):('A'+lo-10);
	}
	*p = '\0';
	UART_send_str(buf);
	UART_send_str("\r\n"); // CR+LF
	_delay_ms(1500);       // espera suficiente al módulo
}

// ---------------- Delay confiable ----------------
void delay_sec(uint16_t sec){
	for(uint16_t i=0;i<sec;i++){
		_delay_ms(1000);
	}
}

// ---------------- MAIN ----------------
int main(void){
	UART_init();
	OPT101_init();

	uint8_t payload[8];
	const float calibration_factor = 1.18f;

	// contador de envío
	uint32_t seconds_counter = 0;

	while(1){
		// ----------------- Lectura sensor -----------------
		uint16_t raw_luz = OPT101_read();
		float lux = OPT101_to_lux(raw_luz) * calibration_factor;

		// convertimos a uint16_t (2 bytes)
		uint16_t luz_val = (uint16_t)lux;

		// ----------------- Empaquetar payload -----------------
		payload[0] = (luz_val >> 8) & 0xFF;
		payload[1] = luz_val & 0xFF;
		payload[2] = 0x00; // temperatura placeholder
		payload[3] = 0x00;
		payload[4] = 0x00; // humedad placeholder
		payload[5] = 0x00;
		payload[6] = 0x00; // presión placeholder
		payload[7] = 0x00;

		// ----------------- Enviar solo si pasó el intervalo -----------------
		if(seconds_counter >= SEND_INTERVAL_MIN * 60){
			sigfox_send_hex_payload(payload,8);
			seconds_counter = 0; // reinicia contador
		}

		// espera 1 segundo antes de siguiente lectura
		_delay_ms(1000);
		seconds_counter++;
	}
}
