#include <util/delay.h>
#include <avr/io.h> 
#include <inttypes.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <avr/power.h>
#include "nRF24L01.h"

#define CLICKER_CONFIG ( (1<<EN_CRC) | (1<<CRCO) )

#define LED_PIN1 PA0
#define LED_PIN2 PA1

#define BUTTON PB2

#define NRF_CE  PA7
#define NRF_CSN PA3
#define NRF_DO  PA5
#define NRF_DI  PA6
#define NRF_SCK PA4

#define CE_HIGH PORTA |=  _BV(NRF_CE)
#define CE_LOW  PORTA &= ~_BV(NRF_CE)
#define CSN_HIGH PORTA |= _BV(NRF_CSN)
#define CSN_LOW PORTA &= ~_BV(NRF_CSN)

uint8_t tx_addr[3] = {0x56, 0x34, 0x12}; // sent lsb first = 0x123456
uint8_t my_addr[3] = {0xC5, 0x80, 0x37}; // sent lsb first = 0x3780C5
uint8_t PTX = 0;

void click(char vote);
void blink_red();
void blink_green();
void setup();
void setup_nrf();
void setup_spi();
void send(uint8_t value);
uint8_t get_data();
uint8_t get_status();
void flush_rx();
void power_up_rx();
void power_up_tx();
uint8_t data_ready();
uint8_t rx_fifo_empty();
void write_register(uint8_t reg, uint8_t value);
void write_register_n(uint8_t reg, uint8_t * data, uint8_t len);
uint8_t spi_transfer(uint8_t data);
void spi_write(uint8_t * data, uint8_t len);
void spi_write_read(uint8_t * data, uint8_t len);

ISR( EXT_INT0_vect ) {
  cli();

  click('1');
}

int main() {
  setup();
  setup_spi();
  setup_nrf();

  blink_red();
  _delay_ms(100);
  blink_green();
  _delay_ms(100);
  blink_red();
  _delay_ms(100);
  blink_green();

  _delay_ms(1000);

  if(get_status() == 0xe) {
    blink_green();
    _delay_ms(100);
    blink_green();
  }

  while(42) {
    sei();
  }
}

void click(char vote) {
  uint8_t resp;

  power_up_rx();
  
  while(!data_ready()) {
    send(vote);
    power_up_rx();
    _delay_ms(100);
    blink_red();
  }
  
  resp = get_data();
  
  if(resp == 0x11) {
    blink_green();
    _delay_ms(100);
    blink_green();
  }
  
  flush_rx();
  
  write_register(CONFIG, CLICKER_CONFIG | (0 << PWR_UP));
  _delay_ms(500);
  power_up_rx();

}

void blink_red() {
  PORTA |= (1 << LED_PIN2);
  _delay_ms(100);
  PORTA &= (0 << LED_PIN2);
}

void blink_green() {
  PORTA |= (1 << LED_PIN1);
  _delay_ms(100);
  PORTA &= (0 << LED_PIN1);
}

void setup() {
  DDRA |= (1 << LED_PIN1);
  DDRA |= (1 << LED_PIN2);

  GIMSK |= (1 << INT0);
  PORTB |= (1 << BUTTON);
}

void setup_nrf() {
  CE_LOW;
  CSN_HIGH;

  write_register(EN_AA, 0x00);           // no shockburst
  write_register(SETUP_AW, 0x01);        // 3 byte address
  write_register(RF_CH, 26);             // channel 53   
  write_register(RF_SETUP, 0x06);        // 1Mbps - max power  
  write_register(SETUP_RETR, 0x00);      // no auto-retry
  write_register_n(RX_ADDR_P0, my_addr, 3); // my_addr (made-up)
  write_register_n(TX_ADDR, tx_addr, 3);    // where were sending to
  write_register(RX_PW_P0, 0x01);        // receiving payload size of 1
  flush_rx();
}

void setup_spi() {
  DDRA |= _BV(NRF_CE);
  DDRA |= _BV(NRF_CSN);
  DDRA |= _BV(NRF_DO);
  DDRA |= _BV(NRF_SCK);
  DDRA &= ~_BV(NRF_DI);
  PORTA |= _BV(NRF_DI);
}

void send(uint8_t value) {
  uint8_t status;
  status = get_status();
  while (PTX) {
    status = get_status();
    if((status & ((1 << TX_DS)  | (1 << MAX_RT)))){
      PTX = 0;
      break;
    }
  }
  CE_LOW;
  power_up_tx();

  CSN_LOW;
  spi_transfer( FLUSH_TX ); 
  CSN_HIGH;

  uint8_t data[5];
  data[0] = W_TX_PAYLOAD;
  data[1] = my_addr[2];
  data[2] = my_addr[1];
  data[3] = my_addr[0];
  data[4] = value;

  CSN_LOW;
  spi_write(data, 5);
  CSN_HIGH;

  CE_HIGH;
}

uint8_t get_data() {
  uint8_t resp;

  CSN_LOW;
  spi_transfer(R_RX_PAYLOAD);
  resp = spi_transfer(0);
  CSN_HIGH;

  write_register(STATUS, (1<<RX_DR));

  return resp;
}

void power_up_rx() {
  PTX = 0;
  CE_LOW;
  write_register(CONFIG, CLICKER_CONFIG | ( (1<<PWR_UP) | (1<<PRIM_RX) ) );
  CE_HIGH;
  write_register(STATUS,(1 << TX_DS) | (1 << MAX_RT));
}

void power_up_tx() {
  PTX = 1;
  write_register(CONFIG, CLICKER_CONFIG | ( (1<<PWR_UP) | (0<<PRIM_RX) ) );
}

void flush_rx() {
  CSN_LOW;
  spi_transfer(FLUSH_RX);
  CSN_HIGH;
}

uint8_t data_ready() {
  uint8_t status = get_status();
  if ( status & (1 << RX_DR) ) return 1;
  return !rx_fifo_empty();
}

uint8_t rx_fifo_empty() {
  uint8_t status = 0x00;
  CSN_LOW;
  spi_transfer(FIFO_STATUS);
  status = spi_transfer(0);
  CSN_HIGH;
  return (status & (1 << RX_EMPTY));
}

uint8_t get_status() {
  uint8_t val;

  CSN_LOW;
  val =  spi_transfer(0x00);
  CSN_HIGH;

  return val;
}

void write_register(uint8_t reg, uint8_t value) {
  CSN_LOW;
  uint8_t data[2] = {W_REGISTER | (REGISTER_MASK & reg), \
		  value};
  spi_write(data, 2);
  CSN_HIGH;
}

void write_register_n(uint8_t reg, uint8_t * values, uint8_t len) {
  CSN_LOW;
  uint8_t data[len+1];
  data[0] = W_REGISTER | (REGISTER_MASK & reg);
  uint8_t i;
  for(i = 1; i < len+1; i++)
    data[i] = values[i-1];
  spi_write(data, len+1);
  CSN_HIGH;
}

void spi_write(uint8_t * data, uint8_t len) {
  uint8_t i;
  for(i = 0; i < len; i++)
    spi_transfer(data[i]);
}

void spi_write_read(uint8_t * data, uint8_t len) {
  uint8_t i;
  for(i = 0; i < len; i++)
    data[i] = spi_transfer(data[i]);
}

uint8_t spi_transfer(uint8_t data) {
  USIDR = data;
  USISR = _BV(USIOIF); // clear flag
 
  while ( (USISR & _BV(USIOIF)) == 0 ) {
    USICR = (1<<USIWM0)|(1<<USICS1)|(1<<USICLK)|(1<<USITC);
  }
  return USIDR;
}
