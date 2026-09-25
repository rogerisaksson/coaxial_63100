/** native.h - What board/native's files share: the clock, the interrupts, the parts. */
#ifndef NATIVE_H
#define NATIVE_H

#include <stdbool.h>
#include <stdint.h>

/* native.c: the clock; main() waiting `cycles`, the interrupts due running; an interrupt's
   handler run as the NVIC runs it; whether one is enabled. */
uint64_t native_now(void);
void native_wait(uint64_t cycles);
void native_irq(int irq, void (*handler)(void));
bool native_irq_enabled(int irq);

/* native.c: what the parts sense - AFE_ON, the board's temperature, the shaft. */
bool native_powered(void);
double native_ntc_celsius(void);
double native_shaft_degrees(void);

/* native_io.c: the pins and the SPI buses. */
void native_io_open(void);
void native_io_poll(void);

/* native_a1335.c: the angle sensor on SPI4, chip select PE4. */
void a1335_open(void);
void a1335_select(bool level);
uint8_t a1335_transmit(uint8_t word);

/* native_bno085.c: the IMU on SPI2; its pins 0 chip select PB12, 1 WAKE PD9, 2 NRSTN PD10. */
void bno085_open(void);
void bno085_pin(uint8_t which, bool level);
uint8_t bno085_transmit(uint8_t byte);
bool bno085_interrupt(void);
void bno085_tick(void);

#endif /* NATIVE_H */
