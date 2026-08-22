/**
  ******************************************************************************
  * @file    console.h
  * @brief   The ASCII console on USART3.
  *
  * It exists for one reason: to get into the binary link and back out by hand.
  * Every reading it used to print is a binary command now, decoded on the host.
  * Adding a report back here would mean two implementations of the same
  * measurement, which is exactly how the two ADC read paths drifted apart.
  ******************************************************************************
  */
#ifndef CONSOLE_H
#define CONSOLE_H

#ifdef __cplusplus
extern "C" {
#endif

/** Print the boot banner: identity, clock tree, and the key bindings. */
void Console_Banner(void);

/** Poll for one keypress. Non-blocking; call from the main loop. */
void Console_Poll(void);

#ifdef __cplusplus
}
#endif

#endif /* CONSOLE_H */
