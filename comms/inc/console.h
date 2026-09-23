/** console.h - The ASCII console on USART3. */
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
