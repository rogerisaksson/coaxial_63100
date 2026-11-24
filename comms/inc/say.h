/** say.h - the ASCII console's words written without stdio or heap: text, decimals, hex. */
#ifndef SAY_H
#define SAY_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/** `text` up to its NUL, as it stands. */
void say_text(const char *text);

/** `value` in decimal, unpadded: what printf's %u and %lu wrote. */
void say_u32(uint32_t value);

/** `value` in upper-case hex, `width` digits at least and 8 at most: what printf's %0<width>X wrote. */
void say_hex(uint32_t value, uint8_t width);

#ifdef __cplusplus
}
#endif

#endif /* SAY_H */
