/** board_console.c - The ASCII console's bytes out on USART3, the board's alone (say.c). */
#include "board.h"
#include "board_hw.h"

/* The board's own: the native and fake boards, which build none of USART3, write the console's
   words to the host's stdout (fake_board.c). */
void Board_ConsoleWrite(const uint8_t *data, uint16_t len)
{
  (void)HAL_UART_Transmit(&huart3, data, len, HAL_MAX_DELAY);
}
