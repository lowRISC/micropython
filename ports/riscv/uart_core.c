#include <unistd.h>
#include "py/mpconfig.h"

/*
 * Core UART functions to implement for a port
 */

extern uint8_t uart_recv();
extern void uart_send_buf(const char *buf, const int32_t len);

// Receive single character
int mp_hal_stdin_rx_chr(void) {
  return uart_recv();
}

// Send string of given length
void mp_hal_stdout_tx_strn(const char *str, mp_uint_t len) {
  uart_send_buf(str, len);
}
