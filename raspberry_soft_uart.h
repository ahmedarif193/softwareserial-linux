#ifndef RASPBERRY_SOFT_UART_H
#define RASPBERRY_SOFT_UART_H

#include "queue.h"

// Linux Kernel Includes
#include <linux/gpio.h>
#include <linux/hrtimer.h>
#include <linux/interrupt.h>
#include <linux/ktime.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/platform_device.h>
#include <linux/tty.h>
#include <linux/tty_driver.h>
#include <linux/tty_flip.h>
#include <linux/version.h>

struct soft_uart_instance
{
    int gpio_tx;
    int gpio_rx;
    int idx;
    struct queue queue_tx;
    struct hrtimer timer_tx;
    struct hrtimer timer_rx;
    ktime_t period;
    int rx_bit_index;
    struct tty_port *port; // to write from rx to tty
};

// Function Declarations for Raspberry Soft UART
int raspberry_soft_uart_init(struct tty_struct *tty);
int raspberry_soft_uart_open(struct tty_struct *tty);
int raspberry_soft_uart_close(struct tty_struct *tty);
int raspberry_soft_uart_send_string(struct tty_struct *tty, const unsigned char *string, int string_size);
int raspberry_soft_uart_get_tx_queue_room(struct tty_struct *tty);
int raspberry_soft_uart_get_tx_queue_size(struct tty_struct *tty);

#endif // RASPBERRY_SOFT_UART_H