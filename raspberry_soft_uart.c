
#include "raspberry_soft_uart.h"

static irq_handler_t handle_rx_start(unsigned int irq, void* device, struct pt_regs* registers);
static enum hrtimer_restart handle_tx(struct hrtimer* timer);
static enum hrtimer_restart handle_rx(struct hrtimer* timer);

extern struct soft_uart_instance *uart_instances;

static DEFINE_MUTEX(current_tty_mutex);

/**
 * Initializes the Raspberry Soft UART infrastructure.
 * This must be called during the module initialization.
 * The GPIO pin used as TX is configured as output.
 * The GPIO pin used as RX is configured as input.
 * @param gpio_tx GPIO pin used as TX
 * @param gpio_rx GPIO pin used as RX
 * @return 1 if the initialization is successful. 0 otherwise.
 */
int raspberry_soft_uart_init(struct tty_struct* tty)
{
  bool success = true;
  struct soft_uart_instance *instance = tty->driver_data;
  
  mutex_init(&current_tty_mutex);
  
  // Initializes the TX timer.
  hrtimer_init(&instance->timer_tx, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
  instance->timer_tx.function = &handle_tx;
  
  // Initializes the RX timer.
  hrtimer_init(&instance->timer_rx, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
  instance->timer_rx.function = &handle_rx;
   
  success &= gpio_request(instance->gpio_tx, "soft_uart_tx") == 0;
  success &= gpio_direction_output(instance->gpio_tx, 1) == 0;

  success &= gpio_request(instance->gpio_rx, "soft_uart_rx") == 0;
  success &= gpio_direction_input(instance->gpio_rx) == 0;
  
  // Initializes the interruption.
  success &= request_irq(
    gpio_to_irq(instance->gpio_rx),
    (irq_handler_t) handle_rx_start,
    IRQF_TRIGGER_FALLING,
    "soft_uart_irq_handler",
    instance) == 0;
  disable_irq(gpio_to_irq(instance->gpio_rx));
    
  return success;
}

/**
 * Opens the Soft UART.
 * @param tty
 * @return 1 if the operation is successful. 0 otherwise.
 */
int raspberry_soft_uart_open(struct tty_struct* tty)
{
  int success = 0;
  struct soft_uart_instance *instance = tty->driver_data;
  mutex_lock(&current_tty_mutex);
    initialize_queue(&instance->queue_tx);
    success = 1;
    enable_irq(gpio_to_irq(instance->gpio_rx));
  mutex_unlock(&current_tty_mutex);
  return success;
}

/**
 * Closes the Soft UART.
 */
int raspberry_soft_uart_close(struct tty_struct* tty)
{
  int success = 0;
  struct soft_uart_instance *instance = tty->driver_data;
  mutex_lock(&current_tty_mutex);

    disable_irq(gpio_to_irq(instance->gpio_rx));
    hrtimer_cancel(&instance->timer_tx);
    hrtimer_cancel(&instance->timer_rx);
    success = 1;

  mutex_unlock(&current_tty_mutex);
  return success;
}

/**
 * Adds a given string to the TX queue.
 * @paran string given string
 * @param string_size size of the given string
 * @return The amount of characters successfully added to the queue.
 */
int raspberry_soft_uart_send_string(struct tty_struct* tty, const unsigned char* string, int string_size)
{
  struct soft_uart_instance *instance = tty->driver_data;
  
  int result = enqueue_string(&instance->queue_tx, string, string_size);
  
  // Starts the TX timer if it is not already running.
  if (!hrtimer_active(&instance->timer_tx))
  {
    hrtimer_start(&instance->timer_tx, instance->period, HRTIMER_MODE_REL);
  }
  
  return result;
}

/*
 * Gets the number of characters that can be added to the TX queue.
 * @return number of characters.
 */
int raspberry_soft_uart_get_tx_queue_room(struct tty_struct* tty)
{
  struct soft_uart_instance *instance = tty->driver_data;
  return get_queue_room(&instance->queue_tx);
}

/*
 * Gets the number of characters in the TX queue.
 * @return number of characters. 
 */
//TODO not used 
int raspberry_soft_uart_get_tx_queue_size(struct tty_struct* tty)
{
  struct soft_uart_instance *instance = tty->driver_data;
  return get_queue_size(&instance->queue_tx);
}

/**
 * If we are waiting for the RX start bit, then starts the RX timer. Otherwise,
 * does nothing.
 */
static irq_handler_t handle_rx_start(unsigned int irq, void* device, struct pt_regs* registers)
{
    struct soft_uart_instance *instance = (struct soft_uart_instance *)device;

    if (instance && instance->rx_bit_index == -1)
    {
        hrtimer_start(&instance->timer_rx, ktime_set(0, instance->period / 2), HRTIMER_MODE_REL);
    }
    return (irq_handler_t) IRQ_HANDLED;
}


/**
 * Dequeues a character from the TX queue and sends it.
 */
static enum hrtimer_restart handle_tx(struct hrtimer* timer)
{
  struct soft_uart_instance *instance = container_of(timer, struct soft_uart_instance, timer_tx);

  ktime_t current_time = ktime_get();
  static unsigned char character = 0;
  static int bit_index = -1;
  
  // Start bit.
  if (bit_index == -1)
  {
    if (dequeue_character(&instance->queue_tx, &character))
    {
      gpio_set_value(instance->gpio_tx, 0);
      bit_index = 0;
      hrtimer_forward(&instance->timer_tx, current_time, instance->period);
      return HRTIMER_RESTART;
    }
  }
  
  // Data bits.
  else if (bit_index < 8)
  {
    gpio_set_value(instance->gpio_tx, character & 1);
    character >>= 1;
    bit_index++;
    hrtimer_forward(&instance->timer_tx, current_time, instance->period);
    return HRTIMER_RESTART;
  }
  
  // Stop bit.
  else if (bit_index == 8)
  {
    gpio_set_value(instance->gpio_tx, 1);
    bit_index = -1;
    if (get_queue_size(&instance->queue_tx) > 0)
    {
      hrtimer_forward(&instance->timer_tx, current_time, instance->period);
      return HRTIMER_RESTART;
    }
  }
  
  return HRTIMER_NORESTART;
}


static enum hrtimer_restart handle_rx(struct hrtimer* timer)
{
  //TODO verify this : 
  struct soft_uart_instance *instance = container_of(timer, struct soft_uart_instance, timer_rx);

  ktime_t current_time = ktime_get();
  static unsigned int character = 0;
  int bit_value = gpio_get_value(instance->gpio_rx);
  
  // Start bit.
  if (instance->rx_bit_index == -1)
  {
    instance->rx_bit_index = 0;
    character = 0;
    hrtimer_forward(&instance->timer_rx, current_time, instance->period);
    return HRTIMER_RESTART;
  }
  
  // Data bits.
  else if (instance->rx_bit_index < 8)
  {
    character >>= 1;
    if (bit_value)
    {
      character |= 0x80;
    }
    
    instance->rx_bit_index++;
    hrtimer_forward(&instance->timer_rx, current_time, instance->period);
    return HRTIMER_RESTART;
  }
  
  // Stop bit.
  else if (instance->rx_bit_index == 8)
  {
    mutex_lock(&current_tty_mutex);
    tty_insert_flip_char(instance->port, (unsigned char)character, TTY_NORMAL);
    tty_flip_buffer_push(instance->port);
    instance->rx_bit_index = -1;
    mutex_unlock(&current_tty_mutex);
  }
  
  return HRTIMER_NORESTART;
}
