
#include "raspberry_soft_uart.h"

#define TX_BUFFER_FLUSH_TIMEOUT 4000 // milliseconds

// Driver instance.
// static
//
static int uart_instance_count = 0;
struct tty_driver *soft_uart_driver;

/**
 * Opens a given TTY device.
 * @param tty given TTY device
 * @param file
 * @return error code.
 */
static int soft_uart_open(struct tty_struct *tty, struct file *file)
{
  int error = 0;

  if (raspberry_soft_uart_open(tty))
  {
    printk(KERN_INFO "soft_uart: Device opened.\n");
  }
  else
  {
    printk(KERN_ALERT "soft_uart: Device busy.\n");
    error = -ENODEV;
  }

  return error;
}

/**
 * Closes a given TTY device.
 * @param tty
 * @param file
 */
static void soft_uart_close(struct tty_struct *tty, struct file *file)
{
  // Waits for the TX buffer to be empty before closing the UART.
  int wait_time = 0;
  while ((raspberry_soft_uart_get_tx_queue_size(tty) > 0) && (wait_time < TX_BUFFER_FLUSH_TIMEOUT))
  {
    // TODO : msleep(100);
    wait_time += 100;
  }

  if (raspberry_soft_uart_close(tty))
  {
    printk(KERN_INFO "soft_uart: Device closed.\n");
  }
  else
  {
    printk(KERN_ALERT "soft_uart: Could not close the device.\n");
  }
}

/**
 * Writes the contents of a given buffer into a given TTY device.
 * @param tty given TTY device
 * @param buffer given buffer
 * @param buffer_size number of bytes contained in the given buffer
 * @return number of bytes successfuly written into the TTY device
 */
static int soft_uart_write(struct tty_struct *tty, const unsigned char *buffer, int buffer_size)
{
  return raspberry_soft_uart_send_string(tty, buffer, buffer_size);
}

/**
 * Tells the kernel the number of bytes that can be written to a given TTY.
 * @param tty given TTY
 * @return number of bytes
 */
static unsigned int soft_uart_write_room(struct tty_struct *tty)
{
  return raspberry_soft_uart_get_tx_queue_room(tty);
}

/**
 * Does nothing.
 * @param tty
 */
static void soft_uart_flush_buffer(struct tty_struct *tty)
{
}

/**
 * Tells the kernel the number of bytes contained in the buffer of a given TTY.
 * @param tty given TTY
 * @return number of bytes
 */
static unsigned int soft_uart_chars_in_buffer(struct tty_struct *tty)
{
  return raspberry_soft_uart_get_tx_queue_size(tty);
}

/**
 * Sets the UART parameters for a given TTY (only the baudrate is taken into account).
 * @param tty given TTY
 * @param termios parameters
 */
static void soft_uart_set_termios(struct tty_struct *tty, struct ktermios *termios)
{
  struct soft_uart_instance *instance = tty->driver_data;

  int cflag = 0;
  speed_t baudrate = tty_get_baud_rate(tty);
  printk(KERN_INFO "soft_uart: soft_uart_set_termios: baudrate = %d.\n", baudrate);

  // Gets the cflag.
  cflag = tty->termios.c_cflag;

  // Verifies the number of data bits (it must be 8).
  if ((cflag & CSIZE) != CS8)
  {
    printk(KERN_ALERT "soft_uart: Invalid number of data bits.\n");
  }

  // Verifies the number of stop bits (it must be 1).
  if (cflag & CSTOPB)
  {
    printk(KERN_ALERT "soft_uart: Invalid number of stop bits.\n");
  }

  // Verifies the parity (it must be none).
  if (cflag & PARENB)
  {
    printk(KERN_ALERT "soft_uart: Invalid parity.\n");
  }

  // set baudrate
  instance->period = ktime_set(0, 1000000000 / baudrate);
  gpio_set_debounce(instance->gpio_rx, 1000 / baudrate / 2);
  // TODO set bitstop and parity
}

/**
 * Does nothing.
 * @param tty
 */
static void soft_uart_stop(struct tty_struct *tty)
{
  printk(KERN_DEBUG "soft_uart: soft_uart_stop.\n");
}

/**
 * Does nothing.
 * @param tty
 */
static void soft_uart_start(struct tty_struct *tty)
{
  printk(KERN_DEBUG "soft_uart: soft_uart_start.\n");
}

/**
 * Does nothing.
 * @param tty
 */
static void soft_uart_hangup(struct tty_struct *tty)
{
  printk(KERN_DEBUG "soft_uart: soft_uart_hangup.\n");
}

/**
 * Does nothing.
 * @param tty
 */
static int soft_uart_tiocmget(struct tty_struct *tty)
{
  return 0;
}

/**
 * Does nothing.
 * @param tty
 * @param set
 * @param clear
 */
static int soft_uart_tiocmset(struct tty_struct *tty, unsigned int set, unsigned int clear)
{
  return 0;
}

/**
 * This function is a placeholder for IOCTL operations for a soft UART.
 * It returns 0 for specific recognized commands and -ENOIOCTLCMD for others.
 *
 * @param tty Pointer to the TTY structure.
 * @param command IOCTL command.
 * @param parameter Additional parameter for the IOCTL command.
 * @return Error code: 0 for recognized commands (TIOCMSET, TIOCMGET), -ENOIOCTLCMD for others.
 */
static int soft_uart_ioctl(struct tty_struct *tty, unsigned int command, unsigned int long parameter)
{
  // Return 0 for recognized commands, -ENOIOCTLCMD otherwise
  return (command == TIOCMSET || command == TIOCMGET) ? 0 : -ENOIOCTLCMD;
}

/**
 * Does nothing.
 * @param tty
 */
static void soft_uart_throttle(struct tty_struct *tty)
{
  printk(KERN_DEBUG "soft_uart: soft_uart_throttle.\n");
}

/**
 * Does nothing.
 * @param tty
 */
static void soft_uart_unthrottle(struct tty_struct *tty)
{
  printk(KERN_DEBUG "soft_uart: soft_uart_unthrottle.\n");
}

// Module operations.
struct tty_operations soft_uart_operations = {
    .open = soft_uart_open,
    .close = soft_uart_close,
    .write = soft_uart_write,
    .write_room = soft_uart_write_room,
    .flush_buffer = soft_uart_flush_buffer,
    .chars_in_buffer = soft_uart_chars_in_buffer,
    .ioctl = soft_uart_ioctl,
    .set_termios = soft_uart_set_termios,
    .stop = soft_uart_stop,
    .start = soft_uart_start,
    .hangup = soft_uart_hangup,
    .tiocmget = soft_uart_tiocmget,
    .tiocmset = soft_uart_tiocmset,
    .throttle = soft_uart_throttle,
    .unthrottle = soft_uart_unthrottle};

// Probe function
static int soft_uart_probe(struct platform_device *pdev)
{
  struct device *dev = &pdev->dev;
  struct device_node *node = dev->of_node;
  struct device_node *child;
  int ret = 0, i = 0;
  int gpio_tx, gpio_rx;
  struct soft_uart_instance *uart_internal_data;
  struct tty_port *port;
  struct tty_struct *tty;
  uart_instance_count = of_get_child_count(node);

  soft_uart_driver = tty_alloc_driver(uart_instance_count, TTY_DRIVER_REAL_RAW | TTY_DRIVER_DYNAMIC_DEV);
  if (IS_ERR(soft_uart_driver))
  {
    printk(KERN_ALERT "soft_uart: Failed to allocate the driver.\n");
    return PTR_ERR(soft_uart_driver);
  }

  // Set common properties for the driver
  soft_uart_driver->owner = THIS_MODULE;
  soft_uart_driver->driver_name = "soft_uart";
  soft_uart_driver->name = "ttySOFT";
  soft_uart_driver->major = TTY_MAJOR;
  soft_uart_driver->minor_start = 0;
  soft_uart_driver->type = TTY_DRIVER_TYPE_SERIAL;
  soft_uart_driver->subtype = SERIAL_TYPE_NORMAL;
  soft_uart_driver->init_termios = tty_std_termios;
  soft_uart_driver->init_termios.c_ispeed = 4800;
  soft_uart_driver->init_termios.c_ospeed = 4800;
  soft_uart_driver->init_termios.c_cflag = B4800 | CREAD | CS8 | CLOCAL;
  soft_uart_driver->flags = TTY_DRIVER_REAL_RAW;
  tty_set_operations(soft_uart_driver, &soft_uart_operations);

  // Allocate tty_struct and tty_port for each instance
  soft_uart_driver->ttys = devm_kcalloc(dev, uart_instance_count, sizeof(struct tty_struct *), GFP_KERNEL);
  soft_uart_driver->ports = devm_kcalloc(dev, uart_instance_count, sizeof(struct tty_port *), GFP_KERNEL);

  if (!soft_uart_driver->ttys || !soft_uart_driver->ports)
  {
    tty_driver_kref_put(soft_uart_driver);
    return -ENOMEM;
  }

  for_each_available_child_of_node(node, child)
  {
    gpio_tx = of_get_named_gpio(child, "tx-pin", 0);
    if (gpio_tx < 0)
    {
      ret = -EIO;
      goto exit_cleanup;
    }

    gpio_rx = of_get_named_gpio(child, "rx-pin", 0);
    if (gpio_rx < 0)
    {
      ret = -EIO;
      goto exit_cleanup;
    }
    uart_internal_data = devm_kcalloc(dev, 1, sizeof(*uart_internal_data), GFP_KERNEL);
    if (!uart_internal_data)
    {
      return -ENOMEM;
    }
    uart_internal_data->gpio_tx = gpio_tx;
    uart_internal_data->gpio_rx = gpio_rx;
    uart_internal_data->idx = i;
    uart_internal_data->rx_bit_index = -1;
    // Initialize tty_port and link it to the tty_driver
    port = devm_kzalloc(dev, sizeof(struct tty_port), GFP_KERNEL);
    if (!port)
    {
      ret = -ENOMEM;
      goto exit_cleanup;
    }
    tty_port_init(port);
    soft_uart_driver->ports[i] = port;
    tty = tty_port_tty_get(port);
    if (tty)
    {
      tty->driver_data = uart_internal_data;
      // todo maybe no this ? : tty_kref_put(tty);
    }
    if (!raspberry_soft_uart_init(tty))
    {
      dev_err(dev, "Failed to initialize GPIO for node %pOFn.\n", child);
      ret = -EIO;
      goto exit_cleanup;
    }

    tty_port_link_device(port, soft_uart_driver, i);
    uart_internal_data->port = port;
    i++;
  }

  if (tty_register_driver(soft_uart_driver))
  {
    printk(KERN_ALERT "soft_uart: Failed to register the driver.\n");
    tty_driver_kref_put(soft_uart_driver);
    return -EIO;
  }

  return 0;

exit_cleanup:
  // Cleanup code for partially initialized instances
  tty_driver_kref_put(soft_uart_driver);
  // Additional cleanup as needed...
  return ret;
}

static int soft_uart_remove(struct platform_device *pdev)
{
  int i;
  printk(KERN_INFO "soft_uart: Finalizing the module...\n");
  // Iterate over each UART instance and clean up
  for (i = 0; i < uart_instance_count; i++)
  {
    // If there's any specific cleanup required for tty_port
    struct tty_port *port = soft_uart_driver->ports[i];
    if (port)
    {
      struct tty_struct *tty = tty_port_tty_get(port);
      if (tty)
      {

        struct soft_uart_instance *instance = tty->driver_data;
        if (instance)
        {
          // Release GPIO resources
          free_irq(gpio_to_irq(instance->gpio_rx), NULL);
          if (gpio_is_valid(instance->gpio_tx))
            gpio_free(instance->gpio_tx);
          if (gpio_is_valid(instance->gpio_rx))
            gpio_free(instance->gpio_rx);

          kfree(tty->driver_data); // Freeing uart_internal_data
        }
        tty_kref_put(tty);
        tty_port_destroy(port);
        // No need to free port here as it was allocated with devm_kzalloc
      }
    }
  }

  // Unregister the TTY driver
  if (soft_uart_driver)
  {
    tty_unregister_driver(soft_uart_driver);
    tty_driver_kref_put(soft_uart_driver);
  }

  printk(KERN_INFO "soft_uart: Module finalized.\n");
  return 0;
}

// Platform driver structure
static const struct of_device_id soft_uart_dt_ids[] = {
    {
        .compatible = "akasoft,soft-uart",
    },
    {/* sentinel */}};

MODULE_DEVICE_TABLE(of, soft_uart_dt_ids);

static struct platform_driver soft_uart_platform_driver = {
    .probe = soft_uart_probe,
    .remove = soft_uart_remove,
    .driver = {
        .name = "soft_uart",
        .of_match_table = soft_uart_dt_ids,
    },
};

// Module init function
static int __init soft_uart_init(void)
{
  printk(KERN_INFO "soft_uart: Initializing module...\n");
  return platform_driver_register(&soft_uart_platform_driver);
}

// Module exit function
static void __exit soft_uart_exit(void)
{
  platform_driver_unregister(&soft_uart_platform_driver);
}

module_init(soft_uart_init);
module_exit(soft_uart_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Your Name");
MODULE_DESCRIPTION("Software UART Driver");
