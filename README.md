# softwareserial-linux


This project has been adapted from Adrianomarto's original work on soft UART for the Raspberry Pi, available at https://github.com/adrianomarto/soft_uart. 

A huge acknowledgment to him for his exceptional contribution. This version extends his work into a more generalized module, It uses device tree to create multiple instances of softserial ports.

use this dts section to map each pin to each module for example:  
/ {
    pinctrl {
        soft_uart0_pins: soft-uart0 {
            pinmux = <&gpio 17 0>, <&gpio 27 0>;
            bias-disable;
        };

        soft_uart1_pins: soft-uart1 {
            pinmux = <&gpio 19 0>, <&gpio 28 0>;
            bias-disable;
        };
    };

    soft_uart {
        compatible = "soft-uart";

        uart0 {
            compatible = "akasoft,soft-uart";
            pinctrl-names = "default";
            pinctrl-0 = <&soft_uart0_pins>;
            tx-pin = <&gpio 17 0>;  // GPIO pin 17 from MT7621 GPIO controller
            rx-pin = <&gpio 27 0>;  // GPIO pin 27 from MT7621 GPIO controller
        };

        uart1 {
            compatible = "akasoft,soft-uart";
            pinctrl-names = "default";
            pinctrl-0 = <&soft_uart1_pins>;
            tx-pin = <&gpio 19 0>;  // GPIO pin 19 from MT7621 GPIO controller
            rx-pin = <&gpio 28 0>;  // GPIO pin 28 from MT7621 GPIO controller
        };
    };
};

# Technical Considerations:

Pin Control (pinctrl) Integration: The soft_uart nodes are enhanced with pinctrl definitions. Each UART interface (uart0, uart1) has a corresponding pinctrl state (soft-uart0, soft-uart1) defined at the top level of the DTS. This approach provides a clearer mapping between the UART interfaces and their respective GPIO configurations.

Pin Multiplexing (pinmux): The pinmux property within each pinctrl state is used to specify the GPIO pins and their function (in this case, as UART Tx and Rx). The bias-disable property is added as a placeholder for pin bias configuration; this should be adjusted based on specific hardware requirements.

Reference to GPIO Bank: Each UART interface references the GPIO pins from the MT7621 GPIO controller. Care is taken to ensure that the correct GPIO bank is referenced (&gpio), as specified in the provided gpio node definition.

Testing with libgpiod:

Once the DTS is recompiled and the system is configured with the new DTB, you can test the GPIO pins using the libgpiod library.
Use gpiodetect to list all gpiochips available on the system and identify it the way I used for the hlk7621 dev board.
use gpioinfo to list all lines of the identified gpiochip, verifying that the pins 17, 27, 19, and 28 are correctly configured and available.
Further testing can be done using gpioset and gpioget to manually set and read the state of these GPIO lines, to ensure they behave as expected for UART transmission and reception.

Roadmap : 
    - add RT patch support for better accuracy