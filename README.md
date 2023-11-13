# softwareserial-linux


This project has been adapted from Adrianomarto's original work on soft UART for the Raspberry Pi, available at https://github.com/adrianomarto/soft_uart. 

A huge acknowledgment to him for his exceptional contribution. This version extends his work into a more generalized module, It uses device tree to create multiple instances of softserial ports.

use this dts section to map each pin to each module,  
	soft_uart {
		compatible = "soft-uart";

		uart0 {
			compatible = "akasoft,soft-uart";
			tx-pin = <17>;  // GPIO pin number for TX
			rx-pin = <27>;  // GPIO pin number for RX
		};

		uart1 {
			compatible = "akasoft,soft-uart";
			tx-pin = <19>;
			rx-pin = <28>;
		};
	}

Roadmap : 
    - add RT patch support for better accuracy