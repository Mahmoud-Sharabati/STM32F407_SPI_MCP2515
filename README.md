# STM32F407_SPI_MCP2515
## This repository contains a driver for the MPC2515 CAN controller, designed to interface with the STM32f407VG microcontroller. The driver is based on the MPC2515 driver by [Daniel Rossi](https://github.com/ProjectoOfficial/STM32/tree/main/STM32_MCP2515).

## Requirements
* A device with an STM32f407VG microcontroller.
* A device with an MPC2515 CAN controller.
* The STM32CubeMX software, available [here](https://www.st.com/en/development-tools/stm32cubemx.html).

## Installation
1. Clone the repository:
git clone [here](https://github.com/Mahmoud-Sharabati/STM32F407_SPI_MCP2515.git).
2. Import the project into STM32CubeMX by selecting File > Import > C/C++ > Existing Code as Makefile Project.
3. Configure the project in STM32CubeMX according to your hardware setup.
4. Generate the code and open the project in your preferred development environment.
5. Build and upload the code to your device.

## Usage

### Before using SPI CAN Driver
1. Modify the Hardware Configuration and Pins using CubeMX software
    This driver use SPI3 with the following pins:
    | SPI3_CS       | SPI3_SCK    | SPI3_MISO   |SPI3_MOSI   |RX0B_Pin
    | ------------- |:-----------:|:-----------:|:----------:| -----------|
    | PC7	    | PC10	  | PC11        | PC12	     |PC9	  |

																		
	*This driver use PC9 pin as RX0B interrupt pin (Named: RX0B_Interrupt)*

2. This Board has an optional LED and Buzzer with the following pins:
   | LED	| Buzzer  |
   |------------|---------|
   | PC5	| PA0	  |

 ### Use CAN SPI Driver instruction
1. Initialize the CAN SPI driver using: CANSPI_Initialize(uint8_t BitRate)

	Two Bit Rate options:
   | 500Kbps	| 250Kbps  |
   |------------|--------- |
   | _500KBPS	| _250KBPS |
       
2. Enable/Disable RX pins Interrupt (on RX0B and RX1B pins)

	Two available options:
   | Enable Interrupts on RX0B and RX1B pins	| Disable Interrupts on RX0B and RX1B pins  |
   |--------------------------------------------|-------------------------------------------|
   | _Enable					| _Disable	  			    |

_NOTE: When using the RX interrupts, the interrupt flag MUST be cleared by software using CANSPI_RXB0_CLR Function_
 
3. Use Transmission and Receive functions:\
Transmission Function:\
`CAN_AddTxMessage(CAN_HeaderTypeDef *tempCanMsg)`\
Receive Function:\
`CAN_GetRxMessage(CAN_HeaderTypeDef *tempCanMsg)`

## Credits
This driver is based on the work of Daniel Rossi, whose original driver can be found [here](https://github.com/ProjectoOfficial/STM32/tree/main/STM32_MCP2515).

## License
This driver is licensed under the MIT License. See the LICENSE file for details.
