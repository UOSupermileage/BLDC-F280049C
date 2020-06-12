# Motor Controller Program for TI F280049C

BLDC motor controller code for use on **TI LaunchXL-F280049C** board. To contribute to this project you must have *Code Composer Studio* and the *C2000Ware* drivers installed on your system. This project **cannot be modified with Energia**.

## Tools Used for Development
- Code Composer Studio IDE
- C2000Ware for device drivers and example projects
- MotorControl SDK
- MathWorks Embedded Target Support (Optional)

## TI TMS320F280049C CPU Information
- 100 MHz C28x CPU with FPU and TMU, 256 KB Flash, InstaSPIN-FOC enabled, 3x 12-bit ADC, CAN, encoder, FSI, UART, etc
- Power domain isolation for real-time debug  and flash programming
- Isolated CAN transceiver, Two encoder interface connectors, FSI interface connector
- Hardware Files are in C2000Ware at boards\LaunchPads\LAUNCHXL_F280049C

## Links and Resources
- [LAUNCHXL-F280049C board website](https://www.ti.com/tool/LAUNCHXL-F280049C)
- [(pdf) CPU Instruction Set (Chip Architecture)](https://www.ti.com/lit/pdf/spru430)
- [(pdf) CPU Technical Reference Manual](https://www.ti.com/lit/pdf/sprui33)
