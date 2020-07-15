# raymarine-evo-pilot-remote
## Libraries used:
https://github.com/ttlappalainen/NMEA2000

https://github.com/ttlappalainen/NMEA2000_mcp

https://github.com/Seeed-Studio/CAN_BUS_Shield

## Hardware:
Maple Mini (STM32F103) / stm32duino

MCP2551 CAN-BUS Module

2 Channel Logic Level Converter (https://eckstein-shop.de/SparkFun-Logic-Level-Converter-Bi-Directiona-Shifter-Pegelwandler)


## Commands via tty

s: Standby

a: Auto (course) mode

w: Wind Vane mode

t: Track mode

+: Plus 10 degrees

-: Minus 10 degrees

c: Confirm Waypoint advance

1: Key +1

2: Key +10

3: Key -1

4: Key -10

5: Keys -1 and -10 pressed simultaneously

6: Keys +1 and +10 pressed simultaneously

## Board

![board](https://raw.githubusercontent.com/matztam/raymarine-evo-pilot-remote/master/Board/NMEA2000_USB_topview.png)
