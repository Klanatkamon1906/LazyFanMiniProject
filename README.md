# LazyFan FRA421Project
apply from LCD-Example by AlphaP\
\
using STM32H745 Dual core

## การต่อสายของระบบ
### LCD module
#### Power
VCC   --> 3.3V\
GND   --> GND\
LED   --> 3.3V
#### Data
SCK   --> D13\
SDA   --> D11\
CS    --> D10\
A0    --> D9\
RESET --> D8 

### HC-35 board
#### Power
VCC --> 3.3 V\
GND --> Gnd
#### Button
S1 --> PF8\
S2 --> PF7\
S3 --> PF9
#### LED
D1 --> PD6\
D3 --> PD5\
D5 --> PD4\
D7 --> PD3
