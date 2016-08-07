# mbed-lora-sx1276-per
Semtech LoRa SX1276 PER Test based on SX1276PingPong (https://developer.mbed.org/teams/Semtech/code/SX1276PingPong/)

## How to Build
```
$ mbed config root .
$ mbed deploy
$ mbed target NUCLEO_L152RE
$ mbed toolchain GCC_ARM
$ mbed compile
```

Copy .build/NUCLEO_L152RE/GCC_ARM/sx1276-per.bin to your mbed target board by USB disk
