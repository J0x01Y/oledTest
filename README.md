# OLED multi language char display project
Hardware NUCLEO-STM32F401RE
* SPI2
    * NSS `PB12`
    * SCK `PB13`
    * MOSI `PB15`
* GPIO
    * OLED_REST `PB1`
    * Data/Cmd `PB2`

Firware
* STM32Cube
    ```
    .
    ├── Core
    ├── Drivers
    ├── Makefile
    ├── STM32F401RETx_FLASH.ld
    ├── font
    ├── oledtest.ioc
    └── startup_stm32f401xe.s
    ```
* u8g2
    ```
    ./u8g2/
    ├── CMakeLists.txt
    ├── ChangeLog
    ├── LICENSE
    ├── README.md
    ├── SConscript
    ├── component.mk
    ├── cppsrc
    ├── csrc
    ├── doc
    ├── pkg
    ├── sys
    └── tools
    ```
* auto gen u8g2 font
    ```
    ./font
    ├── Makefile
    ├── NotoSansTC-Thin.bdf
    ├── NotoSansTC-Thin.ttf
    ├── bdf.tga
    ├── collectUtf2map.py
    ├── u8g2_font_zh_tw.png
    └── zh-tw.map
    ```
