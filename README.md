# spicy_beef
STM32U575 - 240x240 TFT LCD - ST7789 - LVGL

## Wiring Diagram

![Screenshot 2024-06-07 at 2 12 47 PM](https://github.com/burke-david/spicy_beef/assets/153735498/9b0a2f94-640e-4960-b557-68f167956420)

## Pin Descriptions

![Screenshot 2024-06-07 at 2 11 37 PM](https://github.com/burke-david/spicy_beef/assets/153735498/47eeb2c4-fb12-41d1-9c86-e8705045bbb8)

## Initial Goals

Include the following features:
* [NUCLEO-U575ZI-Q](https://www.st.com/en/evaluation-tools/nucleo-u575zi-q.html)
* [LVGL](https://github.com/lvgl/lvgl) graphics library
* [240x240 LCD display with ST7789 driver](https://www.adafruit.com/product/4313)
* UART debug/control
* UART CLI
* User Button
* LED control
* Bare metal
* Build/Debug/Develop environment using VS Code
* CI Scripts / Github workflow integration
* Unit Testing (ceedling)

## Stretch Goals
* Integrate file system library and add ability to read/write files from the uSD card on the LCD board.

## References

* [STM32U575 Datasheet](https://www.st.com/resource/en/datasheet/stm32u575ag.pdf)
* [NUCLEO-U575I-Q Schematic](https://www.st.com/resource/en/schematic_pack/mb1549-u575ziq-c03_schematic.pdf)
* [STM32U5 Nucleo-144 board User Manual](https://www.st.com/resource/en/datasheet/stm32u575ag.pdf)
* [Adafruit 1.3" and 1.54" 240x240 Wide Angle TFT LCD Displays Pinouts](https://learn.adafruit.com/adafruit-1-3-and-1-54-240-x-240-wide-angle-tft-lcd-displays/pinouts)
* 
