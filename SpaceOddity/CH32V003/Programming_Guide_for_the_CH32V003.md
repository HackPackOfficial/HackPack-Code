#  Programming Guide for the CH32V003 on main PCB of Space Oddity

##  Summary:

This PCB has a WCH CH32V003 microcontroller on it that handles the front panel buttons and manages the power distribution for the rest of the system. I added plated holes for pin headers on the top left of the PCB that are specifically for programming this PCB. I designed them for pin headers for prototyping, but you can use pogo pins or another temporary solution for connecting to them for programming. Programming this board requires the use of a WCH LinkE programmer module, which converts USB serial into the custom one-wire programming protocol needed by the CH32V003. I have provided the PlatformIO project for the stock behavior of this microcontroller.

The CH32V003 is always on, as long a USB cable is plugged into the PCB. However, it usually only uses a few microamps because it spends most of its time in deep sleep mode. The power button on the main PCB is debounced in hardware (RC filter -> Schmitt trigger inverter -> GPIO on the CH32V003). When the button is pressed, this triggers an interrupt that wakes the microcontroller from deep sleep. The microcontroller then enables the high side power switch that turns on power to the rest of the circuit, which boots the ESP32-C3. The first thing the ESP32 does is establish a UART connection to the CH32V003, and then they execute a simple custom protocol I designed that lets them pass a few simple messages back and forth. I designed a custom message protocol because it was fun, not because it was the best way to do things. The CH32V003 monitors the power button and the front panel buttons and reports their state to the ESP32. The ESP32 can send a shutdown command that causes the CH32 to disable the high side power switch.

This is the first time that a Hack Pack has used a second microcontroller as a coprocessor or peripheral, but it isn't the last! I already have one more in development that actually uses two CH32V003 microcontrollers as peripherals, and in that one, one of the CH32V003s does a lot more of the heavy computational lifting, functions as a GPIO expander, and presents itself as a peripheral on the I2C bus. Get excited, because I sure am!

##  Required tools:

-  WCH LinkE debugger/programmer ([this is a link](https://www.wch-ic.com/downloads/WCH-LinkUserManual_PDF.html) to find the manual for using the WCH Link programmers. There are several models, the one I have been using is the LinkE).

-  Software for uploading the firmware. There are quite a few options available for this, and any of these would work:

-  Recommended: [CH32V Development Platform for PlatformIO](https://github.com/Community-PIO-CH32V/platform-ch32v) (also requires PlatformIO and an editor that works with PlatformIO, like Visual Studio Code or CLion).

-  Alternative: [MounRiver Studio](https://www.mounriver.com/download) from WCH, the maker of the CH32V003.

-  Hardcore mode: Write your code in something like Neovim or Spacemacs, build it with something like [ch32fun](https://github.com/cnlohr/ch32fun), and flash the code with something like ch32fun or [wchisp](https://github.com/ch32-rs/wchisp). But if this is the approach you're planning to take, I don't think you need me to tell you what tools to use. You tell me what tools you're using (seriously, this is Evan from Crunchlabs; hit me up on our Discord and let me know what tools you're using)!

##  How to program the board:

1.​ Connect the WCH LinkE programmer to the programming connection points on the PCB. Only 3 connections are required: GND, +3.3V, and SWDIO. The programming connection points are in the upper left corner of the board, and if you look at the bottom of the PCB, the points are labelled on the silk screen.​ ​

> **Note:** Tx and Rx are not required for programming. There are broken out as affordances for debugging or hacking. For example, you can connect a USB to Serial converter module to Tx and Rx and send and receive data over the Serial monitor.  

​2.​ Use the chosen software (e.g. PlatformIO) to flash the code onto the microcontroller.

##  Testing and verifying that the code was uploaded correctly:

 
1.​ Provide power to the PCB, either by leaving it connected to the WCH LinkE programming jig, or by plugging a USB cable into the receptacle.

2.​ Press the power button on the left side of the PCB. A red power indicator LED below the power button should immediately turn on.

3.​ Press and hold the power button for 5 seconds. After 5 seconds, the red power indicator LED should turn off.

4.​ Press and release the power button again, and the power LED should immediately turn on again. If all of these steps are successful, then the correct code was uploaded to the board and it's now functioning correctly.
  

![p2-1](https://ide-media.crunchlabs.com/media/img_002_01_f1c9a3f5ad.png)
![p2-2](https://ide-media.crunchlabs.com/media/img_002_02_40ad3abbee.png)
![p3-1](https://ide-media.crunchlabs.com/media/img_003_01_640b0abd1d.png)
![p4-1](https://ide-media.crunchlabs.com/media/img_004_01_0f07a98dbc.png)
![p4-2](https://ide-media.crunchlabs.com/media/img_004_02_eceed17d8c.png)
