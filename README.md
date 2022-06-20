# CheckLoader
CheckLoader is a bootloader for stm32f407vet6, it uses spi for display, and usb for serial  
i am using a tft ili9341, the conection is exact as i describet in my lybrary, with exception of led, wich is connected to PA6  
3V VCC

GND GND

PA4 CS

PA2 RESET

PA3 DC

PA7 SDI (MOSI)

PA5 SCK

PA6 LED

to send files by serial i used realterm  
after you send data, you should see something on display send back by controller, that means that everything works fine 😃
