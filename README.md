# FPGAComputer32
This is a 32-bit computer implemented in the DE0-NANO FPGA.

The computer has 32-bit CPU, 40KB static RAM, 32MB dynamic RAM, UART (115200 bps), VGA video (640x480, text-based frame buffer, 80x60 characters, or the graphics mode of 320x240 pixels, each one in one of 8 colors, with 16 hardware sprites, or 640x480 pixels, two colors), and PS/2 keyboard support.

The 32-bit CPU has 16 general-purpose registers (r0 - r15), pc (program counter), sp (stack pointer - r15 register), ir (instruction register), mbr (memory buffer register), h (higher word when multiplying, or remainder when dividing - r14 register).

The address bus is 24 bits wide, addressing 32MB. Data bus is 16 bits wide, but all the addresses are 8-bit aligned, meaning that two bytes are fetched with one memory access (using even addresses).

Video output is VGA, 640x480. Text mode has 80x60 characters, each character being 8x8  pixels in dimensions. Video frame buffer in text mode has 4800 16-bit words (80x60 characters), starting at 1024 decimal. The lower byte has the ASCII character, while the upper byte has the attributes (3 bits for the background color, 3 bits for the foreground color, inverted, and the two bits unused). 

In graphics mode 1, the resolution is 320x240 pixels. Each pixel is 4 bits long, having two pixels per byte in the frame buffer. Frame buffer starts at 1024 decimal. Each pixel's color is defined by those four bits by: xrgb.

In graphics mode 2, the resolution is 640x480 pixels. Each pixel is 1 bit long, having value 1 (white) or 0 (black). Therefore, one byte of the frame buffer holds 8 pixels. Frame buffer starts at 1024 decimal. 

This computer has three interrupts: IRQ0, IRQ1 and IRQ2. IRQ0 is connected to the internal timer, which is incremented every millisecond. The IRQ1 is connected to the UART, while IRQ2 is connected to the PS/2 keyboard. Whenever a byte comes to the UART, it generates an IRQ1. Whenever a PS/2 key is pressed, couple of bytes are received (make and break codes), in a sequence, each causing the IRQ2 to fire.

The interrupt causes the CPU to push flags to the stack, then to push PC to the stack and then to jump to the location designated for the CPU:
* for the IRQ0, it is 0x0008,
* for the IRQ1, it is 0x0010, and
* for the IRQ2, it is 0x0018.

It is up to the programmer to put the code in those locations. Usually, it is a JUMP instruction. To return from the interrupt routine, it is necessary to put the IRET instruction. It pops the return address, and then pops the flags register, and then goes back into the interrupted program.

KEY0 of the DE0-NANO is used as the reset key. When pressed, it forces CPU to go to the 0x0000 address. Usually there is a JUMP instruction to go to the main program.

# VGA text mode
Text mode is 80x60 characters, occupying 4800 words. Lower byte is the ASCII code of a character, while the upper byte is the attributes. Video frame buffer starts at the address 1024 (decimal).

The foreground color is inverted so zero values (default) would mean white color. That way, you don't need to set the foreground color to white, and by default (0, 0, 0), it is white. The default background color is black (0, 0, 0). This means that if the upper (Attribute) byte is zero (0x00), the background color is black, and the foreground color is white.

VGA female connector is connected via resistors to the GPIO-0 expansion header of the DE0-NANO board:

* GPIO_R (pin 2, GPIO_00, PIN_A3) -> 68Ohm -> VGA_R,
* GPIO_G (pin 4, GPIO_01, PIN_C3) -> 68Ohm -> VGA_G,
* GPIO_B (pin 6, GPIO_03, PIN_D3) -> 68Ohm -> VGA_B,
* GPIO_HS (pin 8, GPIO_05, PIN_B4) -> 470Ohm -> VGA_HORIZONTAL_SYNC,
* GPIO_VS (pin 10, GPIO_07, PIN_B5) -> 470Ohm -> VGA_VERTICAL_SYNC.

# VGA graphics mode 8 colors
This graphics mode is 320x240 pixels. Since the text mode is the default mode, to switch to this graphics mode, you need to type in the assembler code following:

```
mov.w r0, 1
out [128], r0
```

To switch back to the text mode, you need to enter:

```
mov.w r0, 0
out [128], r0
```

Video memory starts from address 1024. One byte of the video memory is organised like this:

xrgbxrgb

One byte holds two pixels. The x bit is unused, and the other bits define red, green and blue component of the color for each of those two pixels. Video frame buffer starts at the address 1024 (decimal). So, if you want to put four white pixels at the top left corner of the screen (from the (0,0) to the (3,0) coordinates), you need to type:

```
mov.w r0, 0x7777
st.s [1024], r0
```

# VGA graphics mode two colors
This graphics mode is 640x480 pixels. Since the text mode is the default mode, to switch to this graphics mode, you need to type in the assembler code following:

```
mov.w r0, 2
out [128], r0
```

To switch back to the text mode, you need to enter:

```
mov.w r0, 0
out [128], r0
```

Video memory starts from address 1024. One byte of the video memory holds 8 pixels, each being 1 or 0. If you want to put four white and four black pixels at the top left corner of the screen (from the (0,0) to the (7,0) coordinates), you need to type:

```
mov.w r0, 0xF0
st.b [1024], r0
```

## Hardware Sprites
The computer supports up to 16 hardware sprites, each being 16x16 pixels, in graphics mode 1 (320x240). Each sprite is defined by the 8-byte structure:
* sprite definition address (2 bytes); must be below 0xB000 (at 0xB000 SDRAM starts, and sprites cannot be loaded from the SDRAM - only from Static RAM)
* x coordinate (2 bytes)
* y coordinate (2 bytes)
* transparent color (2 bytes).

The sprite structure for the first sprite starts at 64 decimal. Each next sprite structure starts 8 bytes later. 

Sprite definition consists of 16 lines, each line described by 16 pixels, each pixel defined by 4 bits: xrgb.
This means that one sprite line consists of 8 bytes (two pixels per byte), so total bytes needed for the sprite definition is 8x16 bytes.

Here is the example of showing one sprite at (25, 25):

```
  mov.w r0, sprite_def
  mov.w r1, 56
  st.s [r1], r0  ; sprite definition is at sprite_def address
  mov.w r0, 25
  st.s [r1 + 2], r0  ; x = 25  at addr 58
  mov.w r0, 25
  st.s [r1 + 4], r0  ; y = 25  at addr 60
  mov.w r0, 0
  st.s [r1 + 6], r0  ; transparent color is black (0) at addr 62
  ; sprite definition
sprite_def:
  #d16 0x0000, 0x0000, 0x0000, 0x0000  ; 0
  #d16 0x0000, 0x000f, 0xf000, 0x0000  ; 1
  #d16 0x0000, 0x000f, 0xf000, 0x0000  ; 2
  #d16 0x0000, 0x000f, 0xf000, 0x0000  ; 3
  #d16 0x0000, 0x004f, 0xf400, 0x0000  ; 4
  #d16 0x0000, 0x004f, 0xf400, 0x0000  ; 5
  #d16 0x0000, 0x044f, 0xf440, 0x0000  ; 6
  #d16 0x0000, 0x444f, 0xf444, 0x0000  ; 7
  #d16 0x0004, 0x444f, 0xf444, 0x4000  ; 8
  #d16 0x0044, 0x444f, 0xf444, 0x4400  ; 9
  #d16 0x0400, 0x004f, 0xf400, 0x0040  ; 10
  #d16 0x0000, 0x004f, 0xf400, 0x0000  ; 11
  #d16 0x0000, 0x004f, 0xf400, 0x0000  ; 12
  #d16 0x0000, 0x041f, 0xf140, 0x0000  ; 13
  #d16 0x0000, 0x4111, 0x1114, 0x0000  ; 14
  #d16 0x0004, 0x4444, 0x4444, 0x4000  ; 15
```

# UART interface

UART interface provides TTL serial communication on 115200kbps. It uses one start bit, one stop bit, and eight data bits, no parity, no handshake.

UART is connected to the GPIO-0 expansion header of the DE0-NANO board:

* TX (pin 32, GPIO_025, PIN_D9) should be connected to the RX pin of the PC,
* RX (pin 34, GPIO_027, PIN_E10) should be connected to the TX pin of the PC.

UART is used within the CPU via IN, and OUT instructions. RX also triggers the IRQ1, which means that whenever a byte is received via UART, the IRQ1 will be triggered, forcing CPU to jump to the 0x0008 address. There you should place the JUMP instruction to your UART interrupt routine.

Inside the UART interrupt routine, you can get the received byte by using the IN instruction:

```
in r1, [64]; r1 holds now received byte from the UART
```

To send a byte, first you need to check if the UART TX is free. You can do it by using the in instruction:

```
loop:
      in r5, [65]   ; tx busy in r5
      cmp.w r5, 0    
      jz not_busy   ; if not busy, send back the received character
      j loop
not_busy:
      out [66], r1  ; send the character to the UART
```

Addresses used by the UART are in the following list:
* 64 -> Received byte from the RX part of the UART (use the IN instruction).
* 65 -> 0 if the TX part of the UART is free to send a byte, 1 if TX part is busy.
* 66 -> Byte to be sent must be placed here using the OUT instruction.

# PS/2 interface

PS/2 interface works with PS/2 keyboards. Just connect the keyboard to the PS/2 connector, and the IRQ2 will be fired for each byte of the make/break sequence. 

PS/2 connector is connected to the GPIO ports of the DE0-NANO board:
* Data is connected to the GPIO31 (PIN_D11) port
* Clock is connected to the GPIO33 (PIN_B12) port.

To read the received make/break code, you need to use the IN instruction:

```
in r1, [68]; r1 holds the received byte from the PS/2 keyboard
```
You need to make additional code to parse make/break code. 

