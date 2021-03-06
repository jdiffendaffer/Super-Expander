This project is for a RAM, sound, Flash, IDE, and programmable timer interrupt board for the VZ200 8 bit computer.
The project is on hold since my VZ200 burned up in a fire.

The Verilog code provides address decoding, and chip selects for all external chips.
It was designed to perform simple page flipping of RAM, and Flash memory.
Besides memory chips, there are chip selects for an SN (Signetics) sound chip, Yamaha AY compatible sound chip, a dual channel DAC, and a simple 8 bit IDE interface.
A programmable interrupt timer based on the CoCo 3 timer is also included, but with modifications to change it from big to little endian.

The final board was to include a control signal to disable the VZ's internal memory mapping
via an unused signal on the expansion buss, and an internal mod to the computer's address decoding circuit.
This was so custom ROMs could be loaded on the FLASH memory, and RAM could replace ROM
for other operating systems like CP/M, or Fuzix. 

Things to do:
Double check the chip select outputs for active high/low required by the chips.
Fit it into a small CPLD.  This may require multiplexing the chip selects.
Verify the Z80 buss logic is correct.
Check the IDE functionality.  That was probably a last minute addition, so I'm not certain what state it's in.
Clock signals.  I hadn't decided whether to pull the timer clock from the system buss, or from an on board clock.
Timing would change between PAL & NTSC, so I was leaning towards using an on board clock.
The final timer design would have to be adjusted accordingly.
Circuit board layout.  I planned to fit the board in an existing RAM expansion box, or 3D printed lookalike.
#   S u p e r - E x p a n d e r  
 #   S u p e r - E x p a n d e r  
 