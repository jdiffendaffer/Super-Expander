////////////////////////////////////////////////////////////////////////////////
// Company:
// Engineer:	James Diffendaffer
//				Copyright(c) 2015 
// Create Date:    April 8, 2015.
// Design Name:
// Module Name:    .
// Project Name:   VZ SuperExpander  
// Target Device: 
// Tool versions: 
// Description:
//
//      VTec Super Expansion, 3 sound chips + stereo DAC - (SN76489N) (YMZ284 AY compatible) (YM2413) (AD7528)
//
//      Inputs:
//         Dip switch - 2 switches, select VZ's amount of built in RAM
//         Address lines from Z80
//         Data lines from Z80
//         *IORQ - from Z80
//         /RD - from Z80
//         /WR - from Z80
//         /RESET from Z80
//      Outputs:
//         /SNCS - Chip select for SN sound chip
//         /AYCS - Chip select for AY sound chip
//         /YMCS - Chip select for YM OPL sound chip
//         /DACS - Chip select for Stereo DAC Chip 
//         /IDE__SEL - IDE I/O Port region enable
//         A14-A18     (5 upper address lines for RAM)
//         RAM /CS     (A0-A15 from Z80 tied to A0-A13 on RAM and CPLD, D0-D7 tied to DQ0-DQ7, *MREQ tied to /OE, *OUT tied to /WE)
//         FLASH CE#   (A0-A15 tied to A0-15, D0-D7 tied to I/O0-7, OE# tied to *IN, WE# tied to *OUT)
//      Registers
//         Memory Page - Port $7F
//            5 bits, selects the 16K RAM page to appear at the top of RAM.  Enough lines to page remainder of 512K RAM
//         Memory map select - Port $7E
//            Controls internal memory map disable line and chip select lines
//                Bit 1-0
//                  00 ROM in bottom 16K
//	                01 FLASH in bottom 16K
//	                10 Same as 11
//	                11 Expansion RAM in bottom 16K
//                Bit 2
//                  0 FLASH in 12K expansion area
//                  1 Expansion RAM in 12K expansion area
//                Bit 3
//                  0 Memory Mapped I/O
//                  1 Expansion RAM
//                Bit 4
//                  0 Video RAM
//                  1 Expansion RAM
//                Bit 5
//                  0 Built in RAM enabled
//                  1 Expansion RAM enabled
//                Bit 6 only set according to dip switch on startup, not program selectable
//                  0 Built in memory can't be disabled
//                  1 Built in memory can be disabled
//      Decodes default RAM config (dip switch) on startup for different default amounts of memory these machines came with.
//         No built in RAM (so defective internal RAM can be removed)  b00: $7800 - $FFFF
//         Laser 200  2K  b01:  $7800 - $7FFF
//         Laser/VZ 200 210  4K  b10:  $7800 - $8FFF
//         Laser VZ 300 310  16K  b11:  $7800 - $B7FF
//      Pages top 16K of 64K address space within 512K ROM.  $C000 - $FFFF
//         If A14 OR A15 is set, we are in page RAM.
//         Page IO port is at address $7F
//         5 page bits held by latch determine page, output to RAM when in page area.  Output b00000 to upper RAM address lines when not.
//
// Dependencies:
//
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
//
////////////////////////////////////////////////////////////////////////////////



module VZSuperExpander (


/////////////////////////////////////////////////////
// I/O lines and register definitions
/////////////////////////////////////////////////////

input RESET;				// load defaults on reset

// Memory Address and Data buss
input [15:0] ADDRESS;		// Address buss input
input [7:0] DATA;			// Data buss input

// Memory control
input [1:0] RAM_CONFIG;		// Built in RAM dip switch Configuration 2K, 4K, 16K
output RAM_CS;				// RAM Chip Select
output FLASH_CS;			// FLASH Chip Select


reg [4:0] RAM_PAGE_REG;		// RAM Page select register
////////// is this ok or do I need to use output [4:0] RAM_HIGH ????
output [18:14] RAM_HIGH;	// Based on RAM page select and location in memory, address lines A14-A18 on 512K RAM chip

input DECODE_DEFEAT;		// Dip switch tells us if the computer supports disabling internal memory map decoding
reg VZ_DECODE_REG;			// VZ memory mapping enable/disable register
output VZ_DECODE_SEL;		// Disables/enables VZ internal memory map decoding


// sound chip control
output SN_CS;				// SN76489A sound chip select
output AY_CS;				// AY - YMZ284 sound chip select
output YM_CS;				// YM2413 FM sound chip select
output DAC_CS;				// AD7528 Stereo DAC chip select

// IDE interface control
output IDE_SEL;				// IDE Port Select 0 or 1
//output IDE_DRIVE_SELECT;		// IDE Drive # Select
//output IDE_PORT_REG;		// IDE control port number to access

// Z80 IO Control lines
input WR_EN;				// Write enable  /WR
input RD_EN;				// Read enable /RD
input IORQ;					// IO memory request  /IORQ

// IO read and write control
output WRITE;				// /WR AND /IORQ    rename to IORD
output READ;				// /RD AND /IORQ    rename to IOWR

// internal signal indicating non port I/O
wire MEM_REQ;				// Memory access request /MEM_REQ




/////////////////////////////////////////////////////
// initial setup
/////////////////////////////////////////////////////
initial
begin
	VZ_DECODE_REG <= 1'b1;			// clear the internal memory mapping disable control register
	RAM_PAGE_REG <= 5'b00000;		// clear the RAM bank select register
	MEM_REQ <= 1'b1;				// clear the MEMory REQest line
	VZ_DECODE_SEL <= 1'b1;			// Regular Memory Map
	IDE_SEL <= 1'b1;				// IDE Disabled
//	IDE_DRIVE_SELECT <= 1'b0		// IDE Drive 0 is default	
	SN_CS <= 1'b1;					// disable the SN chip select
	AY_CS <= 1'b1;					// disable the AY chip select
	YM_CS <= 1'b1;					// disable the YM chip select
	DAC_CS <= 1'b1;					// disable the DAC chip select
	RAM_CS <= 1'b1;					// Disable RAM
	FLASH_CS <= 1'b1;				// Disable FLASH
end


/////////////////////////////////////////////////////
// handle reset (same as initial)
/////////////////////////////////////////////////////
always @(negedge RESET)
begin
	VZ_DECODE_REG <= 1'b1;			// clear the internal memory mapping disable control register
	RAM_PAGE_REG <= 5'b00000;		// clear the RAM bank select register
	VZ_DECODE_SEL <= 1'b1;			// Regular Memory Map
	IDE_SEL <= 1'b1;				// IDE Disabled
//	IDE_DRIVE_SELECT <= 1'b0		// IDE Drive 0 is default	
	SN_CS <= 1'b1;					// disable the SN chip select
	AY_CS <= 1'b1;					// disable the AY chip select
	YM_CS <= 1'b1;					// disable the YM chip select
	DAC_CS <= 1'b1;					// disable the DAC chip select
	RAM_CS <= 1'b1;					// Disable RAM
	FLASH_CS <= 1'b1;				// Disable FLASH
end


/////////////////////////////////////////////////////
// Common behavior for I/O Requests
// Generate read and write lines for IO ports
// Enable the regular memory map
// Disable Expansion RAM
// Disable FLASH
/////////////////////////////////////////////////////
always @(IORQ)
begin
	WRITE <= ~WR_EN ~& ~IORQ;	// AND the signals together so /WR is only active when /IORQ is active
	READ <= ~RD_EN ~& ~IORQ;	// AND the signals together so /RD is only active when /IORQ is active
	
	VZ_DECODE_SEL <= 1'b1);		// Enable Regular Memory Map
	RAM_CS <= 1'b1;				// Disable RAM
	FLASH_CS <= 1'b1;			// Disable FLASH
end


/////////////////////////////////////////////////////
// Generate MEM_REQ signal
// Do I use blocking or non-blocking?
// Valid always statement?
/////////////////////////////////////////////////////
always @((WR_EN or RE_EN) and ~IORQ)
begin
	if((WR_EN == 1'b0 or RE_EN == 1'b0) and IORQ == 1'b1)
		MEM_REQ <= 1'b0;
	else
		MEM_REQ <= 1'b1;
	end
end

/////////////////////////////////////////////////////
// Memory Mapping I/O Ports
/////////////////////////////////////////////////////


/////////////////////////////////////////////////////
// Sound chip address decoding.  $A0-$A7
// Generate the chip select signals based on the memory address and IOREQ signal from the Z80
// added negedge but didn't remove if(IOREQ = 0) yet.  Should I?
/////////////////////////////////////////////////////
always @(IOREQ)
begin
	// check to see if there is an IO REQest
	if(IOREQ == 0)
		// check to see if we are in I/O block with sound chips 
		//  I did this to make it easy to relocate the entire group all at once
		if(address[7:4] == 4'hA)	// I/O port $A#
			// check address for each sound chip
			case (address[3:1])
				3'b000:		// Stereo DAC ports $#0-$#1
					DAC_CS <= 1'b0;						//  enable the DAC chip select if so
				3'b001:		// SN I/O port $#2-$#3 (only requires $#2 but $#3 is reserved for status read if I add it)
					if(address[0] == 1'b0)				// is it a write?
						SN_CS <= 1'b0;					//  enable the SN chip select if so
					else								// must be a read
						// data[7] == SN_STATUS;		//  enable status read
					end
				3'b010:		// AY I/O ports $#4-$#5
					AY_CS <= 1'b0;					//  enable the AY chip select if so
				3'b011:		// YM I/O ports $#6-$#7
					YM_CS <= 1'b0;					//  enable the YM chip select if so
			endcase
		end
	else
		// Make sure all sound chips are disabled when IOREQ is inactive
		SN_CS <= 1'b1;						// disable the SN chip select
		AY_CS <= 1'b1;						// disable the AY chip select
		YM_CS <= 1'b1;						// disable the YM chip select
		DAC_CS <= 1'b1;						// disable the DAC chip select
	end
end


/////////////////////////////////////////////////////
// Set IO Disable register $AE
/////////////////////////////////////////////////////
always @(negedge IOREQ)
begin
	if (address[7:0] == 8'hAE)		// port $AE
		DECODE_DEFEAT <= DATA[0];
	end
end

/////////////////////////////////////////////////////
// Set RAM Page control register $AF
/////////////////////////////////////////////////////
always @(negedge IOREQ)
begin
	if (address[7:0] == 8'hAF)		// port $AF
		RAM_PAGE_REG <= DATA[4:0];			// RAM Page select register
	end
end


/////////////////////////////////////////////////////
// IDE Port %B0-$BF
/////////////////////////////////////////////////////
always @(IOREQ)
begin
	if (address[7:4] == 4'hB)
		//select IDE port range (16 in all address)
		IDE_SEL <= 1'b0;	// IDE Enabled
		// set for drive number 0 or 1
//		if (address[3:2] == 2'b00)
//			IDE_DRIVE_SELECT <= 1'b0
//		else
//			IDE_DRIVE_SELECT <= 1'b1
//		end
	else
		IDE_SEL <= 1'b1;	// IDE Disabled
	end
	
//	// Implement IDE as an external 8 bit latch to set the IDE port (address)
//	// followed by the port access   $AC-$AD
//  reg IDE_DATA_PORT;
//	if (address[7:4] == 4'hA)
//		case(address[3:0])
//			4'hC:   
//				IDE_REG_SEL <= 1'b0;		// Enable IDE Register Select
//				IDE_PORT_REG <= DATA;		// Set the port number to access
//				if(IDE_PORT_REG > 7)
//					IDE_DRIVE_SELECT <= 1'b1;	// set drive select for drive 1
//				else
//					IDE_DRIVE_SELECT <= 1'b0;	// set drive select for drive 0
//				end
//			4'hD:
//				IDE_SEL <= 1'b0;			// IDE R/W port
//				IDE_DATA_PORT <= DATA;		// Write data to the port
//		endcase
//	else
//			IDE_SEL <= 1'b1;			// Disable IDE port
//			IDE_REG_SEL <= 1'b1;		// Disable IDE Register Select
//	end
end


/////////////////////////////////////////////////////
// Memory Maping non-I/O
/////////////////////////////////////////////////////

/////////////////////////////////////////////////////
// Generate Expansion RAM address lines A14-A18 based on page control register and address lines
// high address lines are only tied to RAM so no worry about conflict with the system address buss
/////////////////////////////////////////////////////
always @(negedge MEM_REQ)
begin
	// Check to see if we are in paged RAM which is the top 16K  $BFFF - $FFFF.
	if (ADDRESS[15:14] != 2'b00)	// check top two address lines to see if they are zero
		RAM_HIGH <= 5b'00000;		// if not, high address lines are zero
	else
		RAM_HIGH <= RAM_PAGE_REG;	// if so, high address lines are = the RAM page register
	end
end


/////////////////////////////////////////////////////
// 12 bit programmable timer interrupt
//  Clock code
//  Work in progress
//  Do I need to time activation of the interrupt with the next system clock if I use my own clock?
/////////////////////////////////////////////////////
wire TINS			// Timer Interval NanoSeconds  1 = 278.365 nSec (3.58MHz)  0 = 63.695 uSec (15.87 KHz)
wire TMR			// Timer interrupt control  1 =  Enable  0 = Disable
TIMERCTRL[1:0];		// Timer Control
output SYSINT;		// System Interrupt control line
input CLOCK_358;	// Use clock driving sound chips for consistent timing
reg TIMERCLKDIV[7:0]
reg TIMERCTRL[7:0];	// Timer control register
reg TIMER[11:0];	// 12 bit timer
reg TIMERLSB[7:0];	// LSB of programmable timer
reg TIMERMSB[11:8];	// MSB of programmable timer

always @(negedge CLOCK_358)
begin
	if(TMR == 1'b1)	// is the interrupt enabled 
		begin
			if(TINS == 1)	// 278.365 nSec (3.58MHz)
				TIMER <= TIMER - 12'h001;				// decrement the timer
				if (TIMER == 12'h000)					// has time expired?
					SYSINT <= 1'b0;						// trigger an interrupt.  Time with next Systen Clock negedge?
					TIMER <= {TIMERMSB,TIMERLSB};		// reload the timer
				end
			else		//63.695 uSec (15.87 KHz)
				// divide the clock by using a 2nd counter
				TIMERCLKDIV <= TIMERCLKDIV - 1'b1;
				if (TIMERCLKDIV == 1'b0)				// has the clock divider expired?
					TIMERCLKDIV <= 8'd225;				// reload the clock divider counter (double check divider value)
					TIMER <= TIMER - 12'h001;			// decrement the timer
					if (TIMER[11:0] == 12'h000)			// has time expired?
					begin
						SYSINT <= 1'b0;					// trigger an interrupt
						TIMER <= {TIMERMSB,TIMERLSB};	// reload the timer
					end
				end
			end
		end
//	else		// timer interrupt is disabled
	end
end


/////////////////////////////////////////////////////
// 12 bit programmable timer interrupt
//  User control interface
//  Work in progress
//  Appears in memory as follows:
//  LSB (low 8 bits of timer)
//  MSB (4 upper bits of timer)
//  Timer Control Register (bit 0 = INTerrupt enable/disable, bit 1 = Timer INterrupt status )
/////////////////////////////////////////////////////
always @(negedge IORQ)	//TIMERLSB or TIMERMSB or TIMERCTRL)
begin
	if(IORQ == 0)
		case (address[7:0])
		8'hA8:
			// Timer LSB
			if(WRITE == 0)
				TIMERLSB <= DATA;			// Load Least Significant BYTE
			end
		8'hA9:
			// Timer MSB
			if(WRITE == 0)
				TIMERMSB <= DATA[0:3];			// Load Most Significant bits of timer
				TMR == 1'b1;				//start timer here
			end
		8'hAA:
			// Timer Control
			if(WRITE == 0)
				INT <= DATA[0];
				TINS <= DATA[1];
			else // read
				DATA[0] = ~SYSINT;
				SYSINT = 1'b1
			end
		endcase;
	end
end


/////////////////////////////////////////////////////
// Memory mapping for FLASH memory and RAM
// Should I split this up?
// Generate the VZ address decoding disable/enable signal
// can I use a register for always?
//
// changes to VZ_DECODE_REG
//	Bit 1-0
//		00 ROM in bottom 16K
//		01 FLASH in bottom 16K
//		10 Same as 11
//		11 Expansion RAM in bottom 16K
//	Bit 2
//		0 FLASH in 12K expansion area
//		1 Expansion RAM in 12K expansion area
//	Bit 3
//		0 Memory Mapped I/O
//		1 Expansion RAM
//	Bit 4
//		0 Video RAM
//		1 Expansion RAM
//  Bit 6
//      0 Built in RAM enabled
//      1 Expansion RAM enabled
//  Bit 7 only set according to dip switch on startup, not program selectable
//      0 Built in memory can't be disabled
//      1 Built in memory can be disabled
/////////////////////////////////////////////////////
always @(MEM_REQ)
begin
	// only respond to memory access requests
	if(MEM_REQ == 1'b1)
		VZ_DECODE_SEL <= 1'b1;				// Regular Memory Map
		RAM_CS <= 1'b1;						// Disable RAM
		FLASH_CS <= 1'b1;					// Disable FLASH
	else
		// Find what part of the memory map we are in
		// and set up memory accordingly 
		if(ADDRESS[15:12] < 4'H4)						// $0000 - $3FFF : Built In ROM, FLASH, Expansion RAM
			// First 16K ROM/FLASH/RAM
			case (VZ_DECODE_REG[1:0])
				2'b01:		// FLASH
					VZ_DECODE_SEL <= 1'b0;				// Disable Regular Memory Map
					RAM_CS <= 1'b1;						// Disable RAM
					FLASH_CS <= 1'b0;					// Enable FLASH
				2'b10,		// Expansion RAM
				2'b11:		// Expansion RAM
//					RAM_HIGH <= 5b'00000;				// Set high address lines to zero
					VZ_DECODE_SEL <= 1'b0;				// Disable Regular Memory Map
					RAM_CS <= 1'b0;						// Enable RAM
					FLASH_CS <= 1'b1;					// Disable FLASH
			default:
				2'b00:		// Built in ROM
					VZ_DECODE_SEL <= 1'b1;				// Regular Memory Map
					RAM_CS <= 1'b1;						// Disable RAM
					FLASH_CS <= 1'b1;					// Disable FLASH
			endcase
		else if(ADDRESS[15:8] < 8'H68)					// $4000 - $67FF : FLASH, Expansion RAM
			// ROM/Cart expansion  Nothing/FLASH/RAM  This only needs 1 bit
			case (VZ_DECODE_REG[2])
				1'b1:		// Expansion RAM
//					RAM_HIGH <= 5b'00000;				// Set high address lines to zero
					RAM_CS <= 1'b0;						// Enable RAM
					FLASH_CS <= 1'b1;					// Disable FLASH
			default:
				1'b0:		// FLASH
					RAM_CS <= 1'b1;						// Disable RAM
					FLASH_CS <= 1'b0;					// Enable FLASH
			endcase
		else if(ADDRESS[15:8] < 8'H70)					// $6800 - $6FFF : Memory Mapped I/O, Expansion RAM
			// Memory Mapped IO/Expansion RAM
			case (VZ_DECODE_REG[3])
				1'b1:		// Expansion RAM
//					RAM_HIGH <= 5b'00000;				// Set high address lines to zero
					VZ_DECODE_SEL <= 1'b0;				// Disable Regular Memory Map
					RAM_CS <= 1'b0;						// Enable RAM
					FLASH_CS <= 1'b1;					// Disable FLASH
			default:
				1'b0:		// Memory Mapped IO
					VZ_DECODE_SEL <= 1'b1;				// Regular Memory Map
					RAM_CS <= 1'b1;						// Disable RAM
					FLASH_CS <= 1'b1;					// Disable FLASH
			endcase
		else if(ADDRESS[15:8] < 8'H78)					// $7000 - $77FF : Video RAM, Expansion RAM
			// Video RAM / Expansion RAM
			case (VZ_DECODE_REG[4])
				1'b1:		// Expansion RAM
//					RAM_HIGH <= 5b'00000;				// Set high address lines to zero
					VZ_DECODE_SEL <= 1'b0;				// Disable Regular Memory Map
					RAM_CS <= 1'b0;						// Enable RAM
					FLASH_CS <= 1'b1;					// Disable FLASH				
			default:
				1'b0:		// Video RAM
					VZ_DECODE_SEL <= 1'b1;				// Regular Memory Map
					RAM_CS <= 1'b1;						// Disable RAM
					FLASH_CS <= 1'b1;					// Disable FLASH
			endcase
		else if(ADDRESS[15:8] < 8'H80)					// $7800 - $7FFF : 2K-16K systems Built in RAM, Expansion RAM
			// Built in RAM / Expansion RAM
			case (VZ_DECODE_REG[5])
				1'b1:		// Expansion RAM
//					RAM_HIGH <= 5b'00000;				// Set high address lines to zero
					VZ_DECODE_SEL <= 1'b0;				// Disable Regular Memory Map
					RAM_CS <= 1'b0;						// Enable RAM
					FLASH_CS <= 1'b1;					// Disable FLASH
			default:
				1'b0:		// Built in RAM
					VZ_DECODE_SEL <= 1'b1;				// Regular Memory Map
					RAM_CS <= 1'b1;						// Disable RAM
					FLASH_CS <= 1'b1;					// Disable FLASH
			endcase
		else if(ADDRESS[15:8] < 8'H88)					// $8000 - $87FF : 4K-16K systems Built in RAM, Expansion RAM
			// Built in RAM / Expansion RAM
			case (VZ_DECODE_REG[5])
				1'b1:		// Expansion RAM
//					RAM_HIGH <= 5b'00000;				// Set high address lines to zero
					VZ_DECODE_SEL <= 1'b0;				// Disable Regular Memory Map
					RAM_CS <= 1'b0;						// Enable RAM
					FLASH_CS <= 1'b1;					// Disable FLASH
			default:
				1'b0:		// Built in RAM
					VZ_DECODE_SEL <= 1'b1;				// Regular Memory Map
					RAM_CS <= 1'b1;						// Disable RAM
					FLASH_CS <= 1'b1;					// Disable FLASH
			endcase
		else if(ADDRESS[15:8] < 8'HB0)					// $8800 - $8FFF : 6K-16K systems Built in RAM, Expansion RAM
			// Built in RAM / Expansion RAM
			case (VZ_DECODE_REG[5])
				1'b1:		// Expansion RAM
//					RAM_HIGH <= 5b'00000;				// Set high address lines to zero
					VZ_DECODE_SEL <= 1'b0;				// Disable Regular Memory Map
					RAM_CS <= 1'b0;						// Enable RAM
					FLASH_CS <= 1'b1;					// Disable FLASH
			default:
				1'b0:		// Built in RAM
					VZ_DECODE_SEL <= 1'b1;				// Regular Memory Map
					RAM_CS <= 1'b1;						// Disable RAM
					FLASH_CS <= 1'b1;					// Disable FLASH
			endcase
			
		else if(ADDRESS[15:8] < 8'HB8)					// $9000 - $B7FF : 16K systems Built in RAM, Expansion RAM
			// Built in RAM / Expansion RAM
			case (VZ_DECODE_REG[5])
				1'b1:		// Expansion RAM
//					RAM_HIGH <= 5b'00000;				// Set high address lines to zero
					VZ_DECODE_SEL <= 1'b0;				// Disable Regular Memory Map
					RAM_CS <= 1'b0;						// Enable RAM
					FLASH_CS <= 1'b1;					// Disable FLASH
			default:		// Built in RAM
					VZ_DECODE_SEL <= 1'b1;				// Regular Memory Map
					RAM_CS <= 1'b1;						// Disable RAM
					FLASH_CS <= 1'b1;					// Disable FLASH
			endcase
		else if(ADDRESS[15:8] < 4'HC)					// $B800 - $FFFF : Expansion RAM
//			RAM_HIGH <= 5b'00000;						// Set high address lines to zero
			VZ_DECODE_SEL <= 1'b1;						// Regular Memory Map
			RAM_CS <= 1'b0;								// Enable RAM
			FLASH_CS <= 1'b1;							// Disable FLASH
		else if(ADDRESS[15:8] >= 4'HC)					// $B800 - $FFFF : Paged Expansion RAM  $4000  $BFFF
//			RAM_HIGH <= RAM_PAGE_REG;					// Set high address lines to page register contents
			VZ_DECODE_SEL <= 1'b1;						// Regular Memory Map
			RAM_CS <= 1'b0;								// Enable RAM
			FLASH_CS <= 1'b1;							// Disable FLASH
		end
	end
end

endmodule


/////////////////////////////////////////////////////
// copied here to reduce number of chip select lines in final part
// currently unused.  I changed it to 7 inputs
/////////////////////////////////////////////////////
module encoder (value, gray_code);
//input [7:0] value;
input [6:0] value;
output [3:0] gray_code;
always@(value)
	case (value)
		8’b00000001 : gray_code = 3’b000;
		8’b00000010 : gray_code = 3’b001;
		8’b00000100 : gray_code = 3’b011;
		8’b00001000 : gray_code = 3’b010;
		8’b00010000 : gray_code = 3’b110;
		8’b00100000 : gray_code = 3’b111;
		8’b01000000 : gray_code = 3’b101;
//		8’b10000000 : gray_code = 3’b100;
 endcase




