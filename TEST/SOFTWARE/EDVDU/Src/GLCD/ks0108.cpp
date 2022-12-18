//-----------------------------------------------------------------------------
// Copyright:      RAD Electronic Co. LTD,
// Author:         Sh. Nourbakhsh Rad based on Fabian Maximilian Thiele KS0108 driver!
// Remarks:        
// known Problems: none
// Version:        2.0.0
// Description:    Graphic Library for KS0108- (and compatible) based LCDs
//								 Tuned for memory mapped GLCD and Persian/Arabic/English font use 06-04-2011
//-----------------------------------------------------------------------------

#include "ks0108.h"
//#include "ks0108config.h"
extern void Input_DB_GLCD(void);
extern void Output_DB_GLCD(void);
extern void Write_DB_GLCD(unsigned char x);
extern unsigned char Read_DB_GLCD(void);
extern unsigned int _delay_us(unsigned int Delay);
//******************* Constants
KS108Coord									KS108_Coord;
Invert_Mode									Inverted 					= NON_INVERTED;

//*************************************************
//******************* Functions *******************
//*************************************************
void KS108_delay450ns(unsigned char n)
{
	while(n--)
	{	
		_delay_us(20);
		//---  6 NOPs for about 450ns at 20MHz?  ---//
		/*_NOP();
		_NOP();
		_NOP();

		_NOP();
		_NOP();
		_NOP();*/
	}
} //KS108_delay450ns

void KS108_Init_Port(void)																		//Initial hardware ...
{
	KS108_RST_init();																	// output & low
//	KS108_BKL_init();																	// output & low
	
	#if KS108_PORT_INTERFACE
		KS108_DB_init();																// output & clear
	
		KS108_CS1_init();																// output & low
		KS108_CS2_init();																// output & low
		KS108_RS_init();																// output & low
		KS108_RW_init();																// output & low
		KS108_EN_init();																// output & low
	#else
    sbi(MCUCR, SRE);																// enable RAM interface (memory mapped glcd)
		sbi(XMCRA, SRW11); 															// and one wait state
	#endif
}	//KS108_Init_Port

#if KS108_PORT_INTERFACE
	void Enable(void)
	{
		KS108_EN(HIGH);																	// EN high level width: min. 450ns
		KS108_delay450ns(1);
		
		KS108_EN(LOW);
		KS108_delay450ns(1);
	} //Enable
#endif

void KS108_BusyWait(unsigned char chip)
{	
	#if KS108_PORT_INTERFACE
		unsigned char 		dat = 0x00;
		
		///_CLI();

		// wait until GLCD busy bit goes to zero
		KS108_DB_IN();
		KS108_CS(chip);																	// select the controller chip

		KS108_RS(LOW);
		KS108_RW(HIGH);

		while(dat & KS108_BUSY_FLAG)
		{
			Enable();
			dat = Read_DB_GLCD();
		}

		KS108_RW(LOW);
		KS108_RS(HIGH);
		KS108_DB_OUT();
		
		///_SEI();
	#else
		KS108_delay450ns(1);
		
		while(*(volatile unsigned char *)								// wait until LCD busy bit goes to zero
			(R_CONTROL0_CTRL_ADDR + (CONTROL_ADDR_OFFSET<<chip)) & KS108_BUSY_FLAG);
	#endif
} //KS108_BusyWait

void KS108_WriteCommand(unsigned char cmd, unsigned char chip)// Write a command to chip
{
	#if KS108_PORT_INTERFACE
		KS108_CS(chip);																	// select the controller chip
		KS108_RS(LOW);
		
		KS108_DB_WR(cmd);		
		Enable();
		
		KS108_RS(HIGH);
	#else
		KS108_BusyWait(chip);														// wait until LCD not busy
		*(volatile unsigned char *) (W_CONTROL0_CTRL_ADDR + (CONTROL_ADDR_OFFSET<<chip)) = cmd;
	#endif
} //KS108_WriteCommand

unsigned char KS108_DoReadData(unsigned char first)
{
	unsigned char 			dat, chip;

	if(KS108_Coord.x < CHIP_WIDTH)						chip = CHIP1;
	else if(KS108_Coord.x >= CHIP_WIDTH)			chip = CHIP2;
		
	if(KS108_Coord.x == CHIP_WIDTH && first) 					// chip2 X-address = 0
		KS108_WriteCommand(KS108_SET_ADD, CHIP2);

	#if KS108_PORT_INTERFACE
		KS108_DB_CLR();
		KS108_DB_IN();

		KS108_CS(chip);
		KS108_RW(HIGH);
  	                            								
		KS108_EN(HIGH); 																// EN high level width: min. 450ns
  		KS108_delay450ns(1);
  	
		dat = Read_DB_GLCD();														// read Data
		
		KS108_EN(LOW); 
		KS108_delay450ns(1);
  	  
		KS108_RW(LOW);
		KS108_DB_OUT();
	#else
		KS108_BusyWait(chip);														// wait until LCD not busy
		dat = *(volatile unsigned char *)  (R_CONTROL0_DATA_ADDR + (CONTROL_ADDR_OFFSET<<chip));
	#endif

	KS108_GotoXY(KS108_Coord.x, KS108_Coord.y);
	
	return(Inverted ? ~dat : dat);
} //KS108_DoReadData

unsigned char KS108_ReadData(void)
{
	KS108_DoReadData(DUMMY);
	
	return KS108_DoReadData(REAL);
} //KS108_ReadData

void KS108_WriteData(unsigned char dat)												// Write data to chip
{
	unsigned char 			displayData, yOffset;
	unsigned char 			chip;

	if(KS108_Coord.x >= DISPLAY_WIDTH)				return;
  	
	if(KS108_Coord.x < CHIP_WIDTH)						chip = CHIP1;
	else if(KS108_Coord.x >= CHIP_WIDTH)			chip = CHIP2;
  	
	if(KS108_Coord.x == CHIP_WIDTH)										// chip2 X-address = 0
		KS108_WriteCommand(KS108_SET_ADD, CHIP2);

	//-------------------------
	yOffset = KS108_Coord.y % 8;
	
	if(yOffset != 0)
	{
		// first page
		displayData = KS108_ReadData();
		displayData |= dat << yOffset;
		
		if(Inverted)			displayData = ~displayData;
		
		#if KS108_PORT_INTERFACE
			KS108_CS(chip);																// select the controller chip  		
			
			KS108_DB_WR(displayData);											// write data
			Enable();
 		#else
			KS108_BusyWait(chip);													// wait until LCD not busy
			*(volatile unsigned char *) (W_CONTROL0_DATA_ADDR + (CONTROL_ADDR_OFFSET<<chip)) = displayData;
 		#endif
 		
		// second page
		KS108_GotoXY(KS108_Coord.x, KS108_Coord.y+8);
		displayData = KS108_ReadData();
		displayData |= dat >> (8-yOffset);
		
		if(Inverted)			displayData = ~displayData;
		
		#if KS108_PORT_INTERFACE
			KS108_CS(chip);																// select the controller chip  		
			
			KS108_DB_WR(displayData);											// write data
			Enable();
 		#else
			KS108_BusyWait(chip);													// wait until LCD not busy
			*(volatile unsigned char *) (W_CONTROL0_DATA_ADDR + (CONTROL_ADDR_OFFSET<<chip)) = displayData;
 		#endif
 	
		KS108_GotoXY(KS108_Coord.x+1, KS108_Coord.y-8);
	} 
	else 
	{
		if(Inverted)			dat = ~dat;
		
		#if KS108_PORT_INTERFACE
			KS108_CS(chip);																// select the controller chip  		
			
			KS108_DB_WR(dat);															// write data
			Enable();
 		#else
			KS108_BusyWait(chip);													// wait until LCD not busy
			*(volatile unsigned char *) (W_CONTROL0_DATA_ADDR + (CONTROL_ADDR_OFFSET<<chip)) = dat;
 		#endif
		
		KS108_Coord.x++;
	}
} //KS108_WriteData

void KS108_ClearPage(unsigned char page, unsigned char color)
{
	unsigned char 			x;
	
	for(x=0; x<DISPLAY_WIDTH; x++)
	{	
		KS108_GotoXY(x, page*8);
    KS108_WriteData(color);
  }	
} //KS108_ClearPage

//-------------------------------------------------
void KS108_Init(Invert_Mode invert)														//Initial and start GLCD...
{
	KS108_Init_Port();
	
	//-------------
	KS108_RST(HIGH);
	KS108_BKL(LOW);
	
	KS108_delay450ns(2);
	
	//-------------
	KS108_Coord.x 			= 0;
	KS108_Coord.y 			= 0;
	KS108_Coord.page 		= 0;

	Inverted 						= invert;

	KS108_WriteCommand(KS108_ON, CHIP1);							// power on GLCD
	KS108_WriteCommand(KS108_ON, CHIP2);

	KS108_WriteCommand(KS108_DISP_START, CHIP1);			// display start line = 0
	KS108_WriteCommand(KS108_DISP_START, CHIP2);
	
	KS108_CLS(invert ? BLACK : WHITE);	   					 	// display clear
	
	KS108_GotoXY(0, 0);
	KS108_CLSx();
} //KS108_Init


void KS108_Backlight(unsigned char x)													// GLCD backlight ON/OFF
{
	KS108_BKL(x);
}	//KS108_Backlight


//-------------------------------------------------
void KS108_CLS(unsigned char color)
{
	unsigned char 			page;
	
	for(page=0; page<8; page++)
	{
		KS108_GotoXY(0, page*8);
		KS108_ClearPage(page, color);
  }
} //KS108_CLS

void KS108_GotoXY(unsigned char x, unsigned char y)						//x: 0...127 and y: 0...63 in normal mode!
{
	
  unsigned char 			chip, cmd;
	
  if((x > GetMaxX()) || (y > GetMaxY()))			return;			// exit if Coordinates are not legal
  
  KS108_Coord.x = x;																			// save new KS108_Coordinates
  KS108_Coord.y = y;
	
  if((y/8) != KS108_Coord.page)
  {
  	KS108_Coord.page = y / 8;
		cmd = KS108_SET_PAGE | KS108_Coord.page;							// set y address on all chips	
		
		for(chip=CHIP1; chip<(DISPLAY_WIDTH/CHIP_WIDTH); chip++)
	  	KS108_WriteCommand(cmd, chip);	
  }
  
  chip = KS108_Coord.x / CHIP_WIDTH;
  x = x % CHIP_WIDTH;
  
  cmd = KS108_SET_ADD | x;
  KS108_WriteCommand(cmd, chip);													// set x address on active chip	
} //KS108_GotoXY

//-------------------------------------------------void KS108_SetDot(unsigned char x, unsigned char y, KS108_Color color);
void KS108_SetDot(unsigned char x, unsigned char y, unsigned char color)
{
	unsigned char 			dat;

//////////////////////////////////////////////////////////////////////////////
	x=128-x;
	y=64-y;
//////////////////////////////////////////////////////////////////////////////	
	
	
	KS108_GotoXY(x, y-y%8);														// read data from display memory
	dat = KS108_ReadData();

	if(color == BLACK)		dat |= 	(0x01 << (y%8));		// set dot
	else									dat &= ~(0x01 << (y%8));		// clear dot

	KS108_WriteData(dat);					   									// write data back to display
} //KS108_SetDot

void KS108_FillRect(unsigned char x, unsigned char y, unsigned char width, unsigned char height, unsigned char color)
{
	
	unsigned char 			mask, pageOffset;
	unsigned char 			h, i;
	unsigned char 			dat;

	height++;

	mask = 0xFF;
	pageOffset = y % 8;
	y -= pageOffset;
	
	if(height < (8-pageOffset))
	{
		mask >>= (8-height);
		h = height;
	}
	else
	{
		h = 8 - pageOffset;
	}
	
	mask <<= pageOffset;

	KS108_GotoXY(x, y);
	for(i=0; i<=width; i++)
	{
		dat = KS108_ReadData();

		if(color == BLACK)			dat |=  mask;
		else										dat &= ~mask;

		KS108_WriteData(dat);
	}

	while((h+8) <= height)
	{
		h += 8;
		y += 8;
		
		KS108_GotoXY(x, y);
		for(i=0; i<=width; i++)
			KS108_WriteData(color);
	}

	if(h < height)
	{
		mask = ~(0xFF << (height-h));
		
		KS108_GotoXY(x, y+8);
		for(i=0; i<=width; i++)
		{
			dat = KS108_ReadData();

			if(color == BLACK)			dat |=  mask;
			else										dat &= ~mask;

			KS108_WriteData(dat);
		}
	}
} //KS108_FillRect

void KS108_InvertRect(unsigned char x, unsigned char y, unsigned char width, unsigned char height)
{
	unsigned char 			mask, pageOffset;
	unsigned char 			h, i;
	unsigned char 			dat, tmpData;
	
	height++;

	mask = 0xFF;
	pageOffset = y % 8;
	y -= pageOffset;

	if(height < (8-pageOffset))
	{
		mask >>= (8-height);
		h = height;
	}
	else
	{
		h = 8 - pageOffset;
	}
	
	mask <<= pageOffset;

	KS108_GotoXY(x, y);
	for(i=0; i<=width; i++)
	{
		dat = KS108_ReadData();
		tmpData = ~dat;
		dat = (tmpData & mask) | (dat & ~mask);
		KS108_WriteData(dat);
	}

	while((h+8) <= height)
	{
		h += 8;
		y += 8;
		
		KS108_GotoXY(x, y);
		for(i=0; i<=width; i++)
		{
			dat = KS108_ReadData();
			KS108_WriteData(~dat);
		}
	}

	if(h < height)
	{
		mask = ~(0xFF << (height-h));
		
		KS108_GotoXY(x, y+8);
		for(i=0; i<=width; i++)
		{
			dat = KS108_ReadData();
			tmpData = ~dat;
			dat = (tmpData & mask) | (dat & ~mask);
			KS108_WriteData(dat);
		}
	}
} //KS108_InvertRect

void KS108_SetInverted(Invert_Mode invert)
{
	if(Inverted != invert)
	{
		KS108_InvertRect(0, 0, GetMaxX(), GetMaxY());
		Inverted = invert;
	}
} //KS108_SetInverted

//-------------------------------------------------
//--- Loads a Bitmap to GLCD ---//
void KS108_DrawBitmap(_const unsigned char *bitmap, unsigned char x, unsigned char y, Show_Mode mode)
{
	unsigned char 			width, height, header;
	unsigned char 			displayData;

	unsigned char 			xx, yy, yyy;
	unsigned char				hh, ww;
	
	unsigned char				endShift;
	unsigned int				xy;

  header = (bitmap[0]); 
  width  = (bitmap[1]); 
  height = (bitmap[2]);
  
	// Drawing position calculation
	ww = width;
	hh = height;
	
	if((x+ww) > DISPLAY_WIDTH)		ww = DISPLAY_WIDTH  -x;
	if((y+hh) > DISPLAY_HEIGHT) 	hh = DISPLAY_HEIGHT -y;
	
	if(hh%8)			hh = hh/8+1;
	else					hh = hh/8;
	
	endShift = 0;

	// Draw my BMP!
	if(mode != TRANS)			KS108_FillRect(x, y, width-1, height-1, WHITE);		//no transparent

	for(yy=0; yy<hh; yy++)
	{
		yyy = y + (yy * 8);

		if(yyy > (DISPLAY_HEIGHT-8))
		{
		 	endShift = y % 8;
			yyy = DISPLAY_HEIGHT -8;
		}

		KS108_GotoXY(x, yyy);
		xy = (unsigned int)(yy)*width +header;

		for(xx=0; xx<ww; xx++)
		{
			displayData = (bitmap[xy++]);
			
			if(endShift)			KS108_WriteData((displayData << endShift)|KS108_ReadData());
			else							KS108_WriteData(displayData);
	 	}
  }
	
	if(mode == INVERS)			KS108_InvertRect(x, y, width-1, height-1);			//invers color
} //KS108_DrawBitmap
