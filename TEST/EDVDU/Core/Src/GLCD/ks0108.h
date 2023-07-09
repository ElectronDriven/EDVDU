//-----------------------------------------------------------------------------
// Copyright:      RAD Electronic Co. LTD,
// Author:         Sh. Nourbakhsh Rad based on Fabian Maximilian Thiele KS0108 driver!
// Remarks:        
// known Problems: none
// Version:        2.0.0
// Description:    Graphic Library for KS0108- (and compatible) based LCDs
//								 Tuned for memory mapped GLCD and Persian/Arabic/English font use 06-04-2011
//-----------------------------------------------------------------------------

#ifndef	KS0108_H
	#define KS0108_H
	#include "ks0108config.h"
	


	#define DISPLAY_WIDTH												128
	#define DISPLAY_HEIGHT											64

	// Orientation
	#if KS108_ORN_PORTRAIT
		#define KS108_SIZE_Y											DISPLAY_WIDTH
		#define KS108_SIZE_X											DISPLAY_HEIGHT
	#else
		#define KS108_SIZE_X											DISPLAY_WIDTH
		#define KS108_SIZE_Y											DISPLAY_HEIGHT
	#endif

	#define GetMaxX()														((unsigned int)KS108_SIZE_X-1)
	#define GetMaxY()														((unsigned int)KS108_SIZE_Y-1)

	// Panel controller chips      	
	#define CHIP_WIDTH     											64  		// pixels per chip 
	                            		
	// Chips                    		
	#define CHIP1       												0x00
	#define CHIP2       												0x01
	#define CHIP3       												0x02
	#define CHIP4       												0x03
	                            		

	// Commands                 		
	#define KS108_ON														0x3F
	#define KS108_OFF														0x3E
	#define KS108_SET_ADD												0x40
	#define KS108_SET_PAGE											0xB8
	#define KS108_DISP_START										0xC0
	#define KS108_BUSY_FLAG											0x80

	#define REAL     														0
	#define DUMMY    														1
	                            		
	// Colors                   		
	typedef enum {
		WHITE		= 0x00,
		BLACK		= 0xFF
		
	} KS108_Color;

/*enum Zar {
		WHITE		= 0x00,
		BLACK		= 0xFF
		};	
typedef enum Zar KS108_Color ;	*/

	// BMP draw modes
	typedef enum {
		INVERS,
		NORMAL,
		TRANS
	} Show_Mode;

	// Useful user constants     		
	typedef enum {
		NON_INVERTED	= 0,
	  INVERTED			= 1
	} Invert_Mode;
                            		
	// GLCD internal constants	
	typedef struct {
		unsigned char x;
		unsigned char y;
		unsigned char page;
	} KS108Coord;
	
	
	//******************* Function Prototypes
	void KS108_Init(Invert_Mode invert);
	void KS108_Backlight(unsigned char x);

	void KS108_CLS(unsigned char color);
	void KS108_GotoXY(unsigned char x, unsigned char y);
	
	void KS108_SetDot(unsigned char x, unsigned char y, unsigned char color);
	void KS108_FillRect(unsigned char x, unsigned char y, unsigned char width, unsigned char height, unsigned char color);
	
	void KS108_InvertRect(unsigned char x, unsigned char y, unsigned char width, unsigned char height);
	/*
	baray makous kardan bakhshi az LCD estefade mishavad ke bayad mokhtassat marbote ra vared nemod
	*/
	
	void KS108_SetInverted(Invert_Mode invert);
	/*
	dar sorati ke lazem bashad kolle safhe makous shavad az in dastor estefade mikonim
	*/
	
	void KS108_DrawBitmap(_const unsigned char *bitmap, unsigned char x, unsigned char y, Show_Mode mode);
	/*
	baray rasme ax mibashd ke daraye mod haye zir mibashad:
	NORMAL:nemayesh sahih ax
	INVERS:nemayesh makous ax(be jaye har pixel meshki pixel marbote roye lcd khamosh mishavad
	TRANS:
	*/
	
	#define KS108_CLSx()				KS108_FillRect(0, 0, GetMaxX(), GetMaxY(), WHITE)
	//#define PutPixel(x, y, colr)											KS108_SetDot(x, y, clr)

#endif	//KS0108_H
