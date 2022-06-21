//-----------------------------------------------------------------------------
// Copyright:      RAD Electronic Co. LTD,
// Author:         Sh. Nourbakhsh Rad
// Remarks:        
// known Problems: none
// Version:        1.5.0
// Description:    GLCD's graphic founctions
//-----------------------------------------------------------------------------

#ifndef _GRAPHIC_H_
	#define _GRAPHIC_H_
	

	//#include "ks0108.h"


	#define PutPixel(x, y, clr)											KS108_SetDot(x, y, clr)
	#define RectangleFill(x1, y1, x2, y2, clr)			KS108_FillRect(x1, y1, x2-x1, y2-y1, clr)//rasme morabbe ya mostatil topor
	
	//			Rectangle
	//			Line
	
	#define Bevel(x1, y1, x2, y2, r, clr) 					RoundRectangle(x1, y1, x2, y2, r, 0, clr)
	#define BevelFill(x1, y1, x2, y2, r, clr)				RoundRectangle(x1, y1, x2, y2, r, 1, clr)
	#define Circle(x, y, r, clr)										Circle_Fill(x, y, r, 0, clr)
	#define CircleFill(x, y, r, clr)								Circle_Fill(x, y, r, 1, clr)

	#define VertLine(x, y, length, clr) 						KS108_FillRect(x, y, 0, length, clr)
	#define HoriLine(x, y, length, clr) 						KS108_FillRect(x, y, length, 0, clr)


	//******************* Function Prototypes
	extern void KS108_SetDot(unsigned char x, unsigned char y, unsigned char color);
	extern void Line(unsigned char x1, unsigned char y1, unsigned char x2, unsigned char y2, unsigned char color);
	extern void Rectangle(unsigned char x1, unsigned char y1, unsigned char x2, unsigned char y2, unsigned char color);
	extern void RoundRectangle(unsigned char x1, unsigned char y1, unsigned char x2, unsigned char y2, unsigned char radius, unsigned char fill, unsigned char color);
	extern void Circle_Fill(unsigned char x, unsigned char y, unsigned char radius, unsigned char fill, unsigned char color);
	extern void KS108_FillRect(unsigned char x, unsigned char y, unsigned char width, unsigned char height, unsigned char color);
#endif	//_GRAPHIC_H_
