//-----------------------------------------------------------------------------
// Copyright:      RAD Electronic Co. LTD,
// Author:         Sh. Nourbakhsh Rad based on Fabian Maximilian Thiele KS0108 driver!
// Remarks:        
// known Problems: none
// Version:        1.5.0
// Description:    Graphic Library for KS0108- (and compatible) based LCDs
//								 Tuned for memory mapped GLCD and Persian/Arabic/English font use 06-04-2011
//-----------------------------------------------------------------------------

#ifndef	KS0108C_H
	#define KS0108C_H

#include "../../Inc/main.h"
	//GPIO_InitTypeDef GPIO_InitStruct;

	#define KS108_ORN_PORTRAIT									0						// 1 Portrait display 			: 0 Landscape display
	#define KS108_PORT_INTERFACE								1						// 1 GLCD port interfaced 	: 0 GLCD memory-mapped interfaced
	
	                              	
	// Pins.....
	#if KS108_PORT_INTERFACE
		//--- DATA ---
		/*#define KS108_DB_DDR											DDRA				// KS0108 DATA port
		#define KS108_DB_PRT											PORTA*/
		#define KS108_DB_PIN											Read_DB_GLCD()
		#define KS108_DB_PRT											GPIOB
		#define KS108_DB_High()										HAL_GPIO_WritePin(KS108_DB_PRT,GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3 |GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7,GPIO_PIN_SET)
		#define KS108_DB_Low()										HAL_GPIO_WritePin(KS108_DB_PRT,GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3 |GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7,GPIO_PIN_RESET)

		//--- CTRL ---
		// KS0108  RS (D/I)

		#define KS108_RS_High()										HAL_GPIO_WritePin(GLCD_RS_GPIO_Port,GLCD_RS_Pin,GPIO_PIN_SET)
		#define KS108_RS_Low()										HAL_GPIO_WritePin(GLCD_RS_GPIO_Port,GLCD_RS_Pin,GPIO_PIN_RESET)
		#define KS108_RS_Toggle()									HAL_GPIO_TogglePin(GLCD_RS_GPIO_Port,GLCD_RS_Pin);

		// KS0108  R/W	
		#define KS108_RW_High()										HAL_GPIO_WritePin(GLCD_RW_GPIO_Port,GLCD_RW_Pin,GPIO_PIN_SET)
		#define KS108_RW_Low()										HAL_GPIO_WritePin(GLCD_RW_GPIO_Port,GLCD_RW_Pin,GPIO_PIN_RESET)
		#define KS108_RW_Toggle()									HAL_GPIO_TogglePin(GLCD_RW_GPIO_Port,GLCD_RW_Pin);

		// KS0108  EN		
		#define KS108_EN_High()										HAL_GPIO_WritePin(GLCD_E_GPIO_Port,GLCD_E_Pin,GPIO_PIN_SET)
		#define KS108_EN_Low()										HAL_GPIO_WritePin(GLCD_E_GPIO_Port,GLCD_E_Pin,GPIO_PIN_RESET)
		#define KS108_EN_Toggle()									HAL_GPIO_TogglePin(GLCD_E_GPIO_Port,GLCD_E_Pin);

		// KS0108  CSEL1
		#define KS108_CS1_High()								HAL_GPIO_WritePin(GLCD_CS1_GPIO_Port,GLCD_CS1_Pin,GPIO_PIN_SET)
		#define KS108_CS1_Low()									HAL_GPIO_WritePin(GLCD_CS1_GPIO_Port,GLCD_CS1_Pin,GPIO_PIN_RESET)
		#define KS108_CS1_Toggle()							HAL_GPIO_TogglePin(GLCD_CS1_GPIO_Port,GLCD_CS1_Pin);

		// KS0108  CSEL2	
		#define KS108_CS2_High()								HAL_GPIO_WritePin(GLCD_CS2_GPIO_Port,GLCD_CS2_Pin,GPIO_PIN_SET)
		#define KS108_CS2_Low()									HAL_GPIO_WritePin(GLCD_CS2_GPIO_Port,GLCD_CS2_Pin,GPIO_PIN_RESET)
		#define KS108_CS2_Toggle()							HAL_GPIO_TogglePin(GLCD_CS2_GPIO_Port,GLCD_CS2_Pin);


	#elif	defined(XMemory_OK)
	
		// absolute address of LCD Controller #0 CTRL and DATA registers (Write)
		#define W_CONTROL0_CTRL_ADDR							0x8000
		#define W_CONTROL0_DATA_ADDR							0x8200
						//	 AAAA CCRR
						//	 DDDD SSSW
						//   1000 0000 | 0000 0000  ===  	0x8000
						//   1000 0010 | 0000 0000  ===  	0x8200
		// absolute address of LCD Controller #0 CTRL and DATA registers (Read)
		#define R_CONTROL0_CTRL_ADDR							0x8100
		#define R_CONTROL0_DATA_ADDR							0x8300
						//	 AAAA CCRR
						//	 DDDD SSSW
						//   1000 0001 | 0000 0000  ===  	0x8100
						//   1000 0011 | 0000 0000  ===  	0x8300
	
		// offset of other controllers with respect to controller0
		#define CONTROL_ADDR_OFFSET								0x0400
						//	 AAAA CCRR
						//	 DDDD SSSW
						//   0000 0100 | 0000 0000  ===  	0x0400
	#else
		#error "external data memory interface not available for this device, use GLCD port interface mode"
	#endif
		

	// KS0108  Reset
	#define KS108_RST_High()								HAL_GPIO_WritePin(GLCD_RST_GPIO_Port,GLCD_RST_Pin,GPIO_PIN_SET)
	#define KS108_RST_Low()									HAL_GPIO_WritePin(GLCD_RST_GPIO_Port,GLCD_RST_Pin,GPIO_PIN_RESET)
	#define KS108_RST_Toggle()							HAL_GPIO_TogglePin(GLCD_RST_GPIO_Port,GLCD_RST_Pin);
	
	// KS0108  Backlight
	#define KS108_BKL_PORT									GPIOC
	#define KS108_BKL_bp										GPIO_PIN_13
	#define KS108_BKL_High()								HAL_GPIO_WritePin(KS108_BKL_PORT,KS108_BKL_bp,GPIO_PIN_SET)
	#define KS108_BKL_Low()									HAL_GPIO_WritePin(KS108_BKL_PORT,KS108_BKL_bp,GPIO_PIN_RESET)
	#define KS108_BKL_Toggle()							HAL_GPIO_TogglePin(KS108_BKL_PORT,KS108_BKL_bp);

	//---------------------------------------------------------------

	#if KS108_PORT_INTERFACE			
		#define KS108_DB_CLR()										HAL_GPIO_WritePin(KS108_DB_PRT,GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3 |GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7,GPIO_PIN_RESET)
		#define KS108_DB_PLU()										HAL_GPIO_WritePin(KS108_DB_PRT,GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3 |GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7,GPIO_PIN_SET)

		#define KS108_DB_init()										KS108_DB_CLR()
		#define KS108_DB_WR(x)										Write_DB_GLCD(x)//{HAL_GPIO_WritePin(KS108_DB_PRT,(!x),GPIO_PIN_RESET); HAL_GPIO_WritePin(KS108_DB_PRT,(x),GPIO_PIN_SET);}//KS108_DB_PRT = x
		#define KS108_DB_RD()											KS108_DB_PIN

		#define KS108_DB_OUT()										Output_DB_GLCD()//{  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3 |GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7;/*GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);*/}
		
		#define KS108_DB_IN()											Input_DB_GLCD()//{  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3 |GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7;/*GPIO_InitStruct.Mode = GPIO_MODE_INPUT;GPIO_InitStruct.Pull = GPIO_NOPULL;HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);*/}


		//-------------------------
		#define KS108_CS1_init()									KS108_CS1_Low()
		#define KS108_CS2_init()									KS108_CS2_Low()
		#define KS108_RS_init()										KS108_RS_Low()
		#define KS108_RW_init()										KS108_RW_Low()
		#define KS108_EN_init()										KS108_EN_Low()

		#define KS108_CS1(x)											(x ? (KS108_CS1_High()) : (KS108_CS1_Low()))
		#define KS108_CS2(x)											(x ? (KS108_CS2_High()) : (KS108_CS2_Low()))
		#define KS108_RS(x)												(x ? (KS108_RS_High())  : (KS108_RS_Low()))
		#define KS108_RW(x)												(x ? (KS108_RW_High())  : (KS108_RW_Low()))
		#define KS108_EN(x)												(x ? (KS108_EN_High())  : (KS108_EN_Low()))
	
		#define KS108_CS(cp)											{if(cp==0)			{KS108_CS2(LOW); KS108_CS1(HIGH);}	\
																							 else if(cp==1)	{KS108_CS1(LOW); KS108_CS2(HIGH);}}
	#endif
	
	//-------------------------
	#define KS108_RST_init()										KS108_RST_Low()
	#define KS108_BKL_init()										KS108_BKL_Low()
		
	#define KS108_RST(x)												(x ? (KS108_RST_High()) : (KS108_RST_Low()))
	#define KS108_BKL(x)                        (x ? (KS108_BKL_High()) : (KS108_BKL_Low()))
	
#endif	//KS0108C_H
