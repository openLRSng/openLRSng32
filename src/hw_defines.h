#pragma once


//Flytron/OrangeRX = 3
//DTF UHF 4ch = 5
#define DTFUHF10CH 7


//*******************************
//  SELECT HARDWARE HERE
//*******************************
//set this to the hardware type you are compiling for
#define BOARD_TYPE DTFUHF10CH //DTF UHF 10ch


#if BOARD_TYPE == DTFUHF10CH
//PPM_PIN is the only pin connected to timer1. timerHardware[12]
//RSSI_PIN must be the only pin connected to its timer. timerHardware[8]
enum BOARD_PINMAP {
	CH1_PIN = 0,
	CH2_PIN,
	CH3_PIN,
	CH4_PIN,
	CH5_PIN,
	CH6_PIN,
	CH7_PIN,
	CH8_PIN,
	RSSI_PIN,
	PPM_PIN,
	MAX_OUTPUTS
};

#endif
