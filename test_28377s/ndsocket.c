/*
 * ndsocket.c
 *
 *  Created on: 2016. 9. 25.
 *      Author: ps
 */

#include "F2837xS_device.h"     // F2837xD Headerfile Include File
#include "F2837xS_Examples.h"   // F2837xD Examples Include File
typedef unsigned char   Uint8;

extern Uint16 wfrdata;
extern Uint16 sdataA[30];

void ndrst()
{
	GPIO_WritePin(82, 0);
	DELAY_US(100);
	GPIO_WritePin(82, 1);
	ATmode=0;

}

void ATstart()
{
	int i;

	GPIO_WritePin(82, 0);
	GPIO_WritePin(82, 1);
	for(i=0;i<200;i++)
	{
		scia_xmit(0x0D);
		DELAY_US(10000);




	}
	ATmode=1;
}

void ATinfo()
{
	char* ATmsg;
	ATmsg="AT+NDINFO";
	scia_msg(ATmsg,9);
	 scia_xmit(0x0D);
	 scia_xmit(0x0A);

	 //DELAY_US(100);
}

void ATndstation()
{
	char* ATmsg;
//	ATmsg="AT+NDSTN=msc_WL_lan1,msclabpc";
//	scia_msg(ATmsg,29);

	ATmsg="AT+NDSTN=WirelessAP,66666666";
	scia_msg(ATmsg,28);
	 scia_xmit(0x0D);
	 scia_xmit(0x0A);

	 //DELAY_US(100);
}

void ATndip()
{
	char* ATmsg;

	ATmsg="AT+NDIP=192.168.188.236";
	scia_msg(ATmsg,23);
	 scia_xmit(0x0D);
	 scia_xmit(0x0A);

	 //DELAY_US(100);
}

void ATndgw()
{
	char* ATmsg;

	ATmsg="AT+NDGW=192.168.188.253";
	scia_msg(ATmsg,23);
	 scia_xmit(0x0D);
	 scia_xmit(0x0A);

	 //DELAY_US(100);
}


void ATndmask()
{
	char* ATmsg;

	ATmsg="AT+NDNMASK=255.255.255.0";
	scia_msg(ATmsg,24);
	 scia_xmit(0x0D);
	 scia_xmit(0x0A);

	 //DELAY_US(100);
}

void ATndreboot()
{
	char* ATmsg;

	ATmsg="AT+REBOOT";
	scia_msg(ATmsg,9);
	 scia_xmit(0x0D);
	 scia_xmit(0x0A);

	 //DELAY_US(100);
}

