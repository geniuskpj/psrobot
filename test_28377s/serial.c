/*
 * serial.c
 *
 *  Created on: 2016. 5. 17.
 *      Author: KJC
 */
#include "F2837xD_device.h"     // F2837xD Headerfile Include File
#include "F2837xD_Examples.h"   // F2837xD Examples Include File
#include "serial.h"
#include "psuart.h"
typedef unsigned char Uint8;

Uint32 dx_baud, CPU_FREQ, LSPCLK_FREQ, SCI_FREQ, trdelay, SCI_FREQa, SCI_FREQb,
		SCI_FREQd, SCI_FREQc;
Uint16 SCI_PRDc, SCI_PRDa, SCI_PRDb, SCI_PRDd;

void scia_fifo_init(void);
void scib_fifo_init(void);
void scic_fifo_init(void);
void scid_fifo_init(void);
interrupt void sciaTxFifoIsr(void);
interrupt void sciaRxFifoIsr(void);
interrupt void scibTxFifoIsr(void);
interrupt void scibRxFifoIsr(void);
interrupt void scicTxFifoIsr(void);
interrupt void scicRxFifoIsr(void);
interrupt void scidTxFifoIsr(void);
interrupt void scidRxFifoIsr(void);
void scia_xmit(Uint8 a);
Uint16 scia_msg(const char *a, Uint16 leng);
Uint16 scia_rcv();
void scib_xmit(Uint8 a);
Uint16 scib_msg(const char *a, Uint16 leng);
Uint16 scib_rcv();
void scic_xmit(Uint8 a);
Uint16 scic_msg(Uint8* a, Uint8 leng);
Uint16 scic_rcv();

void scid_xmit(Uint8 a);
Uint16 scid_msg(const char *a, Uint16 leng);
Uint16 scid_rcv();

Uint16 sdataA[30];    // Send data for SCI-A
//Uint16 sdataA2[10];    // Send data for SCI-A
Uint16 rdataA[16];    // Received data for SCI-A
Uint16 bufa;    // Received data for wifi
Uint16 bufb;    // Received data for wifi
Uint16 bufc;    // Received data for wifi
Uint16 wfing;
Uint16 wfrcvnum;
Uint16 ecing;
extern Uint16 ATmode;
//char ATrxmsg[50];
//char ATtxmsg[50];
Uint16 ATcnt = 0;
Uint16 ATcnt2 = 0;
Uint16 lengscib = 64;
bool atxing = false;

void UARTIntclear(Uint16 ui16Base);
void UARTPrimeTransmit(Uint16 ui16Base);

//Uint16 rdataB[16];    // Received data for SCI-B
Uint16 rdata;
Uint16 rdata_pointA; // Used for checking the received data
Uint16 graphdata;
extern Uint16 j, rcvid, err, releng, cmd;
extern unsigned int posint[8];
extern float pos[8], gearrt[8];
extern int16 dir[8], oset[8];

extern bool rcvsuc, test;
extern Uint8 atpntflag;

void scia_fifo_init() {

#ifdef _FLASH
	CPU_FREQ=190E6;
#ifndef __cplusplus
#pragma CODE_SECTION(scidRxFifoIsr, "ramfuncs");
#pragma CODE_SECTION(UARTPrimeTransmit, "ramfuncs");
#pragma CODE_SECTION(scid_xmit, "ramfuncs");
#pragma CODE_SECTION(scid_rcv, "ramfuncs");
#endif

	memcpy(&RamfuncsRunStart, &RamfuncsLoadStart, (size_t)&RamfuncsLoadSize);

#else
	CPU_FREQ = 200E6;
#endif

	LSPCLK_FREQ = CPU_FREQ / 4;
	//#define LSPCLK_FREQ CPU_FREQ/2

	SCI_FREQa = 9600;
	SCI_PRDa = ((LSPCLK_FREQ / (SCI_FREQa * 8)) - 1);

	SciaRegs.SCICCR.all = 0x0007;   // 1 stop bit,  No loopback
									// No parity,8 char bits,
									// async mode, idle-line protocol
	SciaRegs.SCICTL1.all = 0x0003;  // enable TX, RX, internal SCICLK,
									// Disable RX ERR, SLEEP, TXWAKE
	SciaRegs.SCICTL2.all = 0x0003;
	SciaRegs.SCICTL2.bit.TXINTENA = 1;
	SciaRegs.SCICTL2.bit.RXBKINTENA = 1;
	SciaRegs.SCIHBAUD.all = SCI_PRDa >> 8;
	SciaRegs.SCILBAUD.all = SCI_PRDa & 0xFF;

//   SciaRegs.SCIFFTX.all=0xE040;
//   SciaRegs.SCIFFRX.all=0x2060;
//   SciaRegs.SCIFFCT.all=0x00;
//   SciaRegs.SCIFFTX.bit.SCIFFENA=0;
//   SciaRegs.SCIFFTX.bit.TXFFIENA=0;
//   SciaRegs.SCIFFRX.bit.RXFFIENA=1;
//   SciaRegs.SCIFFTX.bit.TXFFIL = 1;
//   SciaRegs.SCIFFRX.bit.RXFFIL = 1;
	SciaRegs.SCICTL1.all = 0x0023;     // Relinquish SCI from Reset
//   SciaRegs.SCIFFTX.bit.TXFIFORESET=1;
//   SciaRegs.SCIFFRX.bit.RXFIFORESET=1;

	//scia_xmit(0x0A);
//   memset(ATtxmsg, 0, sizeof(ATtxmsg));
//   memset(ATrxmsg, 0, sizeof(ATrxmsg));

}

void scib_fifo_init() {

	//	SCI_FREQb=    115200;
	//	SCI_PRDb=     48;
	SCI_FREQb = 19200;
	SCI_PRDb = (LSPCLK_FREQ / (SCI_FREQb * 8)) - 1;

	ScibRegs.SCICCR.all = 0x0007;   // 1 stop bit,  No loopback
									// No parity,8 char bits,
									// async mode, idle-line protocol
	ScibRegs.SCICTL1.all = 0x0003;  // enable TX, RX, internal SCICLK,
									// Disable RX ERR, SLEEP, TXWAKE
	ScibRegs.SCICTL2.all = 0x0003;
	ScibRegs.SCICTL2.bit.TXINTENA = 1;
	ScibRegs.SCICTL2.bit.RXBKINTENA = 1;
	ScibRegs.SCIHBAUD.all = SCI_PRDb >> 8;
	ScibRegs.SCILBAUD.all = SCI_PRDb & 0xFF;

//   ScibRegs.SCIFFTX.all=0xE040;
//   ScibRegs.SCIFFRX.all=0x2060;
//   ScibRegs.SCIFFCT.all=0x00;
//   ScibRegs.SCIFFTX.bit.TXFFIL = 1;
//   ScibRegs.SCIFFTX.bit.SCIFFENA=1;
//   ScibRegs.SCIFFTX.bit.TXFFIENA=1;
//   ScibRegs.SCIFFRX.bit.RXFFIL = 1;
	ScibRegs.SCICTL1.all = 0x0023;     // Relinquish SCI from Reset
//   ScibRegs.SCIFFTX.bit.TXFIFORESET=1;
//   ScibRegs.SCIFFRX.bit.RXFIFORESET=1;

}

void scic_fifo_init() {

	SCI_FREQc = 115200;
	SCI_PRDc = (LSPCLK_FREQ / (SCI_FREQc * 8)) - 1;

	ScicRegs.SCICCR.all = 0x0007;   // 1 stop bit,  No loopback
									// No parity,8 char bits,
									// async mode, idle-line protocol
	ScicRegs.SCICTL1.all = 0x0003;  // enable TX, RX, internal SCICLK,
									// Disable RX ERR, SLEEP, TXWAKE
	ScicRegs.SCICTL2.all = 0x0003;
	ScicRegs.SCICTL2.bit.TXINTENA = 1;
	ScicRegs.SCICTL2.bit.RXBKINTENA = 1;
	ScicRegs.SCIHBAUD.all = SCI_PRDc >> 8;
	ScicRegs.SCILBAUD.all = SCI_PRDc & 0xFF;

//   ScibRegs.SCIFFTX.all=0xE040;
//   ScibRegs.SCIFFRX.all=0x2060;
//   ScibRegs.SCIFFCT.all=0x00;
//   ScibRegs.SCIFFTX.bit.TXFFIL = 1;
//   ScibRegs.SCIFFTX.bit.SCIFFENA=1;
//   ScibRegs.SCIFFTX.bit.TXFFIENA=1;
//   ScibRegs.SCIFFRX.bit.RXFFIL = 1;
	ScicRegs.SCICTL1.all = 0x0023;     // Relinquish SCI from Reset
//   ScibRegs.SCIFFTX.bit.TXFIFORESET=1;
//   ScibRegs.SCIFFRX.bit.RXFIFORESET=1;

}

void scid_fifo_init() {
	dx_baud = 1; // 1MBps
//	dx_baud=7; // 250kBps
			//#define dx_baud 34 // default

	SCI_FREQd = 2000000 / (dx_baud + 1);
	trdelay = 24E6 / SCI_FREQd;
#ifdef _FLASH
	SCI_PRDd=5;
#else

	SCI_PRDd = (LSPCLK_FREQ / (SCI_FREQd * 8)) - 1;
#endif
//	SCI_PRDd=     5;

	ScidRegs.SCICCR.all = 0x0007;   // 1 stop bit,  No loopback
									// No parity,8 char bits,
									// async mode, idle-line protocol
	ScidRegs.SCICTL1.all = 0x0003;  // enable TX, RX, internal SCICLK,
									// Disable RX ERR, SLEEP, TXWAKE
	ScidRegs.SCICTL2.all = 0x0003;
	ScidRegs.SCICTL2.bit.TXINTENA = 1;
	ScidRegs.SCICTL2.bit.RXBKINTENA = 1;
	ScidRegs.SCIHBAUD.all = SCI_PRDd >> 8;
	ScidRegs.SCILBAUD.all = SCI_PRDd & 0xFF;

//   ScibRegs.SCIFFTX.all=0xE040;
//   ScibRegs.SCIFFRX.all=0x2060;
//   ScibRegs.SCIFFCT.all=0x00;
//   ScibRegs.SCIFFTX.bit.TXFFIL = 1;
//   ScibRegs.SCIFFTX.bit.SCIFFENA=1;
//   ScibRegs.SCIFFTX.bit.TXFFIENA=1;
//   ScibRegs.SCIFFRX.bit.RXFFIL = 1;
	ScidRegs.SCICTL1.all = 0x0023;     // Relinquish SCI from Reset
//   ScibRegs.SCIFFTX.bit.TXFIFORESET=1;
//   ScibRegs.SCIFFRX.bit.RXFIFORESET=1;

}

interrupt void sciaTxFifoIsr(void) {
#ifdef UART_BUFFERED
	Uint16 base = 0;
	if (!aTX_BUFFER_EMPTY)
		UARTPrimeTransmit(base);

#endif

//    SciaRegs.SCIFFTX.bit.TXFFINTCLR=1;  // Clear SCI Interrupt flag, don't need if not fifo mode

	//GpioDataRegs.GPADAT.bit.GPIO31=0;
	PieCtrlRegs.PIEACK.all |= M_INT9;      // Issue PIE ACK

}

interrupt void scibTxFifoIsr(void) {
#ifdef UART_BUFFERED
	Uint16 base = 1;
	if (!bTX_BUFFER_EMPTY)
		UARTPrimeTransmit(base);

#endif
//    ScibRegs.SCIFFTX.bit.TXFFINTCLR=1;  // Clear SCI Interrupt flag

	//GpioDataRegs.GPADAT.bit.GPIO31=0;
	PieCtrlRegs.PIEACK.all |= M_INT9;      // Issue PIE ACK

}
interrupt void scicTxFifoIsr(void) {

#ifdef UART_BUFFERED
	Uint16 base = 2;
	if (!cTX_BUFFER_EMPTY)
		UARTPrimeTransmit(base);

#endif
//    ScibRegs.SCIFFTX.bit.TXFFINTCLR=1;  // Clear SCI Interrupt flag

	//GpioDataRegs.GPADAT.bit.GPIO31=0;
	PieCtrlRegs.PIEACK.all |= M_INT9;      // Issue PIE ACK

}

interrupt void scidTxFifoIsr(void) {
#ifdef UART_BUFFERED
	Uint16 base = 3;
	if (!dTX_BUFFER_EMPTY)
		UARTPrimeTransmit(base);

#endif
//    ScibRegs.SCIFFTX.bit.TXFFINTCLR=1;  // Clear SCI Interrupt flag

	//GpioDataRegs.GPADAT.bit.GPIO31=0;
	PieCtrlRegs.PIEACK.all |= M_INT9;      // Issue PIE ACK

}

interrupt void sciaRxFifoIsr(void) {

#ifdef UART_BUFFERED
	Uint16 base = 0;
	Uint16 ui16char;
	Uint8 cchar;
	static bool blastwascr = false;
//	UARTIntclear(base); //clear interrupt
	while (SciaRegs.SCIRXST.bit.RXRDY) {
		//read a character
		cchar = (unsigned char) scia_rcv();

		//backspace
		if (cchar == '\b') {
			if (!aRX_BUFFER_EMPTY) {
				scia_msg("\b \b", 3);

				//
				// Decrement the number of characters in the buffer.
				//
				if (a_ui16UARTRxWriteIndex == 0) {
					a_ui16UARTRxWriteIndex = UART_RX_BUFFER_SIZE - 1;
				} else {
					a_ui16UARTRxWriteIndex--;
				}

			}
			continue;
		}

		//
		// If this character is LF and last was CR, then just gobble up
		// the character since we already echoed the previous CR and we
		// don't want to store 2 characters in the buffer if we don't
		// need to.
		//
		if ((cchar == '\n') && blastwascr) {
			blastwascr = false;
			continue;
		}

		//
		// See if a newline or escape character was received.
		//
		if ((cchar == '\r') || (cchar == '\n') || (cchar == 0x1b)) {
			//
			// If the character is a CR, then it may be followed by an
			// LF which should be paired with the CR.  So remember that
			// a CR was received.
			//
			if (cchar == '\r') {
				blastwascr = 1;
			}

			//
			// Regardless of the line termination character received,
			// put a CR in the receive buffer as a marker telling
			// UARTgets() where the line ends.  We also send an
			// additional LF to ensure that the local terminal echo
			// receives both CR and LF.
			//
			cchar = '\r';
			//scia_msg("\n", 1);
		}

		if (!aRX_BUFFER_FULL) {
			//write to rx ring buffer
			a_pcUARTRxBuffer[a_ui16UARTRxWriteIndex] = cchar;
			ADVANCE_RX_BUFFER_INDEX(a_ui16UARTRxWriteIndex);

			//write to tx ring buffer
//        		scia_msg(&cchar,1);

			//write to b_tx ring buffer
//        		scib_msg(&cchar,1);
		}

	}

//	    SciaRegs.SCIFFRX.bit.RXFFOVRCLR=1;   // Clear Overflow flag
//	    SciaRegs.SCIFFRX.bit.RXFFINTCLR=1;   // Clear Interrupt flag

	PieCtrlRegs.PIEACK.all |= M_INT9;       // Issue PIE ack
#else

			char* wftxmsg;
			int i;

			EINT;
			while(SciaRegs.SCIFFRX.bit.RXFFST!=0)
			{
				bufa=SciaRegs.SCIRXBUF.all;  // Read data

				if (ATmode)
				{

					ATtxmsg[ATcnt]=bufa;
					scia_xmit(ATtxmsg[ATcnt]);

					ATcnt++;
					if(bufa==0x0D)
					{
						scia_xmit(0x0A);
						scib_msg(ATtxmsg,0);

						ATcnt=0;
						break;
						//atpntflag=1;
					}
				}

			}

			SciaRegs.SCIFFRX.bit.RXFFOVRCLR=1;   // Clear Overflow flag
			SciaRegs.SCIFFRX.bit.RXFFINTCLR=1;// Clear Interrupt flag

			PieCtrlRegs.PIEACK.all|=M_INT9;// Issue PIE ack

#endif
}

interrupt void scibRxFifoIsr(void) {

#ifdef UART_BUFFERED
	Uint16 base = 1;
	Uint16 ui16char;
	Uint8 cchar;
	static bool blastwascr = false;
//	UARTIntclear(base); //clear interrupt
	while (ScibRegs.SCIRXST.bit.RXRDY) {
		//read a character
		cchar = (unsigned char) scib_rcv();

		//backspace
		if (cchar == '\b') {
			if (!bRX_BUFFER_EMPTY) {
				scib_msg("\b \b", 3);

				//
				// Decrement the number of characters in the buffer.
				//
				if (b_ui16UARTRxWriteIndex == 0) {
					b_ui16UARTRxWriteIndex = UART_RX_BUFFER_SIZE - 1;
				} else {
					b_ui16UARTRxWriteIndex--;
				}

			}
			continue;
		}

		//
		// If this character is LF and last was CR, then just gobble up
		// the character since we already echoed the previous CR and we
		// don't want to store 2 characters in the buffer if we don't
		// need to.
		//
		if ((cchar == '\n') && blastwascr) {
			blastwascr = false;
			continue;
		}

		//
		// See if a newline or escape character was received.
		//
		if ((cchar == '\r') || (cchar == '\n') || (cchar == 0x1b)) {
			//
			// If the character is a CR, then it may be followed by an
			// LF which should be paired with the CR.  So remember that
			// a CR was received.
			//
			if (cchar == '\r') {
				blastwascr = 1;
			}

			//
			// Regardless of the line termination character received,
			// put a CR in the receive buffer as a marker telling
			// UARTgets() where the line ends.  We also send an
			// additional LF to ensure that the local terminal echo
			// receives both CR and LF.
			//
			cchar = '\r';
			//scia_msg("\n", 1);
		}

		if (!bRX_BUFFER_FULL) {
			b_pcUARTRxBuffer[b_ui16UARTRxWriteIndex] = cchar;
			ADVANCE_RX_BUFFER_INDEX(b_ui16UARTRxWriteIndex);

//        		scia_msg(&cchar,1);

		}

	}

//
//	    ScibRegs.SCIFFRX.bit.RXFFOVRCLR=1;   // Clear Overflow flag
//	    ScibRegs.SCIFFRX.bit.RXFFINTCLR=1;   // Clear Interrupt flag

	PieCtrlRegs.PIEACK.all |= M_INT9;       // Issue PIE ack
#else
			char trash;

			while(ScibRegs.SCIFFRX.bit.RXFFST!=0)
			{
				bufb=ScibRegs.SCIRXBUF.all;  // Read data

				if (ATmode&(bufb!=0x0A))
				{

					ATrxmsg[ATcnt2]=bufb;
					ATcnt2++;
					if(bufb==0x0D)
					{
						scia_msg(ATrxmsg,0);
						ATcnt2=0;
//		  			 atpntflag=1;
						break;

					}
				}

			}

			ScibRegs.SCIFFRX.bit.RXFFOVRCLR=1;   // Clear Overflow flag
			ScibRegs.SCIFFRX.bit.RXFFINTCLR=1;// Clear Interrupt flag

			PieCtrlRegs.PIEACK.all|=M_INT9;// Issue PIE ack
#endif
}

interrupt void scicRxFifoIsr(void) {
#ifdef UART_BUFFERED
	Uint16 base = 2;
	Uint16 ui16char;
	Uint8 cchar;
	static bool blastwascr = false;
	//	UARTIntclear(base); //clear interrupt
	while (ScicRegs.SCIRXST.bit.RXRDY) {
		//read a character
		cchar = (unsigned char) scic_rcv();

		//backspace
		if (cchar == '\b') {
			if (!cRX_BUFFER_EMPTY) {
				scic_msg("\b \b", 3);

				//
				// Decrement the number of characters in the buffer.
				//
				if (c_ui16UARTRxWriteIndex == 0) {
					c_ui16UARTRxWriteIndex = UART_RX_BUFFER_SIZE - 1;
				} else {
					c_ui16UARTRxWriteIndex--;
				}

			}
			continue;
		}

		//
		// If this character is LF and last was CR, then just gobble up
		// the character since we already echoed the previous CR and we
		// don't want to store 2 characters in the buffer if we don't
		// need to.
		//
		if ((cchar == '\n') && blastwascr) {
			blastwascr = false;
			continue;
		}

		//
		// See if a newline or escape character was received.
		//
		if ((cchar == '\r') || (cchar == '\n') || (cchar == 0x1b)) {
			//
			// If the character is a CR, then it may be followed by an
			// LF which should be paired with the CR.  So remember that
			// a CR was received.
			//
			if (cchar == '\r') {
				blastwascr = 1;
			}

			//
			// Regardless of the line termination character received,
			// put a CR in the receive buffer as a marker telling
			// UARTgets() where the line ends.  We also send an
			// additional LF to ensure that the local terminal echo
			// receives both CR and LF.
			//
			cchar = '\r';
			//scia_msg("\n", 1);
		}

		if (!cRX_BUFFER_FULL) {
			c_pcUARTRxBuffer[c_ui16UARTRxWriteIndex] = cchar;
			ADVANCE_RX_BUFFER_INDEX(c_ui16UARTRxWriteIndex);

			//        		scia_msg(&cchar,1);

		}

	}

	//
	//	    ScibRegs.SCIFFRX.bit.RXFFOVRCLR=1;   // Clear Overflow flag
	//	    ScibRegs.SCIFFRX.bit.RXFFINTCLR=1;   // Clear Interrupt flag

	PieCtrlRegs.PIEACK.all |= M_INT9;       // Issue PIE ack
#else
	char trash;

	while(ScicRegs.SCIFFRX.bit.RXFFST!=0)
	{
		bufc=ScicRegs.SCIRXBUF.all;  // Read data

		if (ATmode&(bufc!=0x0A))
		{

			ATrxmsg[ATcnt2]=bufc;
			ATcnt2++;
			if(bufc==0x0D)
			{
				scia_msg(ATrxmsg,0);
				ATcnt2=0;
				//		  			 atpntflag=1;
				break;

			}
		}

	}

	ScicRegs.SCIFFRX.bit.RXFFOVRCLR=1;   // Clear Overflow flag
	ScicRegs.SCIFFRX.bit.RXFFINTCLR=1;// Clear Interrupt flag

	PieCtrlRegs.PIEACK.all|=M_INT9;// Issue PIE ack
#endif
}

#ifdef __cplusplus
#pragma CODE_SECTION("ramfuncs");
#endif
interrupt void scidRxFifoIsr(void) {

#ifdef UART_BUFFERED
	Uint16 base = 3;
	Uint16 ui16char;
	Uint8 cchar;
	static bool blastwascr = false;
//	UARTIntclear(base); //clear interrupt
	while (ScidRegs.SCIRXST.bit.RXRDY) {
		//read a character
		cchar = (unsigned char) scid_rcv();

//			//backspace
//			if(cchar =='\b')
//			{
//				if(!dRX_BUFFER_EMPTY)
//				{
//					scid_msg("\b \b",3);
//
//                    //
//                    // Decrement the number of characters in the buffer.
//                    //
//                    if(d_ui16UARTRxWriteIndex == 0)
//                    {
//                        d_ui16UARTRxWriteIndex = UART_RX_BUFFER_SIZE - 1;
//                    }
//                    else
//                    {
//                        d_ui16UARTRxWriteIndex--;
//                    }
//
//				}
//				continue;
//			}

		//
		// If this character is LF and last was CR, then just gobble up
		// the character since we already echoed the previous CR and we
		// don't want to store 2 characters in the buffer if we don't
		// need to.
		//
//            if((cchar == '\n') && blastwascr)
//            {
//                blastwascr = false;
//                continue;
//            }

		//
		// See if a newline or escape character was received.
		//
//            if((cchar == '\r') || (cchar == '\n') || (cchar == 0x1b))
//            {
//                //
//                // If the character is a CR, then it may be followed by an
//                // LF which should be paired with the CR.  So remember that
//                // a CR was received.
//                //
//                if(cchar == '\r')
//                {
//                    blastwascr = 1;
//                }
//
//                //
//                // Regardless of the line termination character received,
//                // put a CR in the receive buffer as a marker telling
//                // UARTgets() where the line ends.  We also send an
//                // additional LF to ensure that the local terminal echo
//                // receives both CR and LF.
//                //
//                cchar = '\r';
//                //scia_msg("\n", 1);
//            }

		if (!dRX_BUFFER_FULL) {
			d_pcUARTRxBuffer[d_ui16UARTRxWriteIndex] = cchar;
			ADVANCE_RX_BUFFER_INDEX(d_ui16UARTRxWriteIndex);

//        		scia_msg(&cchar,1);

		}

	}

//
//	    ScibRegs.SCIFFRX.bit.RXFFOVRCLR=1;   // Clear Overflow flag
//	    ScibRegs.SCIFFRX.bit.RXFFINTCLR=1;   // Clear Interrupt flag

	PieCtrlRegs.PIEACK.all |= M_INT9;       // Issue PIE ack
#else
			char trash;

			while(ScidRegs.SCIFFRX.bit.RXFFST!=0)
			{
				bufb=ScidRegs.SCIRXBUF.all;  // Read data

				if (ATmode&(bufb!=0x0A))
				{

					ATrxmsg[ATcnt2]=bufb;
					ATcnt2++;
					if(bufb==0x0D)
					{
						scia_msg(ATrxmsg,0);
						ATcnt2=0;
//		  			 atpntflag=1;
						break;

					}
				}

			}

			ScidRegs.SCIFFRX.bit.RXFFOVRCLR=1;   // Clear Overflow flag
			ScidRegs.SCIFFRX.bit.RXFFINTCLR=1;// Clear Interrupt flag

			PieCtrlRegs.PIEACK.all|=M_INT9;// Issue PIE ack
#endif
}

#ifdef __cplusplus
#pragma CODE_SECTION("ramfuncs");
#endif

void scia_xmit(Uint8 a) {
#ifdef UART_BUFFERED
	if (SciaRegs.SCICTL2.bit.TXRDY) {
		SciaRegs.SCITXBUF.all = a;
	}

#else
	while (SciaRegs.SCICTL2.bit.TXRDY != 1) {}
	SciaRegs.SCITXBUF.all =a;
#endif
}

Uint16 scia_msg(const char *a, Uint16 leng) {
	Uint16 i;
	Uint16 base = 0;
	if (!leng)
		leng = 64;

#ifdef UART_BUFFERED

	atxing = true;
	//SEND CHARACTERS
	for (i = 0; i < leng; i++) {

		if (a[i] == 0x0d) {
			if (!aTX_BUFFER_FULL) {
				a_pcUARTTxBuffer[a_ui16UARTTxWriteIndex] = '\n';
				ADVANCE_TX_BUFFER_INDEX(a_ui16UARTTxWriteIndex);
			} else {
				break;
			}
		}
		if (!aTX_BUFFER_FULL) {
			a_pcUARTTxBuffer[a_ui16UARTTxWriteIndex] = a[i];
			ADVANCE_TX_BUFFER_INDEX(a_ui16UARTTxWriteIndex);
		} else {
			break;
		}
	}

	if (!aTX_BUFFER_EMPTY) {
		UARTPrimeTransmit(base);
	}

	atxing = false;
	return i;
#else
	//SEND CHARACTERS
	for (i=0;i<leng;i++)
	{
		scia_xmit(a[i]);

		if(a[i] == 0x0d)
		{

			SciaRegs.SCITXBUF.all =0x0A;
			return i;
		}

	}
	return i;
#endif
}

Uint16 scia_rcv() {
	if (SciaRegs.SCIRXST.bit.RXRDY) {
		return SciaRegs.SCIRXBUF.bit.SAR;
	}
}

void scib_xmit(Uint8 a) {
#ifdef UART_BUFFERED
	if (ScibRegs.SCICTL2.bit.TXRDY) {
		ScibRegs.SCITXBUF.all = a;
	}

#else
	while (ScibRegs.SCICTL2.bit.TXRDY != 1) {}
	ScibRegs.SCITXBUF.all =a;
#endif
}

Uint16 scib_msg(const char *a, Uint16 leng) {
	Uint16 i;
	Uint16 base = 1;
	if (!leng)
		leng = 64;

#ifdef UART_BUFFERED
	//SEND CHARACTERS
	for (i = 0; i < leng; i++) {

		if (a[i] == 0x0d) {
			if (!bTX_BUFFER_FULL) {
				//b_pcUARTTxBuffer[b_ui16UARTTxWriteIndex] = '\n';
				//ADVANCE_TX_BUFFER_INDEX(b_ui16UARTTxWriteIndex);
			} else {
				break;
			}
		}
		if (!bTX_BUFFER_FULL) {
			b_pcUARTTxBuffer[b_ui16UARTTxWriteIndex] = a[i];
			ADVANCE_TX_BUFFER_INDEX(b_ui16UARTTxWriteIndex);
		} else {
			break;
		}
	}
	if (!bTX_BUFFER_EMPTY) {
		UARTPrimeTransmit(base);
	}
	return i;
#else

	for (i=0;i<leng;i++)
	{

		while (ScibRegs.SCICTL2.bit.TXRDY != 1) {}
		ScibRegs.SCITXBUF.all =a[i];
		if(a[i] == 0x0D)
		{
			return i;
		}

	}
	return i;
#endif

}
Uint16 scib_rcv() {
	if (ScibRegs.SCIRXST.bit.RXRDY) {
		return ScibRegs.SCIRXBUF.bit.SAR;
	}
}

void scic_xmit(Uint8 a) {
#ifdef UART_BUFFERED
	if (ScicRegs.SCICTL2.bit.TXRDY) {
		ScicRegs.SCITXBUF.all = a;
	}

#else
	while (ScicRegs.SCICTL2.bit.TXRDY != 1) {}
	ScicRegs.SCITXBUF.all =a;
#endif
}

Uint16 scic_msg(Uint8* a, Uint8 leng) {
	Uint16 i;
		Uint16 base = 2;
		if (!leng)
			leng = 64;

	#ifdef UART_BUFFERED
		//SEND CHARACTERS
		for (i = 0; i < leng; i++) {

			if (a[i] == 0x0d) {
				if (!cTX_BUFFER_FULL) {
					//c_pcUARTTxBuffer[c_ui16UARTTxWriteIndex] = '\n';
					//ADVANCE_TX_BUFFER_INDEX(c_ui16UARTTxWriteIndex);
				} else {
					break;
				}
			}
			if (!cTX_BUFFER_FULL) {
				c_pcUARTTxBuffer[c_ui16UARTTxWriteIndex] = a[i];
				ADVANCE_TX_BUFFER_INDEX(c_ui16UARTTxWriteIndex);
			} else {
				break;
			}
		}
		if (!cTX_BUFFER_EMPTY) {
			UARTPrimeTransmit(base);
		}
		return i;
	#else

		for (i=0;i<leng;i++)
		{

			while (ScicRegs.SCICTL2.bit.TXRDY != 1) {}
			ScicRegs.SCITXBUF.all =a[i];
			if(a[i] == 0x0D)
			{
				return i;
			}

		}
		return i;
	#endif
}

Uint16 scic_rcv() {
	if (ScicRegs.SCIRXST.bit.RXRDY) {
		return ScicRegs.SCIRXBUF.bit.SAR;
	}
}

#ifdef __cplusplus
#pragma CODE_SECTION("ramfuncs");
#endif
void scid_xmit(Uint8 a) {
#ifdef UART_BUFFERED
	while (ScidRegs.SCICTL2.bit.TXRDY != 1) {
	}
	ScidRegs.SCITXBUF.all = a;

#else
	while (ScidRegs.SCICTL2.bit.TXRDY != 1) {}
	ScidRegs.SCITXBUF.all =a;
#endif
}
#ifdef __cplusplus
#pragma CODE_SECTION("ramfuncs");
#endif

Uint16 scid_msg(const char *a, Uint16 leng) {
	Uint16 i;
	Uint16 base = 3;
	if (!leng)
		leng = 64;

#ifdef UART_BUFFERED
	//SEND CHARACTERS

	for (i = 0; i < leng; i++) {

		if (a[i] == 0x00) {
			if (!dTX_BUFFER_FULL) {
				//b_pcUARTTxBuffer[b_ui16UARTTxWriteIndex] = '\n';
				//ADVANCE_TX_BUFFER_INDEX(b_ui16UARTTxWriteIndex);
			} else {
				break;
			}
		}
		if (!dTX_BUFFER_FULL) {
			d_pcUARTTxBuffer[d_ui16UARTTxWriteIndex] = a[i];
			ADVANCE_TX_BUFFER_INDEX(d_ui16UARTTxWriteIndex);
		} else {
			break;
		}
	}
	if (!dTX_BUFFER_EMPTY) {

		UARTPrimeTransmit(base);

	}
	return i;
#else

	for (i=0;i<leng;i++)
	{

		while (ScidRegs.SCICTL2.bit.TXRDY != 1) {}
		ScidRegs.SCITXBUF.all =a[i];
		if(a[i] == 0x0D)
		{
			return i;
		}

	}
	return i;
#endif

}

#ifdef __cplusplus
#pragma CODE_SECTION("ramfuncs");
#endif

Uint16 scid_rcv() {
	if (ScidRegs.SCIRXST.bit.RXRDY) {
		return ScidRegs.SCIRXBUF.bit.SAR;
	}
}

#ifdef __cplusplus
#pragma CODE_SECTION("ramfuncs");
#endif

void UARTIntclear(Uint16 ui16Base) {
	if (!ui16Base) {
		//clear uart interrupt
		if (SciaRegs.SCIRXST.bit.RXRDY | SciaRegs.SCIRXST.bit.BRKDT) {
			SciaRegs.SCICTL1.bit.SWRESET = 0;
			__asm(" nop");
			__asm(" nop");
			__asm(" nop");
			__asm(" nop");
			SciaRegs.SCICTL1.bit.SWRESET = 1;
		}

		if (SciaRegs.SCIFFTX.bit.TXFFINT)
			SciaRegs.SCIFFTX.bit.TXFFINTCLR = 1;   // Clear Interrupt flag

		if (SciaRegs.SCIFFRX.bit.RXFFINT)
			SciaRegs.SCIFFRX.bit.RXFFINTCLR = 1;   // Clear Interrupt flag
	} else if (ui16Base == 1) {

		//clear uart interrupt
		if (ScibRegs.SCIRXST.bit.RXRDY | ScibRegs.SCIRXST.bit.BRKDT) {
			ScibRegs.SCICTL1.bit.SWRESET = 0;
			__asm(" nop");
			__asm(" nop");
			__asm(" nop");
			__asm(" nop");
			ScibRegs.SCICTL1.bit.SWRESET = 1;
		}

		if (ScibRegs.SCIFFTX.bit.TXFFINT)
			ScibRegs.SCIFFTX.bit.TXFFINTCLR = 1;   // Clear Interrupt flag

		if (ScibRegs.SCIFFRX.bit.RXFFINT)
			ScibRegs.SCIFFRX.bit.RXFFINTCLR = 1;   // Clear Interrupt flag
	}

	else if (ui16Base == 2) {

			//clear uart interrupt
			if (ScicRegs.SCIRXST.bit.RXRDY | ScicRegs.SCIRXST.bit.BRKDT) {
				ScicRegs.SCICTL1.bit.SWRESET = 0;
				__asm(" nop");
				__asm(" nop");
				__asm(" nop");
				__asm(" nop");
				ScicRegs.SCICTL1.bit.SWRESET = 1;
			}

			if (ScicRegs.SCIFFTX.bit.TXFFINT)
				ScicRegs.SCIFFTX.bit.TXFFINTCLR = 1;   // Clear Interrupt flag

			if (ScicRegs.SCIFFRX.bit.RXFFINT)
				ScicRegs.SCIFFRX.bit.RXFFINTCLR = 1;   // Clear Interrupt flag
		}

	else if (ui16Base == 3) {

		//clear uart interrupt
		if (ScidRegs.SCIRXST.bit.RXRDY | ScidRegs.SCIRXST.bit.BRKDT) {
			ScidRegs.SCICTL1.bit.SWRESET = 0;
			__asm(" nop");
			__asm(" nop");
			__asm(" nop");
			__asm(" nop");
			ScidRegs.SCICTL1.bit.SWRESET = 1;
		}

		if (ScidRegs.SCIFFTX.bit.TXFFINT)
			ScidRegs.SCIFFTX.bit.TXFFINTCLR = 1;   // Clear Interrupt flag

		if (ScidRegs.SCIFFRX.bit.RXFFINT)
			ScidRegs.SCIFFRX.bit.RXFFINTCLR = 1;   // Clear Interrupt flag
	}

}

#ifdef UART_BUFFERED
//*****************************************************************************
//
//! Determines whether the ring buffer whose pointers and size are provided
//! is full or not.
//!
//! \param pui16Read points to the read index for the buffer.
//! \param pui16Write points to the write index for the buffer.
//! \param ui16Size is the size of the buffer in bytes.
//!
//! This function is used to determine whether or not a given ring buffer is
//! full.  The structure of the code is specifically to ensure that we do not
//! see warnings from the compiler related to the order of volatile accesses
//! being undefined.
//!
//! \return Returns \b true if the buffer is full or \b false otherwise.
//
//*****************************************************************************
IsBufferFull(volatile Uint16 *pui16Read, volatile Uint16 *pui16Write,
		Uint16 ui16Size) {
	Uint16 ui16Write;
	Uint16 ui16Read;

	ui16Write = *pui16Write;
	ui16Read = *pui16Read;

	return ((((ui16Write + 1) % ui16Size) == ui16Read) ? true : false);
}

//*****************************************************************************
//
//! Determines whether the ring buffer whose pointers and size are provided
//! is empty or not.
//!
//! \param pui16Read points to the read index for the buffer.
//! \param pui16Write points to the write index for the buffer.
//!
//! This function is used to determine whether or not a given ring buffer is
//! empty.  The structure of the code is specifically to ensure that we do not
//! see warnings from the compiler related to the order of volatile accesses
//! being undefined.
//!
//! \return Returns \b true if the buffer is empty or \b false otherwise.
//
//*****************************************************************************
IsBufferEmpty(volatile Uint16 *pui16Read, volatile Uint16 *pui16Write) {
	Uint16 ui16Write;
	Uint16 ui16Read;

	ui16Write = *pui16Write;
	ui16Read = *pui16Read;

	return ((ui16Write == ui16Read) ? true : false);
}

//*****************************************************************************
//
//! Determines the number of bytes of data contained in a ring buffer.
//!
//! \param pui16Read points to the read index for the buffer.
//! \param pui16Write points to the write index for the buffer.
//! \param ui16Size is the size of the buffer in bytes.
//!
//! This function is used to determine how many bytes of data a given ring
//! buffer currently contains.  The structure of the code is specifically to
//! ensure that we do not see warnings from the compiler related to the order
//! of volatile accesses being undefined.
//!
//! \return Returns the number of bytes of data currently in the buffer.
//
//*****************************************************************************

static Uint16 GetBufferCount(volatile Uint16 *pui16Read,
		volatile Uint16 *pui16Write, Uint16 ui16Size) {
	Uint16 ui16Write;
	Uint16 ui16Read;

	ui16Write = *pui16Write;
	ui16Read = *pui16Read;

	return ((ui16Write >= ui16Read) ?
			(ui16Write - ui16Read) : (ui16Size - (ui16Read - ui16Write)));
}

//*****************************************************************************
//
// Take as many bytes from the transmit buffer as we have space for and move
// them into the UART transmit FIFO.
//
//*****************************************************************************

#ifdef __cplusplus
#pragma CODE_SECTION("ramfuncs");
#endif
void UARTPrimeTransmit(Uint16 ui16Base) {

	if (!ui16Base) {
		//
		// Do we have any data to transmit?
		//
		if (!aTX_BUFFER_EMPTY) {
			//

			//
			// Yes - take some characters out of the transmit buffer and feed
			// them to the UART transmit FIFO.
			//
			while (SciaRegs.SCICTL2.bit.TXRDY && !aTX_BUFFER_EMPTY) {
				scia_xmit(a_pcUARTTxBuffer[a_ui16UARTTxReadIndex]);
				ADVANCE_TX_BUFFER_INDEX(a_ui16UARTTxReadIndex);
			}
		}

	} else if (ui16Base == 1) {
		//
		// Do we have any data to transmit?
		//
		if (!bTX_BUFFER_EMPTY) {
			//

			//
			// Yes - take some characters out of the transmit buffer and feed
			// them to the UART transmit FIFO.
			//
			while (ScibRegs.SCICTL2.bit.TXRDY && !bTX_BUFFER_EMPTY) {
				scib_xmit(b_pcUARTTxBuffer[b_ui16UARTTxReadIndex]);
				ADVANCE_TX_BUFFER_INDEX(b_ui16UARTTxReadIndex);
			}
		}
	}

	else if (ui16Base == 2) {
			//
			// Do we have any data to transmit?
			//
			if (!cTX_BUFFER_EMPTY) {
				//

				//
				// Yes - take some characters out of the transmit buffer and feed
				// them to the UART transmit FIFO.
				//
				while (ScicRegs.SCICTL2.bit.TXRDY && !cTX_BUFFER_EMPTY) {
					scic_xmit(c_pcUARTTxBuffer[c_ui16UARTTxReadIndex]);
					ADVANCE_TX_BUFFER_INDEX(c_ui16UARTTxReadIndex);
				}
			}
		}


	else if (ui16Base == 3) {
		//
		// Do we have any data to transmit?
		//
		if (!dTX_BUFFER_EMPTY) {
			//

			//
			// Yes - take some characters out of the transmit buffer and feed
			// them to the UART transmit FIFO.
			//
			GPIO_WritePin(22, 1);
			while (!dTX_BUFFER_EMPTY) {
				scid_xmit(d_pcUARTTxBuffer[d_ui16UARTTxReadIndex]);
				ADVANCE_TX_BUFFER_INDEX(d_ui16UARTTxReadIndex);
			}
			while (ScidRegs.SCICTL2.bit.TXEMPTY != 1) {
			}

			GPIO_WritePin(22, 0);
		}

	}

}
#ifdef __cplusplus
#pragma CODE_SECTION("ramfuncs");
#endif

//*****************************************************************************
//
//! A simple UART based get string function, with some line processing.
//!
//! \param pcBuf points to a buffer for the incoming string from the UART.
//! \param ui16Len is the length of the buffer for storage of the string,
//! including the trailing 0.
//!
//! This function will receive a string from the UART input and store the
//! characters in the buffer pointed to by \e pcBuf.  The characters will
//! continue to be stored until a termination character is received.  The
//! termination characters are CR, LF, or ESC.  A CRLF pair is treated as a
//! single termination character.  The termination characters are not stored in
//! the string.  The string will be terminated with a 0 and the function will
//! return.
//!
//! In both buffered and unbuffered modes, this function will block until
//! a termination character is received.  If non-blocking operation is required
//! in buffered mode, a call to UARTPeek() may be made to determine whether
//! a termination character already exists in the receive buffer prior to
//! calling UARTgets().
//!
//! Since the string will be null terminated, the user must ensure that the
//! buffer is sized to allow for the additional null character.
//!
//! \return Returns the count of characters that were stored, not including
//! the trailing 0.
//
//*****************************************************************************
Uint16 UARTgets(char *pcBuf, Uint16 Len, Uint16 base) {

	Uint16 Count = 0;
	Uint16 once = 0;
	unsigned char cChar;
	if (!Len) {
		return 0;
	}

	//
	// Adjust the length back by 1 to leave space for the trailing
	// null terminator.
	//
	Len--;

	if (1)
//    if((b_pcUARTRxBuffer[b_ui16UARTRxWriteIndex-1]=='\r'&base==1)|(a_pcUARTRxBuffer[a_ui16UARTRxWriteIndex-1]=='\r'&base==0))
	{

		//
		// Process characters until a newline is received.
		//
		while (1) {

			if (!base) {
				//
				// Read the next character from the receive buffer.
				//

				if (!aRX_BUFFER_EMPTY) {

					cChar = a_pcUARTRxBuffer[a_ui16UARTRxReadIndex];
					ADVANCE_RX_BUFFER_INDEX(a_ui16UARTRxReadIndex);

					//
					// Process the received character as long as we are not at the end
					// of the buffer.  If the end of the buffer has been reached then
					// all additional characters are ignored until a newline is
					// received.
					//
					if (Count < Len) {
						//
						// Store the character in the caller supplied buffer.
						//
						pcBuf[Count] = cChar;

						//
						// Increment the count of characters received.
						//
						Count++;
					} else
						break;
					//
					// See if a newline or escape character was received.
					//
					if ((cChar == '\r') || (cChar == '\n') || (cChar == 0x1b)) {
						//
						// Stop processing the input and end the line.
						//
						break;
					}

				} else
					break;

			} else if (base == 1) {

				//
				// Read the next character from the receive buffer.
				//
				if (!bRX_BUFFER_EMPTY) {
					cChar = b_pcUARTRxBuffer[b_ui16UARTRxReadIndex];
					ADVANCE_RX_BUFFER_INDEX(b_ui16UARTRxReadIndex);

					//
					// Process the received character as long as we are not at the end
					// of the buffer.  If the end of the buffer has been reached then
					// all additional characters are ignored until a newline is
					// received.
					//
					if (Count < Len) {
						//
						// Store the character in the caller supplied buffer.
						//
						pcBuf[Count] = cChar;

						//
						// Increment the count of characters received.
						//
						Count++;
					} else
						break;
					//
					// See if a newline or escape character was received.
					//
					if ((cChar == '\r') || (cChar == '\n') || (cChar == 0x1b)) {
						//
						// Stop processing the input and end the line.
						//
						break;
					}
				} else
					break;

			}

			 else if (base == 2) {

							//
							// Read the next character from the receive buffer.
							//
							if (!cRX_BUFFER_EMPTY) {
								cChar = c_pcUARTRxBuffer[c_ui16UARTRxReadIndex];
								ADVANCE_RX_BUFFER_INDEX(c_ui16UARTRxReadIndex);

								//
								// Process the received character as long as we are not at the end
								// of the buffer.  If the end of the buffer has been reached then
								// all additional characters are ignored until a newline is
								// received.
								//
								if (Count < Len) {
									//
									// Store the character in the caller supplied buffer.
									//
									pcBuf[Count] = cChar;

									//
									// Increment the count of characters received.
									//
									Count++;
								} else
									break;
								//
								// See if a newline or escape character was received.
								//
								if ((cChar == '\r') || (cChar == '\n') || (cChar == 0x1b)) {
									//
									// Stop processing the input and end the line.
									//
									break;
								}
							} else
								break;

						}

			else if (base == 3) {
				if (!once) {
					Len = Len + 1;
					once = 1;

				}

				//
				// Read the next character from the receive buffer.
				//
				if (!dRX_BUFFER_EMPTY) {

					//
					// Process the received character as long as we are not at the end
					// of the buffer.  If the end of the buffer has been reached then
					// all additional characters are ignored until a newline is
					// received.
					//
					if (Count < Len) {

						cChar = d_pcUARTRxBuffer[d_ui16UARTRxReadIndex];
						ADVANCE_RX_BUFFER_INDEX(d_ui16UARTRxReadIndex);
						//
						// Store the character in the caller supplied buffer.
						//
						pcBuf[Count] = cChar;
						//
						// Increment the count of characters received.
						//
						Count++;
					} else
						break;

				} else
					break;

			}
		}

	}

	//
	// Add a null termination to the string.
	//
	pcBuf[Count] = 0;

	//
	// Return the count of int8_ts in the buffer, not counting the trailing 0.
	//
	return (Count);
}

//*****************************************************************************
//
//! Read a single character from the UART, blocking if necessary.
//!
//! This function will receive a single character from the UART and store it at
//! the supplied address.
//!
//! In both buffered and unbuffered modes, this function will block until a
//! character is received.  If non-blocking operation is required in buffered
//! mode, a call to UARTRxAvail() may be made to determine whether any
//! characters are currently available for reading.
//!
//! \return Returns the character read.
//
//*****************************************************************************

unsigned char UARTgetc(Uint16 base) {
	unsigned char cChar;

	if (!base) {

		//
		// Wait for a character to be received.
		//
		if (aRX_BUFFER_EMPTY) {
			return 0;

		}

		//
		// Read a character from the buffer.
		//
		cChar = a_pcUARTRxBuffer[a_ui16UARTRxReadIndex];
		ADVANCE_RX_BUFFER_INDEX(a_ui16UARTRxReadIndex);

	}

	else if (base == 1) {

		//
		// Wait for a character to be received.
		//
		if (bRX_BUFFER_EMPTY) {
			return 0;
		}

		//
		// Read a character from the buffer.
		//
		cChar = b_pcUARTRxBuffer[b_ui16UARTRxReadIndex];
		ADVANCE_RX_BUFFER_INDEX(b_ui16UARTRxReadIndex);
	}

	else if (base == 2) {

		//
		// Wait for a character to be received.
		//
		if (cRX_BUFFER_EMPTY) {
			return 0;
		}

		//
		// Read a character from the buffer.
		//
		cChar = c_pcUARTRxBuffer[c_ui16UARTRxReadIndex];
		ADVANCE_RX_BUFFER_INDEX(c_ui16UARTRxReadIndex);
	}


	else if (base == 3) {

		//
		// Wait for a character to be received.
		//
		if (dRX_BUFFER_EMPTY) {
			return 0;
		}

		//
		// Read a character from the buffer.
		//
		cChar = d_pcUARTRxBuffer[d_ui16UARTRxReadIndex];
		ADVANCE_RX_BUFFER_INDEX(d_ui16UARTRxReadIndex);
	}
	//
	// Return the character to the caller.
	//
	return (cChar);
}

//*****************************************************************************
//
//! Looks ahead in the receive buffer for a particular character.
//!
//! \param ucChar is the character that is to be searched for.
//!
//! This function, available only when the module is built to operate in
//! buffered mode using \b UART_BUFFERED, may be used to look ahead in the
//! receive buffer for a particular character and report its position if found.
//! It is typically used to determine whether a complete line of user input is
//! available, in which case ucChar should be set to CR ('\\r') which is used
//! as the line end marker in the receive buffer.
//!
//! \return Returns -1 to indicate that the requested character does not exist
//! in the receive buffer.  Returns a non-negative number if the character was
//! found in which case the value represents the position of the first instance
//! of \e ucChar relative to the receive buffer read pointer.
//
//*****************************************************************************
Uint16 UARTPeek(unsigned char ucChar, Uint16 base) {
	Uint16 iCount;
	Uint16 iAvail;
	uint32_t ui16ReadIndex;

	if (!base) {
		//
		// How many characters are there in the receive buffer?
		//

		iAvail = (Uint16) aRX_BUFFER_USED;
		ui16ReadIndex = a_ui16UARTRxReadIndex;

		//
		// Check all the unread characters looking for the one passed.
		//
		for (iCount = 0; iCount < iAvail; iCount++) {
			if (a_pcUARTRxBuffer[ui16ReadIndex] == ucChar) {
				//
				// We found it so return the index
				//
				if (iCount == 0)
					iCount = 1;
				return (iCount);
			} else {
				//
				// This one didn't match so move on to the next character.
				//
				ADVANCE_RX_BUFFER_INDEX(ui16ReadIndex);
			}
		}
	} else if (base == 1) {
		//
		// How many characters are there in the receive buffer?
		//

		iAvail = (Uint16) bRX_BUFFER_USED;
		ui16ReadIndex = b_ui16UARTRxReadIndex;

		//
		// Check all the unread characters looking for the one passed.
		//
		for (iCount = 0; iCount < iAvail; iCount++) {
			if (b_pcUARTRxBuffer[ui16ReadIndex] == ucChar) {
				//
				// We found it so return the index
				//
				if (iCount == 0)
					iCount = 1;
				return (iCount);
			} else {
				//
				// This one didn't match so move on to the next character.
				//
				ADVANCE_RX_BUFFER_INDEX(ui16ReadIndex);
			}
		}
	}

	 else if (base == 2) {
			//
			// How many characters are there in the receive buffer?
			//

			iAvail = (Uint16) cRX_BUFFER_USED;
			ui16ReadIndex = c_ui16UARTRxReadIndex;

			//
			// Check all the unread characters looking for the one passed.
			//
			for (iCount = 0; iCount < iAvail; iCount++) {
				if (c_pcUARTRxBuffer[ui16ReadIndex] == ucChar) {
					//
					// We found it so return the index
					//
					if (iCount == 0)
						iCount = 1;
					return (iCount);
				} else {
					//
					// This one didn't match so move on to the next character.
					//
					ADVANCE_RX_BUFFER_INDEX(ui16ReadIndex);
				}
			}
		}

	else if (base == 3) {
		//
		// How many characters are there in the receive buffer?
		//

		iAvail = (Uint16) dRX_BUFFER_USED;
		ui16ReadIndex = d_ui16UARTRxReadIndex;

		//
		// Check all the unread characters looking for the one passed.
		//
		for (iCount = 0; iCount < iAvail; iCount++) {
			if (d_pcUARTRxBuffer[ui16ReadIndex] == ucChar) {
				//
				// We found it so return the index
				//
				if (iCount == 0)
					iCount = 1;
				return (iCount);
			} else {
				//
				// This one didn't match so move on to the next character.
				//
				ADVANCE_RX_BUFFER_INDEX(ui16ReadIndex);
			}
		}
	}

	//
	// If we drop out of the loop, we didn't find the character in the receive
	// buffer.
	//
	return (0);
}
#endif

