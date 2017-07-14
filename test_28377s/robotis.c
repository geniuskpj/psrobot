/*
 * robotis.c
 *
 *  Created on: 2016. 5. 11.
 *      Author: KJC
 */
#include "F2837xD_device.h"     // F2837xD Headerfile Include File
#include "F2837xD_Examples.h"   // F2837xD Examples Include File
typedef unsigned char   Uint8;



void error(void);
void senddx(Uint8 id, Uint8 length,Uint8 param[]);
void readdx(Uint8 id,Uint8 param1,Uint8 param2);
void ping(Uint8 id);
void reset(Uint8 id);

void led(Uint8 id, bool ono);
void mvf(Uint8 id,float goalpositon,float vel);
void mv_p(Uint8 id,float goalpositon);
void mv_s(Uint8 id,float vel);
void setoffset(Uint8 id,int16 offset);
void setcwlimit(Uint8 id,int16 limit);
void setccwlimit(Uint8 id,int16 limit);
void bulkreaddx(Uint8 startadress);
void syncmv(Uint8 modnum,float goal[4]);
//void retfowr(Uint8 id,bool ono);
void motorstop();
//float gearrt[8]={1,44.0/20.0,1,72.0/19.0,1,44.0/20.0,1,72.0/19.0};
float gearrt[8]={1,5.32,1,72.0/19.0,1,44.0/20.0,1,72.0/19.0};




extern Uint16 sdataA[30];    // Send data for SCI-A
extern Uint16 sdataA2[10];    // Send data for SCI-A
extern Uint16 releng=6;
extern int16 dir[8],oset[8];
extern Uint16 p[5];
extern Uint16 obs[9];





void error(void)
{
   asm("     ESTOP0"); // Test failed!! Stop!
    for (;;);
}


void readdx(Uint8 id,Uint8 param1,Uint8 param2)
{
	Uint8 cksum;

/*
	scic_xmit(0xFF);
	scic_xmit(0xFF);
	scic_xmit(id);
	scic_xmit(0x04);
	scic_xmit(0x02);
	scic_xmit(param1);
	scic_xmit(param2);
	*/
	sdataA[0]=0xFF;
	sdataA[1]=0xFF;
	sdataA[2]=id;
	sdataA[3]=0x04;
	sdataA[4]=0x02;
	sdataA[5]=param1;
	sdataA[6]=param2;




	cksum=(~((id+0x04+0x02+param1+param2)&0xFF))&0xFF;
	//scic_xmit(cksum);
	sdataA[7]=cksum;
//	scic_msg((Uint8*)sdataA,8);
	scid_msg((Uint8*)sdataA,8);
//	ScicRegs.SCIFFRX.bit.RXFFIL = 6+(Uint16)param2;
//	releng=ScicRegs.SCIFFRX.bit.RXFFIL;
	fflush(sdataA);


}

void senddx(Uint8 id, Uint8 length,Uint8 param[])
{
	Uint8 checksum;
	//Uint8 pl=sizeof(param)/sizeof(param[0]);
	Uint8 i,ps=0;
	for(i=0;i<length-2;i++)
	{
		ps=ps+param[i];
	}

	sdataA[0]=0xFF;
	sdataA[1]=0xFF;
	sdataA[2]=id;
	sdataA[3]=length;
	sdataA[4]=0x03; // 1개쓸떄
//	sdataA[4]=0x83; // 여러개쓸떄
	for(i=0;i<length-2;i++)
		{
			//scic_xmit(param[i]);
			sdataA[5+i]=param[i];
		}
	checksum=(~((id+length+sdataA[4]+ps)&0xFF))&0xFF;
	sdataA[5+length-2]=checksum;
	scid_msg((Uint8*)sdataA,6+length-2);
	//scic_xmit(checksum);
//	ScicRegs.SCIFFRX.bit.RXFFIL = 6;
//	releng=6;
}


void ping(Uint8 id)
{
	Uint8 cksum;
	Uint16 sdatasum;
	int i;

	//scic_xmit(0xFF);
	//scic_xmit(0xFF);
	//scic_xmit(id);
	//scic_xmit(0x02);
	//scic_xmit(0x01);
	//scic_xmit(0xFB);
	sdataA[0]=0xFF;
	sdataA[1]=0xFF;
	sdataA[2]=id;
	sdataA[3]=0x02;
	sdataA[4]=0x01;

	for(i=2;i<5;i++)
			{
				sdatasum=sdatasum+sdataA[i];
			}
	cksum=(~(sdatasum&0xFF))&0xFF;
	sdataA[5]=cksum;


//	scic_msg((Uint8*)sdataA,6);
	scid_msg((Uint8*)sdataA,6);




//	ScicRegs.SCIFFRX.bit.RXFFIL = 6;
//	releng=6;
}

void reset(Uint8 id)
{
	scic_xmit(0xFF);
	scic_xmit(0xFF);
	scic_xmit(id);
	scic_xmit(0x02);
	scic_xmit(0x06);
	scic_xmit(0xF7);
	ScicRegs.SCIFFRX.bit.RXFFIL = 6;
	releng=6;
}


void bulkreaddx(Uint8 startadress)
{
	Uint8 cksum;
	Uint16 sdatasum;
	int i;


	sdataA[0]=0xFF;
	sdataA[1]=0xFF;
	sdataA[2]=0xFE;
	sdataA[3]=3*4+3;
	sdataA[4]=0x92;
	sdataA[5]=0x00;

	scic_msg((Uint8*)sdataA,6);
	for(i=2;i<6;i++)
			{
				sdatasum=sdatasum+sdataA[i];
			}


	for(i=0;i<4;i++)
	{
		sdataA[3*i]=0x02;
		sdataA[3*i+1]=2*(i+1);
		sdataA[3*i+2]=startadress;
	}

	for(i=0;i<12;i++)
	{
		sdatasum=sdatasum+sdataA[i];
	}

	cksum=(~(sdatasum&0xFF))&0xFF;
	//scic_xmit(cksum);
	sdataA[12]=cksum;
	scic_msg((Uint8*)sdataA,13);
	DELAY_US(10);

//	// when motor id 6,8 connected
//	sdataA[0]=0xFF;
//	sdataA[1]=0xFF;
//	sdataA[2]=0xFE;
//	sdataA[3]=3*2+3;
//	sdataA[4]=0x92;
//	sdataA[5]=0x00;
//
//	for(i=0;i<2;i++)
//	{
//		sdataA[3*i+6]=0x02;
//		sdataA[3*i+7]=2*(i+3);
//		sdataA[3*i+8]=startadress;
//	}
//
//	for(i=2;i<12;i++)
//	{
//		sdatasum=sdatasum+sdataA[i];
//	}
//
//	cksum=(~(sdatasum&0xFF))&0xFF;
//	//scic_xmit(cksum);
//	sdataA[12]=cksum;
//	scic_msg((Uint8*)sdataA,13);
//	//when motor id 6,8 connected


	ScicRegs.SCIFFRX.bit.RXFFIL = 8;
	releng=ScicRegs.SCIFFRX.bit.RXFFIL;
}






void led(Uint8 id,bool ono)
{


	Uint8 p[2]={0x19,0x00};
	if (ono ==true)
	{
		p[1]=0x01;
	}
		senddx(id,0x04,p);

}

void retfowr(Uint8 id,bool ono)
{


	Uint8 p[2]={0x10,0x01};
	if (ono ==true)
	{
		p[1]=0x02;
	}
		senddx(id,0x04,p);

}



void mvf(Uint8 id,float goalposition,float vel)
{
//	ScicRegs.SCIFFTX.bit.TXFFIL = 11;
	int16 psint;
	psint=dir[id-1]*((int16)(goalposition/360*4095*gearrt[id-1]))+oset[id-1];
//	if (psint<0)
//	{psint=psint+4095;}
	obs[id]=psint;
	p[0]=0x1E;
	p[1]=(Uint16)(psint&0xFF);
	p[2]=(Uint16)(psint>>8);
	//p[1]=(Uint16)(goalposition/360*4095)&0xFF;
	//p[2]=(Uint16)(goalposition/360*4095)>>8;
	p[3]=(Uint16)(vel)&0xFF;
	p[4]=(Uint16)(vel)>>8;

	senddx(id,0x07,(Uint8 *)p);

}

void mv_p(Uint8 id,float goalposition)
{
	ScicRegs.SCIFFTX.bit.TXFFIL =9;
	int16 psint;
	psint=dir[id-1]*((int16)(goalposition/360*4095)-2048)+oset[id-1];
	if (psint<0)
	{psint=psint+4095;}
	obs[id]=psint;
	p[0]=0x1E;
	p[1]=(Uint16)(psint&0xFF);
	p[2]=(Uint16)(psint>>8);

	senddx(id,0x05,(Uint8 *)p);

}

void mv_s(Uint8 id,float vel)
{
	ScicRegs.SCIFFTX.bit.TXFFIL = 9;

	p[0]=0x20;
	if(vel>=0)
	{
	p[1]=(Uint16)(vel/6/0.114)&0xFF;
	p[2]=(Uint16)(vel/6/0.114)>>8;
	}
	else
	{
		p[1]=(Uint16)(-vel/6/0.114+1024)&0xFF;
		p[2]=(Uint16)(-vel/6/0.114+1024)>>8;
	}
	senddx(id,0x05,(Uint8 *)p);

}

void setoffset(Uint8 id,int16 offset)
{
	ScicRegs.SCIFFTX.bit.TXFFIL = 9;

	p[0]=0x14; 	//주소
	p[1]=offset&0xFF;//하위바이트
	p[2]=offset>>8;//상위바이트

	senddx(id,0x05,(Uint8 *)p);


	ScicRegs.SCIFFRX.bit.RXFFIL = 6;
	releng=6;
}

void setcwlimit(Uint8 id,int16 limit)
{
	ScicRegs.SCIFFTX.bit.TXFFIL = 9;
	p[0]=0x06; 	//주소
	p[1]=limit&0xFF;//하위바이트
	p[2]=limit>>8;//상위바이트

	senddx(id,0x05,(Uint8 *)p);



	ScicRegs.SCIFFRX.bit.RXFFIL = 6;
	releng=6;
}

void setccwlimit(Uint8 id,int16 limit)
{
	ScicRegs.SCIFFTX.bit.TXFFIL = 9;

	p[0]=0x08; 	//주소
	p[1]=limit&0xFF;//하위바이트
	p[2]=limit>>8;//상위바이트

	senddx(id,0x05,(Uint8 *)p);


	ScicRegs.SCIFFRX.bit.RXFFIL = 6;
	releng=6;
}

void syncmv(Uint8 modnum,float goal[4])
{
	Uint8 checksum;
	Uint16 sdatasum=0;

	int16 gint[8];
	float velf[4]={10/0.114*44/20/2,10/0.114*72/19/2,10/0.114*44/20/2,10/0.114*72/19/2}; //원래 100
	Uint8 i;

		for(i=0;i<modnum;i++)
		{
			gint[i]=dir[(i+1)*2-1]*((int16)(goal[i]/360*4095)*gearrt[(i+1)*2-1])+oset[(i+1)*2-1];
		}

	ScicRegs.SCIFFTX.bit.TXFFIL = 0;
	sdataA[0]=0xFF;
	sdataA[1]=0xFF;
	sdataA[2]=0xFE;
	sdataA[3]=5*modnum+4;
	sdataA[4]=0x83;
	sdataA[5]=0x1E;
	sdataA[6]=0x04;
	sdataA[7]=2;
	sdataA[8]=(Uint16)gint[0]&0xFF;
	sdataA[9]=(Uint16)gint[0]>>8;
	sdataA[10]=(Uint16)velf[0]&0xFF;
	sdataA[11]=(Uint16)velf[0]>>8;
	sdataA[12]=4;
	sdataA[13]=(Uint16)gint[1]&0xFF;
	scic_msg((Uint8*)sdataA,14);
	for(i=2;i<14;i++)
		{
			sdatasum=sdatasum+sdataA[i];
		}
	sdataA[0]=(Uint16)gint[1]>>8;
	sdataA[1]=(Uint16)velf[1]&0xFF;
	sdataA[2]=(Uint16)velf[1]>>8;
	sdataA[3]=6;
	sdataA[4]=(Uint16)gint[2]&0xFF;
	sdataA[5]=(Uint16)gint[2]>>8;
	sdataA[6]=(Uint16)velf[2]&0xFF;
	sdataA[7]=(Uint16)velf[2]>>8;
	sdataA[8]=8;
	sdataA[9]=(Uint16)gint[3]&0xFF;
	sdataA[10]=(Uint16)gint[3]>>8;
	sdataA[11]=(Uint16)velf[3]&0xFF;
	sdataA[12]=(Uint16)velf[3]>>8;

	for(i=0;i<13;i++)
			{
				sdatasum=sdatasum+sdataA[i];
			}
	checksum=(~(sdatasum&0xFF))&0xFF;
//	sdataA[5*(modnum-1)+12]=checksum;
//	scic_msg((Uint8*)sdataA,5*(modnum-1)+12);
	sdataA[13]=checksum;


	scic_msg((Uint8*)sdataA,14);
	DELAY_US(50);

}

