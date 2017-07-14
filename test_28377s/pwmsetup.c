#include "F2837xD_device.h"     // F2837xD Headerfile Include File
#include "F2837xD_Examples.h"   // F2837xD Examples Include File
#include "serial.h"
typedef unsigned char   Uint8;



extern Uint16 a_ui16UARTRxWriteIndex,a_ui16UARTRxReadIndex,b_ui16UARTRxWriteIndex,b_ui16UARTRxReadIndex,c_ui16UARTRxWriteIndex,
c_ui16UARTRxReadIndex,d_ui16UARTRxWriteIndex,d_ui16UARTRxReadIndex;

extern Uint16 b_ui16UARTTxWriteIndex,b_ui16UARTTxReadIndex;





#define EPWM1_TIMER_TBPRD  2000  // Period register
#define EPWM1_MAX_CMPA     1950
#define EPWM1_MIN_CMPA       50
#define EPWM1_MAX_CMPB     1950
#define EPWM1_MIN_CMPB       50

#define EPWM2_TIMER_TBPRD  2000  // Period register
#define EPWM2_MAX_CMPA     1950
#define EPWM2_MIN_CMPA       50
#define EPWM2_MAX_CMPB     1950
#define EPWM2_MIN_CMPB       50


#define EPWM3_TIMER_TBPRD  2000  // Period register
#define EPWM3_MAX_CMPA     1950
#define EPWM3_MIN_CMPA       50
#define EPWM3_MAX_CMPB     1950
#define EPWM3_MIN_CMPB       50

#define EPWM4_TIMER_TBPRD  2000  // Period register
#define EPWM4_MAX_CMPA     1950
#define EPWM4_MIN_CMPA       50
#define EPWM4_MAX_CMPB     1950
#define EPWM4_MIN_CMPB       50

#define PWM_clk 2  //hz

#ifdef _FLASH
#define PWM_period 195312/(2*PWM_clk)*19/20
#else

#define PWM_period 195312/2/(2*PWM_clk)

#endif



#define CPU_clk 200e6
#define PWM_clk2 100
#define TPRD CPU_clk/(2*PWM_clk2)




void InitEPwm1Example(void);
void InitEPwm2Example(void);
void InitEPwm3Example(void);
void InitEPwm4Example(void);
void InitEPwm7Example(void);
void InitEPwmvariable(void);

__interrupt void epwm1_isr(void);
__interrupt void epwm2_isr(void);
__interrupt void epwm3_isr(void);
__interrupt void epwm4_isr(void);
__interrupt void epwm7_isr(void);
Uint16 period=PWM_period;
Uint16 ena,enar,enal,duty16,dir1,dir2,cmd2,rst,slp;
Uint32 pwmtimer1=0;
float duty;
float desv=0;
float desvl=0;
float desvr=0;
extern bool mt;
extern float traj[8];

Uint16 numout=0;
char *printbuf;
char *tempa;
char *cmdbuf;
char *cmdbuf2;
char *btxbuf;
char *mxbuf;
char *mxrbuf;

extern char *dtxbuf;
extern bool atxing;
extern char * ustrstr(const char *s1, const char *s2);
extern int usprintf(char * restrict s, const char *format, ...);
extern char *ustrtok(const char *s1, const char *s2);
extern unsigned long ustrtoul(const char * restrict nptr, const char ** restrict endptr, int base);
extern char *ustrncpy(char * restrict s1, const char * restrict s2, size_t n);
extern size_t ustrlen(const char *s);
extern void UARTPrimeTransmit(Uint16 ui16Base);

extern Uint16 testcnt;






WIZ wizfi;
IMU imu;
GPS gps;
MX mxl;
MX mxr;
DX dxl;
char sdcmd[2]={'0','0'};


/////////////////abs encoder
extern Uint16 aben_p;
extern Uint16 aben_t;
Uint16 target_p,target_t,absen=0;
bool initonce=1;
Uint16 absout=0;
char absbuf[5];
char absbuf2[5];

//////dxl

extern Uint16 cmd;
extern float intogo,intovel;
extern int16 oset[8];
extern float gearrt[8];
Uint16 checkrxd(void);


/// mx
float tragen(float so,float sf,float t,float T);
extern float ltra,rtra,ltraold,rtraold;



char *token;


void InitEPwm1Example()
{

    // Setup TBCLK
    EPwm1Regs.TBPRD = period-1;          // Set timer period 801 TBCLKs
    EPwm1Regs.TBPHS.bit.TBPHS = 0x0000;          // Phase is 0
    EPwm1Regs.TBCTR = 0x0000;                     // Clear counter

    // Set Compare values
    EPwm1Regs.CMPA.bit.CMPA = period*0.01*(100-duty);    // Set compare A value
    EPwm1Regs.CMPB.bit.CMPB = 8000;    // Set Compare B value

    // Setup counter mode
    EPwm1Regs.TBCTL.bit.CTRMODE = TB_COUNT_UPDOWN; // Count up and douwn
    EPwm1Regs.TBCTL.bit.PHSEN = TB_DISABLE;       // Disable phase loading
    EPwm1Regs.TBCTL.bit.HSPCLKDIV = 0;      // Clock ratio to SYSCLKOUT 1/1
    EPwm1Regs.TBCTL.bit.CLKDIV = 3;        // 25/8=3125000

    // Setup shadowing
    EPwm1Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW;
    EPwm1Regs.CMPCTL.bit.SHDWBMODE = CC_SHADOW;
    EPwm1Regs.CMPCTL.bit.LOADAMODE = CC_CTR_ZERO; // Load on Zero
    EPwm1Regs.CMPCTL.bit.LOADBMODE = CC_CTR_ZERO;

    // Set actions
    EPwm1Regs.AQCTLA.bit.CAU = AQ_SET;            // Set PWM1A on event A, up
                                                  // count
    EPwm1Regs.AQCTLA.bit.CAD = AQ_CLEAR;          // Clear PWM1A on event A,
                                                  // down count

    EPwm1Regs.AQCTLB.bit.CBU = AQ_SET;            // Set PWM1B on event B, up
                                                  // count
    EPwm1Regs.AQCTLB.bit.CBD = AQ_CLEAR;          // Clear PWM1B on event B,
                                                  // down count

    // Interrupt where we will change the Compare Values
    EPwm1Regs.ETSEL.bit.INTSEL = ET_CTR_ZERO;     // Select INT on Zero event
    EPwm1Regs.ETSEL.bit.INTEN = 1;                // Enable INT
    EPwm1Regs.ETPS.bit.INTPRD = ET_3RD;           // Generate INT on 3rd event



}

void InitEPwm2Example()
{

    // Setup TBCLK
    EPwm2Regs.TBPRD = period;          // Set timer period 801 TBCLKs
    EPwm2Regs.TBPHS.bit.TBPHS = 0x0000;          // Phase is 0
    EPwm2Regs.TBCTR = 0x0000;                     // Clear counter

    // Set Compare values
    EPwm2Regs.CMPA.bit.CMPA = period*0.01*(100-duty);    // Set compare A value
    EPwm2Regs.CMPB.bit.CMPB = 8000;    // Set Compare B value

    // Setup counter mode
    EPwm2Regs.TBCTL.bit.CTRMODE = TB_COUNT_UPDOWN; // Count up and douwn
    EPwm2Regs.TBCTL.bit.PHSEN = TB_DISABLE;       // Disable phase loading
    EPwm2Regs.TBCTL.bit.HSPCLKDIV = 1;      // Clock ratio to SYSCLKOUT   1/2
    EPwm2Regs.TBCTL.bit.CLKDIV = 7; //   1/128

    // Setup shadowing
    EPwm2Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW;
    EPwm2Regs.CMPCTL.bit.SHDWBMODE = CC_SHADOW;
    EPwm2Regs.CMPCTL.bit.LOADAMODE = CC_CTR_ZERO; // Load on Zero
    EPwm2Regs.CMPCTL.bit.LOADBMODE = CC_CTR_ZERO;

    // Set actions
    EPwm2Regs.AQCTLA.bit.CAU = AQ_SET;            // Set PWM1A on event A, up
                                                  // count
    EPwm2Regs.AQCTLA.bit.CAD = AQ_CLEAR;          // Clear PWM1A on event A,
                                                  // down count

    EPwm2Regs.AQCTLB.bit.CBU = AQ_SET;            // Set PWM1B on event B, up
                                                  // count
    EPwm2Regs.AQCTLB.bit.CBD = AQ_CLEAR;          // Clear PWM1B on event B,
                                                  // down count

    // Interrupt where we will change the Compare Values
    EPwm2Regs.ETSEL.bit.INTSEL = ET_CTR_ZERO;     // Select INT on Zero event
    EPwm2Regs.ETSEL.bit.INTEN = 1;                // Enable INT
    EPwm2Regs.ETPS.bit.INTPRD = ET_1ST;           // Generate INT on 3rd event
}

void InitEPwm3Example()
{

    // Setup TBCLK
    EPwm3Regs.TBPRD = period-1;          // Set timer period 801 TBCLKs
    EPwm3Regs.TBPHS.bit.TBPHS = 0x0000;          // Phase is 0
    EPwm3Regs.TBCTR = 0x0000;                     // Clear counter

    // Set Compare values
    EPwm3Regs.CMPA.bit.CMPA = period*0.01*(100-duty);    // Set compare A value
    EPwm3Regs.CMPB.bit.CMPB = 8000;    // Set Compare B value

    // Setup counter mode
    EPwm3Regs.TBCTL.bit.CTRMODE = TB_COUNT_UPDOWN; // Count up and douwn
    EPwm3Regs.TBCTL.bit.PHSEN = TB_DISABLE;       // Disable phase loading
    EPwm3Regs.TBCTL.bit.HSPCLKDIV = 0;     // 1/1 Clock ratio to SYSCLKOUT  1/1 pwmclk= 25Mhz default(why?
//    EPwm3Regs.TBCTL.bit.HSPCLKDIV = 7;     // 1/1 Clock ratio to SYSCLKOUT  1/14 fullstep
    EPwm3Regs.TBCTL.bit.CLKDIV = 3; // 1/8    f_tbclk 25/8=3125000 /14=223214  f_pwm=f_tbclk/(2*tbprd)

    // Setup shadowing
    EPwm3Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW;
    EPwm3Regs.CMPCTL.bit.SHDWBMODE = CC_SHADOW;
    EPwm3Regs.CMPCTL.bit.LOADAMODE = CC_CTR_ZERO; // Load on Zero
    EPwm3Regs.CMPCTL.bit.LOADBMODE = CC_CTR_ZERO;

    // Set actions
    EPwm3Regs.AQCTLA.bit.CAU = AQ_SET;            // Set PWM1A on event A, up
                                                  // count
    EPwm3Regs.AQCTLA.bit.CAD = AQ_CLEAR;          // Clear PWM1A on event A,
                                                  // down count

    EPwm3Regs.AQCTLB.bit.CBU = AQ_SET;            // Set PWM1B on event B, up
                                                  // count
    EPwm3Regs.AQCTLB.bit.CBD = AQ_CLEAR;          // Clear PWM1B on event B,
                                                  // down count

    // Interrupt where we will change the Compare Values
    EPwm3Regs.ETSEL.bit.INTSEL = ET_CTR_ZERO;     // Select INT on Zero event
    EPwm3Regs.ETSEL.bit.INTEN = 1;                // Enable INT
    EPwm3Regs.ETPS.bit.INTPRD = ET_3RD;           // Generate INT on 3rd event
}

void InitEPwm4Example()
{
	EPwm4Regs.ETSEL.bit.SOCAEN	= 1;	        // Disable SOC on A group
	EPwm4Regs.ETSEL.bit.SOCASEL	= 4;	        // Select SOC on up-count
	EPwm4Regs.ETPS.bit.SOCAPRD = 1;		        // Generate pulse on 1st event
    // Setup TBCLK

	EPwm4Regs.TBPRD = period-1; //10Hz
    EPwm4Regs.TBPHS.bit.TBPHS = 0x0000;          // Phase is 0
    EPwm4Regs.TBCTR = 0x0000;                     // Clear counter

    // Set Compare values
    EPwm4Regs.CMPA.bit.CMPA = period*0.5;    // Set compare A value
    EPwm4Regs.CMPB.bit.CMPB = EPWM4_MIN_CMPB;    // Set Compare B value

    // Setup counter mode
    EPwm4Regs.TBCTL.bit.CTRMODE = TB_COUNT_UPDOWN; // Count up and douwn
    EPwm4Regs.TBCTL.bit.PHSEN = TB_DISABLE;       // Disable phase loading
    EPwm4Regs.TBCTL.bit.HSPCLKDIV = TB_DIV1;      // Clock ratio to SYSCLKOUT
    EPwm4Regs.TBCTL.bit.CLKDIV = TB_DIV1;
//    EPwm4Regs.TBCTL.bit.HSPCLKDIV = 0;      // Clock ratio to SYSCLKOUT
//    EPwm4Regs.TBCTL.bit.CLKDIV = 3;        // 25/8=3125000

    // Setup shadowing
    EPwm4Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW;
    EPwm4Regs.CMPCTL.bit.SHDWBMODE = CC_SHADOW;
    EPwm4Regs.CMPCTL.bit.LOADAMODE = CC_CTR_ZERO; // Load on Zero
    EPwm4Regs.CMPCTL.bit.LOADBMODE = CC_CTR_ZERO;

    // Set actions
    EPwm4Regs.AQCTLA.bit.CAU = AQ_SET;           // Set PWM2A on event A, up
                                                 // count
    EPwm4Regs.AQCTLA.bit.CBD = AQ_CLEAR;         // Clear PWM2A on event B, down
                                                 // count

    EPwm4Regs.AQCTLB.bit.ZRO = AQ_CLEAR;         // Clear PWM2B on zero
    EPwm4Regs.AQCTLB.bit.PRD = AQ_SET  ;         // Set PWM2B on period

    // Interrupt where we will change the Compare Values
    EPwm4Regs.ETSEL.bit.INTSEL = ET_CTR_ZERO;    // Select INT on Zero event
    EPwm4Regs.ETSEL.bit.INTEN = 1;               // Enable INT
    EPwm4Regs.ETPS.bit.INTPRD = ET_3RD;          // Generate INT on 3rd event


}

void InitEPwm7Example()
{

    // Setup TBCLK
    EPwm7Regs.TBPRD = period/50;          // Set timer period 801 TBCLKs

    EPwm7Regs.TBPHS.bit.TBPHS = 0x0000;          // Phase is 0
    EPwm7Regs.TBCTR = 0x0000;                     // Clear counter

    // Set Compare values
    EPwm7Regs.CMPA.bit.CMPA = PWM_period*0.1*0.01*(100-duty);    // Set compare A value
    EPwm7Regs.CMPB.bit.CMPB = 8000;    // Set Compare B value

    // Setup counter mode
    EPwm7Regs.TBCTL.bit.CTRMODE = TB_COUNT_UPDOWN; // Count up and douwn
    EPwm7Regs.TBCTL.bit.PHSEN = TB_DISABLE;       // Disable phase loading
    EPwm7Regs.TBCTL.bit.HSPCLKDIV = 1;      // Clock ratio to SYSCLKOUT
    EPwm7Regs.TBCTL.bit.CLKDIV = 7;

    // Setup shadowing
    EPwm7Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW;
    EPwm7Regs.CMPCTL.bit.SHDWBMODE = CC_SHADOW;
    EPwm7Regs.CMPCTL.bit.LOADAMODE = CC_CTR_ZERO; // Load on Zero
    EPwm7Regs.CMPCTL.bit.LOADBMODE = CC_CTR_ZERO;

    // Set actions
    EPwm7Regs.AQCTLA.bit.CAU = AQ_SET;            // Set PWM1A on event A, up
                                                 // count
    EPwm7Regs.AQCTLA.bit.CAD = AQ_CLEAR;          // Clear PWM1A on event A,
                                                  // down count

    EPwm7Regs.AQCTLB.bit.CBU = AQ_SET;            // Set PWM1B on event B, up
                                                  // count
    EPwm7Regs.AQCTLB.bit.CBD = AQ_CLEAR;          // Clear PWM1B on event B,
                                                  // down count

    // Interrupt where we will change the Compare Values
    EPwm7Regs.ETSEL.bit.INTSEL = ET_CTR_ZERO;     // Select INT on Zero event
    EPwm7Regs.ETSEL.bit.INTEN = 1;                // Enable INT
    EPwm7Regs.ETPS.bit.INTPRD = ET_3RD;           // Generate INT on 3rd event


}



void InitEPwmvariable(void)
{
	   duty = 50; // percent
	   	duty16= 5000;
	   	period=PWM_period; // when updown pwmclk=200Mhz/4/period
	   	ena=0;
	   	enar=0;
	   	enal=0;
	   	dir1=0;
	   	dir2=0;
	   	//desv=0;
	   	cmd2=0;
	   	dxl.cmd=100;
	   	gps.Lay[0]='9';
	   	gps.Long[0]='9';
	    imu.roll[0]='0';
	    imu.pitch[0]='0';
	    imu.yaw[0]='0';
#ifdef _FLASH
#ifndef __cplusplus
#pragma CODE_SECTION(checkrxd, "ramfuncs");
#pragma CODE_SECTION(epwm7_isr, "ramfuncs");
#endif
	    memcpy(&RamfuncsRunStart, &RamfuncsLoadStart, (size_t)&RamfuncsLoadSize);
#endif
}

__interrupt void epwm1_isr(void)
{
	float velinl;


	//input_old=input;
	//duty = 50 + input*10;
	if (mt==true)
	{
		//desv=traj[2];
	}
	//duty=50;
	duty16=duty*100;
	if(ltra<0)
	{
		dir1=1;
		velinl=-ltra;
	}
	else
	{
		dir1=0;
		velinl=ltra;
	}

#ifdef _FLASH
	EPwm1Regs.TBPRD = 1484375/16/200.0/velinl; //desv(rps) 16step
//	EPwm1Regs.TBPRD = 106027/200.0/velinl; //desv(rps) fullstep
#else
	EPwm1Regs.TBPRD = 1562500/16/200.0/velinl; //desv(rps) 16step
//	EPwm1Regs.TBPRD = 111607/200.0/velinl; //desv(rps) fullstep
#endif
	EPwm1Regs.CMPA.bit.CMPA = EPwm1Regs.TBPRD*0.0001*(10000-duty16);
    // Set compare A value
    // Clear INT flag for this timer
    EPwm1Regs.ETCLR.bit.INT = 1;

    // Acknowledge this interrupt to receive more interrupts from group 3
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP3;

    if((desvl*desvl)<0.0001||desvr*desvr<0.0001)
    {
    	ena=0;
    }

    if (enal&&enar)
    	ena=1;
    else
    	ena=0;

    if (rst){

    	GPIO_WritePin(9, 0);
    	DELAY_US(1);
    	GPIO_WritePin(9, 1);
    	rst=0;


    }

    if (ena){
    	GPIO_WritePin(10, 0);
    	GPIO_WritePin(8, 1);
    	GPIO_WritePin(9, 1);



    }
    else
    		{
    	GPIO_WritePin(10, 1);
    	GPIO_WritePin(8, 0);
    	GPIO_WritePin(9, 0);
    	ltraold=0;
    	rtraold=0;
    }



    if (dir1 == 0){
    	GPIO_WritePin(2, 1);
    }
    else {
    	GPIO_WritePin(2, 0);
    }





}

__interrupt void epwm2_isr(void)
{

	int cmdnum,cmdnum2;
	Uint16 tempnum;


	// Update the CMPA and CMPB values
	if(gps.cmd[0]=='1')
	{
	cmdnum=usprintf(btxbuf,"GPS=%s,%s,%s\r\n",gps.time,gps.Lay,gps.Long);
	scib_msg(btxbuf,cmdnum);
	}

	if(cmd==7||cmd==71||cmd==72||cmd==73||cmd==1||cmd==5)
	{
	tempnum=usprintf(dtxbuf,"DX+POS=%d\r\nDX+TOR=%d\r\n",(long)dxl.pos,(long)(dxl.tor*100));
	scib_msg(dtxbuf,tempnum);
	}

	if(imu.cmd[0]=='1')
	{
	cmdnum=usprintf(btxbuf,"IMU=%s,%s,%s,%s,%s,%s\r\n",imu.roll,imu.pitch,imu.yaw,imu.sx,imu.sy,imu.sz);
	scib_msg(btxbuf,cmdnum);
	}

	UARTPrimeTransmit(1);


	testcnt++;


								    // Clear INT flag for this timer
								    EPwm2Regs.ETCLR.bit.INT = 1;

    // Acknowledge this interrupt to receive more interrupts from group 3
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP3;
}

__interrupt void epwm3_isr(void)
{
	float velinr;



	duty16=duty*100;
	if(rtra<0)
	{
		dir2=1;
		velinr=-rtra;
	}
	else
	{
		dir2=0;
		velinr=rtra;
	}
#ifdef _FLASH
	EPwm3Regs.TBPRD = 1484375/16/200.0/velinr; //desv(rpm)
//	EPwm3Regs.TBPRD = 106027/200.0/velinr; //desv(rps) fullstep
#else
	EPwm3Regs.TBPRD = 1562500/16/200.0/velinr; //desv(rpm)
//	EPwm3Regs.TBPRD = 111607/200.0/velinr; //desv(rps) fullstep
#endif
//	EPwm3Regs.TBPRD = 500; //desv(rpm)
	EPwm3Regs.CMPA.bit.CMPA = EPwm3Regs.TBPRD*0.0001*(10000-duty16);
    // Set compare A value
    // Clear INT flag for this timer
    EPwm3Regs.ETCLR.bit.INT = 1;

    // Acknowledge this interrupt to receive more interrupts from group 3
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP3;


//    if (rst){
//
//    	GPIO_WritePin(9, 0);
//    	DELAY_US(1);
//    	GPIO_WritePin(9, 1);
//    	rst=0;
//
//
//    }
//
//    if (ena){
//    	GPIO_WritePin(10, 0);
//    	GPIO_WritePin(8, 1);
//    	GPIO_WritePin(9, 1);
//
//    }
//    else
//    		{
//    	GPIO_WritePin(10, 1);
//    	GPIO_WritePin(8, 0);
//    	GPIO_WritePin(9, 0);
//    }


    if (dir2 == 0){
    	GPIO_WritePin(6, 0);
    }
    else {
    	GPIO_WritePin(6, 1);
    }



}

__interrupt void epwm4_isr(void)
{


    // Clear INT flag for this timer
    EPwm4Regs.ETCLR.bit.INT = 1;

    // Acknowledge this interrupt to receive more interrupts from group 3
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP3;
}

__interrupt void epwm7_isr(void)
{
	char *strbuf;
	char *strbuf2;
	char *strbuf3;

	char charbuf,charbuf2;

	int cmdnum,cmdnum2;
	Uint16 templ,templ2,templ3,templ4,templ5;
	Uint16 drxsum;
	Uint8 cksum;
	int i;


	Uint16 dxcmdin;



	pwmtimer1++;

	if(SciaRegs.SCIRXST.bit.RXERROR)
	{
		SciaRegs.SCICTL1.bit.SWRESET=0;
	    __asm(" nop");
	    __asm(" nop");
	    __asm(" nop");
	    __asm(" nop");
	    SciaRegs.SCICTL1.bit.SWRESET=1;
	}

// buffer a clear
	if(IsBufferFull(&a_ui16UARTRxReadIndex,  \
            &a_ui16UARTRxWriteIndex, \
            256))
	{
		a_ui16UARTRxWriteIndex = 0;
		a_ui16UARTRxReadIndex = 0;
	}

	if(IsBufferFull(&b_ui16UARTTxReadIndex,  \
            &b_ui16UARTTxWriteIndex, \
            256))
	{

		b_ui16UARTTxWriteIndex = 0;
		b_ui16UARTTxReadIndex = 0;

	}

	if(IsBufferFull(&c_ui16UARTRxReadIndex,  \
            &c_ui16UARTRxWriteIndex, \
            256))
	{

		c_ui16UARTRxWriteIndex = 0;
		c_ui16UARTRxReadIndex = 0;

	}




	if(!IsBufferEmpty(&a_ui16UARTRxReadIndex,   \
            &a_ui16UARTRxWriteIndex))
	{
		//get single char from a to b
//		charbuf=UARTgetc(0);
//		if(charbuf)
//		{
////			scib_msg("AT+SSEND=0,,,1\r",15);
//			scib_msg(&charbuf,1);
//		}



			//numout=UARTgets(tempa,100,0);

			templ=UARTPeek('\r',0);
			if(templ)
			{
			numout=UARTgets(tempa,templ+1,0);
			}
			else
				numout=0;
			if(numout)
			{

					if(strbuf3=ustrstr(tempa,"GGA"))
					{

//						ustrncpy(gps.time,strbuf3+4,6);
//						ustrncpy(gps.Lay,strbuf3+14,10);
//						ustrncpy(gps.Long,strbuf3+27,11);
						token=ustrtok(strbuf3,",");
						templ2=ustrlen(token);
						ustrncpy(strbuf3,strbuf3+templ2+1,ustrlen(strbuf3+templ2));
						token=ustrtok(strbuf3,",");
						templ2=ustrlen(token);
						memcpy(&gps.time,token,templ2);
						gps.time[templ2+1]=0;
						ustrncpy(strbuf3,strbuf3+templ2+1,ustrlen(strbuf3+templ2));
						token=ustrtok(strbuf3,",");
						templ2=ustrlen(token);
						memcpy(&gps.Lay,token,templ2);
						gps.Lay[templ2+1]=0;
						ustrncpy(strbuf3,strbuf3+templ2+1,ustrlen(strbuf3+templ2));
						token=ustrtok(strbuf3,",");
						templ2=ustrlen(token);
						ustrncpy(strbuf3,strbuf3+templ2+1,ustrlen(strbuf3+templ2));
						token=ustrtok(strbuf3,",");
						templ2=ustrlen(token);
						memcpy(&gps.Long,token,templ2);
						gps.Long[templ2+1]=0;
					}


			}








	}

	if(!IsBufferEmpty(&b_ui16UARTRxReadIndex,   \
            &b_ui16UARTRxWriteIndex))
	{
//		//get from b
//		charbuf2=UARTgetc(1);
//		if(charbuf2)
//		{
//		scia_msg(&charbuf2,1);
//		}

		//get from b
		templ=UARTPeek('\r',1);
		if(templ)
		{
		numout=UARTgets(printbuf,templ+1,1);
		}
		else
			numout=0;
		//
		if(numout)
		{
		//scia_msg(printbuf,numout);  //printbuf print in scia
		if(strbuf=ustrstr(printbuf,"cmd"))
		{
			strcpy(wizfi.cmd,strbuf+4);
			if(strbuf2=ustrstr(wizfi.cmd,"imu"))
					{
						ustrncpy(imu.cmd,strbuf2+4,1);
					}
			if(strbuf2=ustrstr(wizfi.cmd,"gps"))
					{
						ustrncpy(gps.cmd,strbuf2+4,1);
					}
			if(strbuf2=ustrstr(wizfi.cmd,"sd"))
								{
									ustrncpy(sdcmd,strbuf2+3,1);
								}
			if(ustrstr(wizfi.cmd,"MXL"))
					{
						mxbuf=ustrstr(wizfi.cmd,"MXL");
						ustrncpy(mxl.ENA,mxbuf+4,1);
						ustrncpy(mxl.DIR,mxbuf+6,1);
						ustrncpy(mxl.DESVBUF,mxbuf+8,3);
						mxl.desv=(float)ustrtoul(mxl.DESVBUF,NULL,10)/60.0;

						if (mxl.DIR[0]=='1')
//							dir1=1;
							desvl=-mxl.desv;
						else
//							dir1=0;
							desvl=mxl.desv;
						if (mxl.ENA[0]=='1'&&desvl)
							enal=1;
						else
							enal=0;
					}
			if(ustrstr(wizfi.cmd,"MXR"))
					{
				mxr.desv=0.0;
				mxrbuf=ustrstr(wizfi.cmd,"MXR");
				ustrncpy(mxr.ENA,mxrbuf+4,1);
				ustrncpy(mxr.DIR,mxrbuf+6,1);
				ustrncpy(mxr.DESVBUF,mxrbuf+8,3);
				mxr.desv = (float)ustrtoul(mxr.DESVBUF,NULL,10)/60.0;
//				mxl.desv=(float)ustrtoul(mxl.DESVBUF,NULL,0)/10.0;

				if (mxr.DIR[0]=='1')
//					dir2=1;
					desvr=-mxr.desv;
				else
//					dir2=0;
					desvr=mxr.desv;
				if (mxr.ENA[0]=='1'&&desvr)
					enar=1;
				else
					enar=0;

				fflush(mxrbuf);
					}

			if(ustrstr(wizfi.cmd,"DX"))
					{
//				dtxbuf=ustrstr(wizfi.cmd,"DX");
				ustrncpy(dtxbuf,ustrstr(wizfi.cmd,"DX"),13);
				ustrncpy(dxl.cmdin,dtxbuf+3,3);
				dxcmdin=(Uint16)ustrtoul(dxl.cmdin,NULL, 10);
				if(dxcmdin==7)
				{
					ustrncpy(dxl.posin,dtxbuf+7,4);
					if(dxl.posin[0]=='-')
					{
						dxl.posin[0]='0';
						intogo=(float)ustrtoul(dxl.posin,NULL, 10);
						intogo=-intogo;
					}
					else
						intogo=(float)ustrtoul(dxl.posin,NULL, 10);

					intovel=(intogo-dxl.pos)*20;
					if(intovel<0)
						intovel=-intovel;


					if(intogo<500&&intogo>-500)
						cmd=7;
					else
						cmd=100;


				}
				if(dxcmdin==12)
				{

					cmd=12;
//					cmdnum=usprintf(dtxbuf,"DX=%f,%f\r",dxl.pos,dxl.tor);
//					scib_msg(dtxbuf,cmdnum);

				}
				if(dxcmdin==13)
				{

					cmd=13;
//					cmdnum=usprintf(dtxbuf,"DX=%f,%f\r",dxl.pos,dxl.tor);
//					scib_msg(dtxbuf,cmdnum);

				}
				fflush(dtxbuf);


					}

			if(ustrstr(wizfi.cmd,"STOP"))
					{
						cmd=12;
						ena=0;
					}


// for absolute encoder
//			if(ustrstr(wizfi.cmd,"PAN"))
//								{
//									mxbuf=ustrstr(wizfi.cmd,"PAN");
//									ustrncpy(absbuf,mxbuf+4,5);
//									target_p=ustrtoul(absbuf,NULL,0);
//								}
//			if(ustrstr(wizfi.cmd,"TIL"))
//										{
//											mxbuf=ustrstr(wizfi.cmd,"TIL");
//											ustrncpy(absbuf,mxbuf+4,5);
//											target_t=ustrtoul(absbuf,NULL,0);
//										}
//			if(ustrstr(wizfi.cmd,"ABS"))
//										{
//											mxbuf=ustrstr(wizfi.cmd,"ABS");
//											ustrncpy(absbuf,mxbuf+4,3);
//											if(absbuf[0]=='1')
//											{
//												absout=1;
//
//
//											}
//
//											else
//											{
//												absout=0;
//
//											}
//											if(absbuf[2]=='1')
//											{
//												absen=1;
//										   		target_p=aben_p;
//										   		target_t=aben_t;
//											}
//											else
//											{
//												absen=0;
//											}
//
//										}

			cmdnum=usprintf(cmdbuf2,"Arrived=%s\r\n",strbuf);
			scib_msg(cmdbuf2,cmdnum);



			fflush(wizfi.cmd);
		}

		}

	}

	if(!IsBufferEmpty(&c_ui16UARTRxReadIndex,   \
	            &c_ui16UARTRxWriteIndex))
		{



				//numout=UARTgets(tempa,100,0);

				templ=UARTPeek('\r',2);
				if(templ)
				{
				numout=UARTgets(tempa,templ+1,2);
				}
				else
					numout=0;
				if(numout)
				{

						if(tempa[0]=='*')
						{
//							memset(imu.roll,0,sizeof(imu.roll));
//							memset(imu.pitch,0,sizeof(imu.pitch));
//							memset(imu.yaw,0,sizeof(imu.yaw));
//							templ2=ustrstr(tempa,",")-ustrstr(tempa,"*");
//							ustrncpy(imu.roll,tempa+1,templ2-1);
//							templ3=ustrstr(tempa+templ2+1,",")-ustrstr(tempa,",");
//							ustrncpy(imu.pitch,tempa+templ2+1,templ3-1);
//							templ4=ustrstr(tempa+templ2+templ3+1,",")-ustrstr(tempa+templ2+1,",");
//							ustrncpy(imu.yaw,tempa+templ2+templ3+1,templ4-1);

							ustrncpy(tempa,tempa+1,ustrlen(tempa));
							token=ustrtok(tempa,",");
							templ2=ustrlen(token);
							memcpy(&imu.roll,token,templ2);
							imu.roll[templ2+1]=0;
							ustrncpy(tempa,tempa+templ2+1,ustrlen(tempa+templ2));
							token=ustrtok(tempa,",");
							templ2=ustrlen(token);
							memcpy(&imu.pitch,token,templ2);
							imu.pitch[templ2+1]=0;
							ustrncpy(tempa,tempa+templ2+1,ustrlen(tempa+templ2));
							token=ustrtok(tempa,",");
							templ2=ustrlen(token);
							memcpy(&imu.yaw,token,templ2);
							imu.yaw[templ2+1]=0;
							ustrncpy(tempa,tempa+templ2+1,ustrlen(tempa+templ2));
							token=ustrtok(tempa,",");
							templ2=ustrlen(token);
							memcpy(&imu.vx,token,templ2);
							imu.vx[templ2+1]=0;
							ustrncpy(tempa,tempa+templ2+1,ustrlen(tempa+templ2));
							token=ustrtok(tempa,",");
							templ2=ustrlen(token);
							memcpy(&imu.vy,token,templ2);
							imu.vy[templ2+1]=0;
							ustrncpy(tempa,tempa+templ2+1,ustrlen(tempa+templ2));
							token=ustrtok(tempa,",");
							templ2=ustrlen(token);
							memcpy(&imu.vz,token,templ2);
							imu.vz[templ2+1]=0;
							ustrncpy(tempa,tempa+templ2+1,ustrlen(tempa+templ2));
							token=ustrtok(tempa,",");
							templ2=ustrlen(token);
							memcpy(&imu.sx,token,templ2);
							imu.sx[templ2+1]=0;
							ustrncpy(tempa,tempa+templ2+1,ustrlen(tempa+templ2));
							token=ustrtok(tempa,",");
							templ2=ustrlen(token);
							memcpy(&imu.sy,token,templ2);
							imu.sy[templ2+1]=0;
							ustrncpy(tempa,tempa+templ2+1,ustrlen(tempa+templ2));
							memcpy(&imu.sz,tempa,ustrlen(tempa));
							imu.sz[templ2+1]=0;


						}

				}








		}




//	//// for absolute encoder
//
//	if(initonce)
//	{
//   		target_p=aben_p;
//   		target_t=aben_t;
//   		initonce=0;
//	}
//
//	if(absen&(target_p-aben_p)*(target_p-aben_p)+(target_t-aben_t)*(target_t-aben_t)>200&(target_p-aben_p)*(target_p-aben_p)+(target_t-aben_t)*(target_t-aben_t)<10e8)
//	{
//	ena=1;
//	desvl=((float)target_p-aben_p)*0.005;
//
//	if( target_p>=aben_p){
//    	dir1=0;
//    }
//    else {
//    	desvl=-desvl;
//    	dir1=1;
//    }
//
//	if(desvl>8)
//	{
//		desvl=8;
//	}
//
//
//	desvr=((float)target_t-aben_t)*0.005;
//	if( target_t>=aben_t){
//    	dir2=0;
//    }
//    else {
//    	desvr=-desvr;
//    	dir2=1;
//    }
//	if(desvr>10)
//	{
//		desvr=10;
//	}
//
//
//
//
//
//
//	}
//
//	else
//	{
//		ena=0;
//	}
//
//
//
//	if(absout)
//	{
//		cmdnum=usprintf(cmdbuf2,"DAT+PAN=%u,TIL=%u\r",(unsigned long)aben_p,(unsigned long)aben_t);
//		scib_msg(cmdbuf2,cmdnum);
//	}
//
//	//// for abs ///




//	checkrxd();











    // Clear INT flag for this timer
    EPwm7Regs.ETCLR.bit.INT = 1;

    // Acknowledge this interrupt to receive more interrupts from group 3
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP3;
}

Uint16 checkrxd(void)
{
	Uint16 templ;
	Uint16 drxsum;
	Uint8 cksum;
	Uint16 i=0;
	Uint16 tempnum;





	if(!IsBufferEmpty(&d_ui16UARTRxReadIndex,   \
	            &d_ui16UARTRxWriteIndex))
		{
		templ=UARTPeek(0xFF,3);
		if(templ)
		{
		numout=UARTgets(tempa,5,3);
		}
		else
			numout=0;
		if(numout)
		{
			if(tempa[2]==2)
			{
				templ=tempa[3];
				dxl.id=2;
				dxl.error=tempa[4];
				drxsum=tempa[2]+tempa[3]+tempa[4];
				numout=UARTgets(dtxbuf,templ-1,3);

				while(i<(templ-2))
				{
					drxsum=drxsum+dtxbuf[i];
					i++;
				}
				dxl.cksum=((~(drxsum&0xFF))&0xFF);
				if(dxl.cksum!=dtxbuf[templ-2])
				{
					dxl.status=1;
					cksum=dtxbuf[templ-2];
				}
				else
				{
					dxl.status=0;
					if(dxl.cmd==1)
					{
						dxl.ipos=(((dtxbuf[1]<<8)+dtxbuf[0]));
						dxl.pos=(dxl.ipos-oset[1])/(gearrt[1]*4095/360);


					}
					else if(dxl.cmd==5)
					{

						dxl.itor=(((Uint16)(dtxbuf[1]<<8)+dtxbuf[0]))&0x3FF;
						if(dtxbuf[1]&0x4)
							{
							dxl.itor=-dxl.itor;
							}

						dxl.tor=dxl.itor/1024.0;

					}
					else if(dxl.cmd==12)
										{

										}
				}

				dxl.cmd=100;
				fflush(tempa);
				fflush(dtxbuf);
				return 1;

			}


		}


		}

	return 0;
}

float tragen(float so,float sf,float t,float T)
{
	float A,B,C,D,E,F,y;
	A=(6*sf)/(T*T*T*T*T) - (6*so)/(T*T*T*T*T);
	B=(15*so)/(T*T*T*T) - (15*sf)/(T*T*T*T);
	C=(10*sf)/(T*T*T) - (10*so)/(T*T*T);
	D=0;
	E=0;
	F=so;
	y=A*(t*t*t*t*t)+ B*(t*t*t*t) + C*(t*t*t) + D*(t*t) + E*t + F;
	return y;
}
