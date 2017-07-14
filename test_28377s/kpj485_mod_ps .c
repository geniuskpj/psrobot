//###########################################################################


#include "F28x_Project.h"     // Device Headerfile and Examples Include File
#include "F2837xD_GlobalPrototypes.h"
#include <stdlib.h>
#include <string.h>
#include "serial.h"





#include "interrupt.h"
#include "cmdline.h"
#include "ustdlib.h"
#include "ff.h"
#include "diskio.h"
   FRESULT fresult;
   FIL fil;
   DWORD filsize;
static FATFS g_sFatFs;
WORD numoutsd=0;
int spfout =0;
char bufsd[64];
char rdsd[64];
Uint16 writeflag=0;
Uint16 sdinitflag=1;
Uint16 sdnum;
extern char sdcmd[2];
extern IMU imu;
extern GPS gps;



//#include <math.h>
//#include <stdio.h>









typedef struct
{
    volatile struct EPWM_REGS *EPwmRegHandle;
    Uint16 EPwm_CMPA_Direction;
    Uint16 EPwm_CMPB_Direction;
    Uint16 EPwmTimerIntCount;
    Uint16 EPwmMaxCMPA;
    Uint16 EPwmMinCMPA;
    Uint16 EPwmMaxCMPB;
    Uint16 EPwmMinCMPB;
}EPWM_INFO;

typedef unsigned char   Uint8;





//buffer for storing conversion results
#define RESULTS_BUFFER_SIZE 25
Uint16 AdcaResults[RESULTS_BUFFER_SIZE];
Uint16 resultsIndex;
Uint16 bufferFull;
Uint16 testcnt=0;;
float degpel=0;
float degknee=0;
// Prototype statements for functions found within this file.


extern interrupt void sciaTxFifoIsr(void);
extern interrupt void sciaRxFifoIsr(void);
extern interrupt void scibTxFifoIsr(void);
extern interrupt void scibRxFifoIsr(void);
extern interrupt void scicTxFifoIsr(void);
extern interrupt void scicRxFifoIsr(void);
extern interrupt void scidTxFifoIsr(void);
extern interrupt void scidRxFifoIsr(void);
__interrupt void cpu_timer0_isr(void);
__interrupt void cpu_timer1_isr(void);


extern void error(void);
extern void senddx(Uint8 id, Uint8 length,Uint8 param[]);
extern void readdx(Uint8 id,Uint8 param1,Uint8 param2);
extern void ping(Uint8 id);
extern void reset(Uint8 id);

//extern void scia_xmit(Uint8 a);
//extern Uint16 scia_msg(const char *a,Uint16 leng);
//extern bool scia_rcv();
//extern void scic_xmit(Uint8 a);
//extern void scic_msg(Uint8* a,Uint8 leng);
//extern bool scic_rcv();

extern void led(Uint8 id, bool ono);
extern void mvf(Uint8 id,float goalpositon,float vel);
extern void mv_p(Uint8 id,float goalpositon);
extern void mv_s(Uint8 id,float vel);
extern void setoffset(Uint8 id,int16 offset);
extern void setcwlimit(Uint8 id,int16 limit);
extern void setccwlimit(Uint8 id,int16 limit);
//void bulkreaddx(Uint8 modnum,Uint8 startadress);
extern void syncmv(Uint8 modnum,float goal[4]);
//void retfowr(Uint8 id,bool ono);
extern void motorstop();
extern float gearrt[8];


//float trajcal(float s0,float sf,float T,float t);

void ConfigureGPIO(void);
void ConfigureADC(void);
interrupt void adca1_isr(void);

extern void InitEPwm1Example(void);
extern void InitEPwm2Example(void);
extern void InitEPwm3Example(void);
extern void InitEPwm4Example(void);
extern void InitEPwm7Example(void);
extern void InitEPwmvariable(void);
extern __interrupt void epwm1_isr(void);
extern __interrupt void epwm2_isr(void);
extern __interrupt void epwm3_isr(void);
extern __interrupt void epwm4_isr(void);
extern __interrupt void epwm7_isr(void);


void SetupADCEpwm(Uint16 channel);
void testprint(void);
void posinit(void);
Uint16 ATpnt();
static char *i2a(unsigned i, char *a, unsigned r);
char *itoa(int i, char *a, int r);


// Global variables
EPWM_INFO epwm1_info;
//EPWM_INFO epwm2_info;
EPWM_INFO epwm3_info;
EPWM_INFO epwm4_info;

extern Uint16 sdataA[30],rdataA[16],wfrdata,wfrcvnum,wfing,rdata,rdata_pointA,ena;    // Send data for SCI-A

Uint16 j,k=0;
Uint16 cmd=100;
Uint16 uid=2;
Uint16 toen=0;
Uint8 err=0;
bool find=false;
extern Uint16 releng;
Uint16 ledonoff=0;
bool rcvflag=false;
bool rcvstart=false;
bool rcvsuc=false;
bool test=false;
Uint8 rcvnum=0;
Uint8 atpntflag=0;
//extern char* ATrxmsg;
extern Uint16 ATcnt2;
Uint16 autoloop=0;
char itoabuf[5];
char itoabuf2[5];
Uint16 ATmode;
extern Uint16 lengscib;
//char *ATmsg;
//char *ATmsg2;
char *temp;
char *temp2;
extern char *printbuf;
extern char *tempa;
extern char *cmdbuf;
extern char *cmdbuf2;
extern char *btxbuf;
extern char *mxbuf;
extern char *token;
char *dtxbuf;
unsigned long prevtime,curtime,excutetime;
extern Uint16 a_ui16UARTRxWriteIndex,a_ui16UARTRxReadIndex,b_ui16UARTRxWriteIndex,b_ui16UARTRxReadIndex,d_ui16UARTRxWriteIndex,d_ui16UARTRxReadIndex;



extern Uint16 period,enar,enal,duty16,dir1,dir2,cmd2;

bool mt;
extern float duty;
extern const float desv;

int sleng=0;
float degtogo=0;
float intogo=0;
float intovel=100;
float pi=3.141592;
float traj[4];
int16 posint[8]={4096,4096,4096,4096,4096,4096,4096,4096};
float pos[8];
float posavg[8]={0,0,0,0,0,0,0,0};
float t=0;
float endtime=2;
Uint8 rcvid;
Uint16 p[5];
Uint16 p2[8];
//int16 oset[8]={0,104+2047,0,3732,0,2595-2047,0,-31};
int16 oset[8]={0,2823,0,0,0,0,0,0};
int16 oset2[8]={0,0,0,0,0,0,0,0};
int16 dir[8]={1,1,1,1,1,-1,1,-1};

float initpos[4]={0,0,0,0};
float goalpos[4]={0,0,0,0};
float goalpos2[4]={0,0,0,0};

Uint16 obs[9];
bool done=false;
//dxl motor(0x01);

//for absencoder
Uint16 aben_p=0;
Uint16 aben_t=0;


// dxl

extern DX dxl;
extern Uint16 checkrxd(void);


//mx
extern float tragen(float so,float sf,float t,float T);
float ttra,ttrar,ltra,rtra,ltraold,rtraold;
extern float desvl,desvr;
extern Uint16 ena;

void main(void)
	{
   Uint16 i;
   int nStatus;

// Step 1. Initialize System Control:
// PLL, WatchDog, enable Peripheral Clocks
// This example function is found in the F2837xD_SysCtrl.c file.
   InitSysCtrl();




   #ifdef _FLASH
#ifndef __cplusplus
#pragma CODE_SECTION(cpu_timer1_isr, "ramfuncs");
#pragma CODE_SECTION(checkrxd, "ramfuncs");
#pragma CODE_SECTION(epwm7_isr, "ramfuncs");
#endif

   memcpy(&RamfuncsRunStart, &RamfuncsLoadStart, (size_t)&RamfuncsLoadSize);

   #endif




// Step 2. Initialize GPIO:
// This example function is found in the F2837xD_Gpio.c file and
// illustrates how to set the GPIO to it's default state.
   InitGpio();

// For this example, only init the pins for the SCI-C port.
// These functions are found in the 7xD_Gpio.c file.

   ConfigureGPIO();
   CpuSysRegs.PCLKCR2.bit.EPWM1=1;
   CpuSysRegs.PCLKCR2.bit.EPWM2=1;
   CpuSysRegs.PCLKCR2.bit.EPWM3=1;
   CpuSysRegs.PCLKCR2.bit.EPWM4=1;
   CpuSysRegs.PCLKCR2.bit.EPWM7=1;
   CpuSysRegs.PCLKCR7.bit.SCI_A=1;
   CpuSysRegs.PCLKCR7.bit.SCI_B=1;
   CpuSysRegs.PCLKCR7.bit.SCI_C=1;
   CpuSysRegs.PCLKCR7.bit.SCI_D=1;

   InitEPwm1Gpio();
   InitEPwm3Gpio();
//   InitEPwm4Gpio();
// Step 3. Clear all interrupts and initialize PIE vector table:
// Disable CPU interrupts
   DINT;

// Initialize PIE control registers to their default state.
// The default state is all PIE interrupts disabled and flags
// are cleared.
// This function is found in the F2837xD_PieCtrl.c file.
   InitPieCtrl();

// Disable CPU interrupts and clear all CPU interrupt flags:
   IER = 0x0000;
   IFR = 0x0000;

// Initialize the PIE vector table with pointers to the shell Interrupt
// Service Routines (ISR).
// This will populate the entire table, even if the interrupt
// is not used in this example.  This is useful for debug purposes.
// The shell ISR routines are found in F2837xD_DefaultIsr.c.
// This function is found in F2837xD_PieVect.c.
   InitPieVectTable();

// Interrupts that are used in this example are re-mapped to
// ISR functions found within this file.
   EALLOW;  // This is needed to write to EALLOW protected registers

//   ClkCfgRegs.LOSPCP.bit.LSPCLKDIV = 1;  // change lspclk to 200/2=100 (default 2, 200/4=50 )


   PieVectTable.SCIA_RX_INT = &sciaRxFifoIsr;
   PieVectTable.SCIA_TX_INT = &sciaTxFifoIsr;
   PieVectTable.SCIB_RX_INT = &scibRxFifoIsr;
   PieVectTable.SCIB_TX_INT = &scibTxFifoIsr;
   PieVectTable.SCIC_RX_INT = &scicRxFifoIsr;
   PieVectTable.SCIC_TX_INT = &scicTxFifoIsr;
   PieVectTable.SCID_RX_INT = &scidRxFifoIsr;
   PieVectTable.SCID_TX_INT = &scidTxFifoIsr;

   PieVectTable.TIMER0_INT = &cpu_timer0_isr;
   PieVectTable.TIMER1_INT = &cpu_timer1_isr;

   PieVectTable.EPWM1_INT = &epwm1_isr;
   PieVectTable.EPWM2_INT = &epwm2_isr;
   PieVectTable.EPWM3_INT = &epwm3_isr;
   PieVectTable.EPWM4_INT = &epwm4_isr;
   PieVectTable.EPWM7_INT = &epwm7_isr;
   PieVectTable.ADCA1_INT = &adca1_isr; //function for ADCA interrupt 1
   EDIS;   // This is needed to disable write to EALLOW protected registers

// Step 4. Initialize the Device Peripherals:

   scia_fifo_init();  // Init SCI-a
   scib_fifo_init();  // Init SCI-b
   scic_fifo_init();  // Init SCI-C
   scid_fifo_init();  // Init SCI-d
   InitCpuTimers();

   ConfigureADC();
   SetupADCEpwm(0);
   InitEPwmvariable();
   InitEPwm1Example();
   InitEPwm2Example();
   InitEPwm3Example();
   InitEPwm4Example();
   InitEPwm7Example();

//
//	Uint8 p[]={0x04,0x01};
//	senddx(0x02,0x04,p);// 속도설정 1mbps
//	DELAY_US(10*1000);
//
//	Uint8 p2[]={0x04,0x01};
//	senddx(0x01,0x04,p2);// 속도설정 1mbps
//	DELAY_US(10*1000);

// Step 5. User specific code, enable interrupts:

   for(resultsIndex = 0; resultsIndex < RESULTS_BUFFER_SIZE; resultsIndex++)
  	    {
  	    	AdcaResults[resultsIndex] = 0;
  	    }
   resultsIndex = 0;
   bufferFull = 0;
//   ATmsg= (char *)malloc(sizeof(char)*20);
//   ATmsg2= (char *)malloc(sizeof(char)*15);
   temp= (char *)malloc(sizeof(char)*10);
   temp2= (char *)malloc(sizeof(char)*10);
   printbuf= (char *)malloc(sizeof(char)*62);
   tempa= (char *)malloc(sizeof(char)*150);
   cmdbuf= (char *)malloc(sizeof(char)*32);
   cmdbuf2= (char *)malloc(sizeof(char)*50);
   btxbuf= (char *)malloc(sizeof(char)*50);
   dtxbuf=(char *)malloc(sizeof(char)*50);
   mxbuf=(char *)malloc(sizeof(char)*10);
   token=(char *)malloc(sizeof(char)*10);

   ttra=0;
   ttrar=0;
   ltra=0;
   rtra=0;
   ltraold=0;
   rtraold=0;







   ConfigCpuTimer(&CpuTimer0, 200, 50000);
   ConfigCpuTimer(&CpuTimer1, 200, 10000);

   #ifdef _FLASH
   ConfigCpuTimer(&CpuTimer0, 190, 50000);
   ConfigCpuTimer(&CpuTimer1, 190, 10000);
   #endif

   CpuTimer0Regs.TCR.all = 0x4000; // Use write-only instruction to set TSS bit = 0
   CpuTimer1Regs.TCR.all = 0x4000; // Use write-only instruction to set TSS bit = 0


// Init send data.  After each transmission this data
// will be updated for the next transmission
   for(i = 0; i<30; i++)
   {
      sdataA[i] = i;
   }

   wfrcvnum=0;
   wfing=0;
   rdata_pointA = sdataA[0];












// Enable interrupts required for this example
   PieCtrlRegs.PIECTRL.bit.ENPIE = 1;   // Enable the PIE block
   PieCtrlRegs.PIEIER9.bit.INTx1=1;     // PIE Group 9, INT1
   PieCtrlRegs.PIEIER9.bit.INTx2=1;     // PIE Group 9, IN2 sciatx

   PieCtrlRegs.PIEIER9.bit.INTx3=1;     // PIE Group 9, INT3
   PieCtrlRegs.PIEIER9.bit.INTx4=1;     // PIE Group 9, IN4 scibtx

   PieCtrlRegs.PIEIER8.bit.INTx5=1;     // PIE Group 8, INT5
   PieCtrlRegs.PIEIER8.bit.INTx6=1;     // PIE Group 8, IN6 scictx
   PieCtrlRegs.PIEIER8.bit.INTx7=1;     // PIE Group 8, INT5 scidrx
   PieCtrlRegs.PIEIER8.bit.INTx8=1;     // PIE Group 8, INT5 scidtx




   PieCtrlRegs.PIEIER1.bit.INTx7 = 1;   // for cputimer0
   PieCtrlRegs.PIEIER1.bit.INTx1 = 1; // PIE 인터럽트(ADCA1INT) 활성화

   PieCtrlRegs.PIEIER3.bit.INTx1 = 1;
   PieCtrlRegs.PIEIER3.bit.INTx2 = 1; //for epwm2
   PieCtrlRegs.PIEIER3.bit.INTx3 = 1; //for epwm3
   PieCtrlRegs.PIEIER3.bit.INTx4 = 1; //for epwm4
   PieCtrlRegs.PIEIER3.bit.INTx7 = 1; //for epwm7
   IER |= M_INT3;  //epwm
   IER |= M_INT8; // Enable scic,d
   IER |= M_INT9; // Enable scia,b
   IER |= M_INT1; //enable group 1
   IER |= M_INT13;

   EINT;
   ERTM;  // Enable Global realtime interrupt DBGM




   DELAY_US(1000*1000);
   cmd=13;
	   ScibRegs.SCICTL1.bit.SWRESET=1;     // Relinquish SCI from Reset
	   ScibRegs.SCICTL1.bit.SWRESET=0;     // Relinquish SCI from Reset
	   ScibRegs.SCICTL1.bit.SWRESET=1;     // Relinquish SCI from Reset



	   IntMasterEnable();

	   fresult = f_mount(0, &g_sFatFs);
	   if(fresult != FR_OK)
	   {
	       scib_msg("f_mount error\r\n", 14);
	   }


	       memset(bufsd, 0, 64);


   // Step 6. IDLE loop. Just sit and loop forever (optional):


   	//motorstop();

    for(;;)
    {
    	if(sdcmd[0]=='0')
    	{

    		   fresult = f_open(&fil, "asdf.txt", FA_OPEN_ALWAYS | FA_READ| FA_WRITE);
    		       if(fresult != FR_OK)
    		       {
    		    	   scib_msg("f_open error\r\n", 13);
    		       }
    		       else
    		           		    {
    		           		    	scib_msg("f_open OK\r\n", 10);
    		           		    }


    		    sdcmd[0]='9';
    		    sdinitflag=1;


    	}

    	if(writeflag)

    	{
    	if(sdcmd[0]=='1'&&(CpuTimer1.InterruptCount%100==0))
    	{
    		if(sdinitflag)
    		{
    			  filsize = (&fil)->fsize;       //
    			    	       	fresult = f_lseek(&fil,filsize);  // it will write from first if comment this
    			    			spfout=usprintf(bufsd,"sdopen\r\n");
    			    			    			fresult = f_write(&fil,bufsd,spfout,&numoutsd);
    			    			fresult = f_sync(&fil);
    			    		    if(fresult != FR_OK)
    			    		    {
    			    		    	scib_msg("f_write init error\r\n", 14);
    			    		    }
    			    		    else
    			    		    {
    			    		    	scib_msg("w_init OK\r\n", 10);
    			    		    }
    			    		    sdinitflag=0;
    		}
    		imu.cmd[0]='1';
    		gps.cmd[0]='1';

    	spfout=usprintf(bufsd, "GPS=%s,%s,%s\r\n",gps.time,gps.Lay,gps.Long);
    			fresult = f_write(&fil,bufsd,spfout,&numoutsd);
    			spfout=usprintf(bufsd,"IMU=%s,%s,%s,%s,%s,%s\r\n",imu.vx,imu.vy,imu.vz,imu.sx,imu.sy,imu.sz);
    			    			fresult = f_write(&fil,bufsd,spfout,&numoutsd);
    			fresult = f_sync(&fil);
    		    if(fresult != FR_OK)
    		    {
    		    	scib_msg("f_write error\r\n", 14);
    		    }


    	}
    	else if(sdcmd[0]=='2')
    	{
    		imu.cmd[0]='0';
    		gps.cmd[0]='0';
			spfout=usprintf(bufsd,"\r\nsdend\r\n");
			fresult = f_write(&fil,bufsd,spfout,&numoutsd);
    		memset(bufsd, 0, 64);
    		fresult = f_close(&fil);
    				    	    if(fresult != FR_OK)
    				    	    {
    				    	    	scib_msg("f_close error\r\n", 14);
    				    	    }
    				    	    scib_msg("sdwrdone\r\n", 9);
    				    	    sdcmd[0]='0';

    	}

    	else if(sdcmd[0]=='3'&&(CpuTimer1.InterruptCount%10==0))
            	{
    		imu.cmd[0]='0';
    		gps.cmd[0]='0';

            		    fresult = f_read(&fil,rdsd,sizeof(rdsd)-1,&numoutsd);
            		    rdsd[numoutsd]=0;
            		    spfout=usprintf(bufsd,"%s",rdsd);
            		    scib_msg(bufsd,spfout);
            		    if(fresult != FR_OK)
            		    {
            		    	scib_msg("readerr\r\n", 8);
            		        sdcmd[0]='2';
            		    }
            		    if(numoutsd<sizeof(rdsd)-1)
            		    {
            		    	scib_msg("readend\r\n", 8);
            		    	sdcmd[0]='0';
            		    }
            	}

    	writeflag=0;
    	}



    	if(test==true)
    	{
    		test=false;
    		testprint();

    	}

    			//while(!bufferFull);
    	    	bufferFull = 0; //clear the buffer full flag



//    	    	if (cmd2==0)//전진
//    	    	    	{
//    	    	    		dir1=0;
//    	    	    		dir2=0;
//    	    	    	}
//    	    	    	else if (cmd2==1)//후진
//    	    	    	{
//    	    	    		dir1=1;
//    	    	    		dir2=1;
//    	    	    	}
//    	    	       	else if (cmd2==2)//좌회전
//    	    	        	{
//    	    	        		dir1=1;
//    	    	        		dir2=0;
//    	    	        	}
//
//    	    	       	else if (cmd2==3)//우회전
//    	    	        	{
//    	    	        		dir1=0;
//    	    	        		dir2=1;
//    	    	        	}







//    	        scia_msg("\n\nSD Card Example Program\r",0);
//    	        scia_msg("Type \'help\' for help.\r",0);
//    	        //
//    	        // Print a prompt to the console.  Show the CWD.
//    	        //
//
//    	        scia_msg("\n%s>",0);
//
//    	        //
//    	        // Get a line of text from the user.
//    	        //
//    	        UARTgets(g_cCmdBuf, sizeof(g_cCmdBuf));
//
//    	        //
//    	        // Pass the line from the user to the command processor.
//    	        // It will be parsed and valid commands executed.
//    	        //
//    	        nStatus = CmdLineProcess(g_cCmdBuf);
//
//    	        //
//    	        // Handle the case of bad command.
//    	        //
//    	        if(nStatus == CMDLINE_BAD_CMD)
//    	        {
//    	            UARTprintf("Bad command!\n");
//    	        }
//    	        //
//    	        // Handle the case of too many arguments.
//    	        //
//    	        else if(nStatus == CMDLINE_TOO_MANY_ARGS)
//    	        {
//    	            UARTprintf("Too many arguments for command processor!\n");
//    	        }
//
//    	        //
//    	        // Otherwise the command was executed.  Print the error
//    	        // code if one was returned.
//    	        //
//    	        else if(nStatus != 0)
//    	        {
//    	            UARTprintf("Command returned error code %s\n",
//    	                       StringFromFresult((FRESULT)nStatus));
//    	        }
//


    }

}






__interrupt void cpu_timer0_isr(void)
{

   CpuTimer0.InterruptCount++;
   Uint16 milcnt;
	Uint16 rxdrcv=0;






//		  dtxbuf[0]=0xff;
//		  dtxbuf[1]=0xee;
//		  dtxbuf[2]=0xdd;
//		  dtxbuf[3]=0xcc;
//		  dtxbuf[4]=0xbb;
//		  dtxbuf[5]=0xaa;
//		  dtxbuf[6]=0x11;
//		  dtxbuf[7]='\n';
//		  scid_msg(dtxbuf,8);
//		  fflush(dtxbuf);





//   GPIO_WritePin(66, 1);
//
//   //bulkreaddx(0x24);
//   //DELAY_US(200);
//
//   //if(mt==true&&t<4*endtime)
////   if(mt==true&&t<5)
////   {
////	   t=t+0.01;
////	  milcnt=t*10;
////	  traj[0]=trajcal(posp[milcnt],posp[milcnt+1],0.1,t-milcnt*0.1);
////	  traj[1]=trajcal(posk[milcnt],posk[milcnt+1],0.1,t-milcnt*0.1);
////
////		goalpos2[0]=traj[0]/2;
////		goalpos2[1]=traj[1]/2;
////		goalpos2[2]=traj[0]/2;
////		goalpos2[3]=traj[1]/2;
////		syncmv(4,goalpos2);
////			DELAY_US(500);
////		bulkreaddx(0x24);
////			DELAY_US(500);
////			//test=true;
////
////   }
////
////   else
////   {
////	   //ena=0;
////	   mt=false;
////	   t=0;
////   }
////
//
if (cmd == 0)
{
//     	led(uid,ledonoff); //led 켜기
//     	posinit();
     	ping(uid);

//	DELAY_US(1000);
     	cmd=100;
}
else if (cmd== 1) //read position
{

	if (uid !=0)
	{
	readdx(uid,0x24,0x02); //현재위치 id2
	dxl.cmd=1;
	}
	else
	{
		bulkreaddx(0x24);

	}


	cmd=100;
}
else if (cmd == 2)
{

	if (uid == 0)
	{
		mvf(2,intogo,intovel);//move float id,vel
		mvf(4,intogo,intovel);//move float id,vel
		mvf(6,intogo,intovel);//move float id,vel
		mvf(8,intogo,intovel);//move float id,vel
	}
	else
	{
		mvf(uid,intogo,intovel);//move float id,vel
		dxl.cmd=2;
	}
	cmd=100;
}

else if (cmd == 3)//경로실
{
	mt=true;
	cmd=100;
	t=0;
	ena=1;
	//ena=0;
}

else if (cmd == 4) //sync이동
	{
		goalpos[0]=degpel+degtogo/6;
		goalpos[1]=degknee-degtogo/6;
		goalpos[2]=degpel+degtogo/6;
		goalpos[3]=degknee-degtogo/6;
		syncmv(4,goalpos);
		//DELAY_US(300);
		//cmd=100;
		DELAY_US(100);
		//bulkreaddx(0x24);
		//cmd=5;
		//cmd=100;
	}

else if (cmd == 5) //read torque
	{

	readdx(uid,0x28,0x02);
	dxl.cmd=5;
	cmd=100;
	}

else if (cmd == 6) //read current
	{

	readdx(uid,0x44,0x02);
	}

else if (cmd == 7) //job
	{
	dxl.cmd=1;
	readdx(uid,0x24,0x02); //현재위치 id2

	cmd=71;
	}
else if (cmd == 71) //job
	{
	dxl.cmd=5;
	readdx(uid,0x28,0x02); //현재토크 id2

	cmd=72;
	}
else if (cmd == 72) //job
	{
	mvf(uid,intogo,intovel);//move float id,vel
	dxl.cmd=2;

	cmd=73;
	}
else if (cmd == 73) //job
	{

	cmd=7;
	}


else if (cmd == 11)
{
	setccwlimit(uid,0xFFF);
}

else if (cmd == 12)
{
	if (toen!=0)
	{
		Uint8 p[]={0x18,0x01};
		senddx(0x02,0x04,p);// torque enable
//		senddx(0x04,0x04,p);// torque enable
//		senddx(0x06,0x04,p);// torque enable
//		senddx(0x08,0x04,p);// torque enable
	}
	else
	{
		Uint8 p2[]={0x18,0x00};
		senddx(0x02,0x04,p2);// torque disable
//		senddx(0x04,0x04,p2);// torque disable
//		senddx(0x06,0x04,p2);// torque disable
//		senddx(0x08,0x04,p2);// torque disable
	}
	dxl.cmd=12;
	cmd=100;
}

else if (cmd == 13)
{

	readdx(uid,0x24,0x02); //현재위치 id2
	dxl.cmd=1;
	cmd=131;
}

else if (cmd == 131)
{
	oset[1]=dxl.ipos;
	cmd=12;
}
else if (cmd == 96) //torque limit
{
	readdx(uid,0x22,0x02);

}

else if (cmd == 97)
{
		Uint8 p[]={0x03,0x06};
		senddx(uid,0x04,p);// id설정 6

}


else if (cmd == 98)
{
		Uint8 p[]={0x04,0x01};
		senddx(uid,0x04,p);// 속도설정 1mbps

}

else if (cmd == 99)
{
	reset(uid);
}




else if (cmd == 1001)
{
	ATmode=1;
	char *attemp;
	attemp="AT+MRESET\r";
	scib_msg(attemp,0);
	cmd =100;
}

else if (cmd == 1002)
{
	CpuTimer1.InterruptCount=0;
	memset(itoabuf,0, sizeof(itoabuf));
	autoloop=1;
	cmd=100;
}



//
//ATpnt();
//
//
//
//
//
//
//
//
   if( ScidRegs.SCIRXBUF.bit.SCIFFFE==1||ScidRegs.SCIRXST.bit.RXERROR==1) //framing error
     {

     	//ScicRegs.SCIFFRX.bit.RXFIFORESET=1;
     	   ScidRegs.SCICTL1.bit.SWRESET=1;     // Relinquish SCI from Reset
     	   ScidRegs.SCICTL1.bit.SWRESET=0;     // Relinquish SCI from Reset
     	   ScidRegs.SCICTL1.bit.SWRESET=1;     // Relinquish SCI from Reset


     }
//
//   // Acknowledge this interrupt to receive more interrupts from group 1
   PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;



//
//
//
//
//
   //	 while (ScicRegs.SCICTL2.bit.TXEMPTY != 1) {}
//	 GPIO_WritePin(66, 0);           //485 tx disable
















}





void ConfigureADC(void)
{
	EALLOW;

	//write configurations
	AdcaRegs.ADCCTL2.bit.PRESCALE = 6; //set ADCCLK divider to /4
    AdcSetMode(ADC_ADCA, ADC_RESOLUTION_12BIT, ADC_SIGNALMODE_SINGLE);

	//Set pulse positions to late
	AdcaRegs.ADCCTL1.bit.INTPULSEPOS = 1;

	//power up the ADC
	AdcaRegs.ADCCTL1.bit.ADCPWDNZ = 1;

	//delay for 1ms to allow ADC time to power up
	DELAY_US(1000);

	EDIS;
}

void SetupADCEpwm(Uint16 channel)
{
	Uint16 acqps;

	//determine minimum acquisition window (in SYSCLKS) based on resolution
	if(ADC_RESOLUTION_12BIT == AdcaRegs.ADCCTL2.bit.RESOLUTION){
		acqps = 14; //75ns
	}
	else { //resolution is 16-bit
		acqps = 63; //320ns
	}

	//Select the channels to convert and end of conversion flag
	EALLOW;
	AdcaRegs.ADCSOC0CTL.bit.CHSEL = channel;  //SOC0 will convert pin A0
	AdcaRegs.ADCSOC0CTL.bit.ACQPS = acqps; //sample window is 100 SYSCLK cycles
	//AdcaRegs.ADCSOC0CTL.bit.TRIGSEL = 7; //trigger on ePWM2 SOCA/C
	AdcaRegs.ADCSOC0CTL.bit.TRIGSEL = 11; //trigger on ePWM2 SOCA/C
	AdcaRegs.ADCINTSEL1N2.bit.INT1SEL = 0; //end of SOC0 will set INT1 flag
	AdcaRegs.ADCINTSEL1N2.bit.INT1E = 1;   //enable INT1 flag
	AdcaRegs.ADCINTFLGCLR.bit.ADCINT1 = 1; //make sure INT1 flag is cleared
}

interrupt void adca1_isr(void)
{
//	degtogo=AdcaResultRegs.ADCRESULT0*0.2;
	degtogo=AdcaResultRegs.ADCRESULT0*0.1;
	//AdcaResults[resultsIndex++] = AdcaResultRegs.ADCRESULT0;
	if(RESULTS_BUFFER_SIZE <= resultsIndex)
	{
		resultsIndex = 0;
		bufferFull = 1;
	}

	AdcaRegs.ADCINTFLGCLR.bit.ADCINT1 = 1; //clear INT1 flag
	PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;
}


void motorstop()
{
	   //ena=1;
	   toen=0;
	   cmd=12;
}

//float trajcal(float s0,float sf,float T,float t)
//{
//	float s;
//	s=6*(sf-s0)*pow(t/T,5)+15*(s0-sf)*pow(t/T,4)+10*(sf-s0)*pow(t/T,3)+s0;
//	return s;
//
//}

void ConfigureGPIO(void)
{


	   //scirxa
//	   GPIO_SetupPinMux(85, GPIO_MUX_CPU1, 5);
//	   GPIO_SetupPinOptions(85, GPIO_INPUT,  GPIO_PULLUP);
//	   GpioCtrlRegs.GPCQSEL2.bit.GPIO85 = 3;

//	   GPIO_SetupPinMux(43, GPIO_MUX_CPU1, 15);
//	   GPIO_SetupPinOptions(43, GPIO_INPUT,  GPIO_PULLUP);
//	   GpioCtrlRegs.GPBQSEL1.bit.GPIO43 = 3;

	   GPIO_SetupPinMux(28, GPIO_MUX_CPU1, 1);
	   GPIO_SetupPinOptions(28, GPIO_INPUT,  GPIO_PULLUP);

	   //scitxa
//	   GPIO_SetupPinMux(84, GPIO_MUX_CPU1, 5);
//	   GPIO_SetupPinOptions(84, GPIO_OUTPUT,  GPIO_PULLUP);
//	   GpioCtrlRegs.GPCQSEL2.bit.GPIO84 = 3;

//	   GPIO_SetupPinMux(42, GPIO_MUX_CPU1, 15);
//	   GPIO_SetupPinOptions(42, GPIO_OUTPUT,  GPIO_PULLUP);
//	   GpioCtrlRegs.GPBQSEL1.bit.GPIO42 = 3;

	   GPIO_SetupPinMux(29, GPIO_MUX_CPU1, 1);
	   GPIO_SetupPinOptions(29, GPIO_OUTPUT,  GPIO_PULLUP);

	   //scirxb
//	   GPIO_SetupPinMux(87, GPIO_MUX_CPU1, 5);
//	   GPIO_SetupPinOptions(87, GPIO_INPUT, GPIO_PULLUP);
//	   GpioCtrlRegs.GPCQSEL2.bit.GPIO87 = 3;

	   GPIO_SetupPinMux(55, GPIO_MUX_CPU1, 6);
	   GPIO_SetupPinOptions(55, GPIO_INPUT, GPIO_PULLUP);

	   //scitxb
//	   GPIO_SetupPinMux(86, GPIO_MUX_CPU1, 5);
//	   GPIO_SetupPinOptions(86, GPIO_OUTPUT, GPIO_PULLUP);
//	   GpioCtrlRegs.GPCQSEL2.bit.GPIO86 = 3;

	   GPIO_SetupPinMux(54, GPIO_MUX_CPU1, 6);
	   GPIO_SetupPinOptions(54, GPIO_OUTPUT, GPIO_PUSHPULL);

	   // direction
//	   GPIO_SetupPinMux(66, GPIO_MUX_CPU1, 0);
//	   GPIO_SetupPinOptions(66, GPIO_OUTPUT, GPIO_PUSHPULL);
//

	   //scirxc
	   GPIO_SetupPinMux(57, GPIO_MUX_CPU1, 6);
	   GPIO_SetupPinOptions(57, GPIO_INPUT, GPIO_PUSHPULL);

	   //scitxc
	   GPIO_SetupPinMux(56, GPIO_MUX_CPU1, 1);
	   GPIO_SetupPinOptions(56, GPIO_OUTPUT, GPIO_PUSHPULL);


	   // direction
	   GPIO_SetupPinMux(22, GPIO_MUX_CPU1, 0);
	   GPIO_SetupPinOptions(22, GPIO_OUTPUT, GPIO_PULLUP);


	   //scirxd
	   GPIO_SetupPinMux(77, GPIO_MUX_CPU1, 6);
	   GPIO_SetupPinOptions(77, GPIO_INPUT, GPIO_PUSHPULL);

	   //scitxd
	   GPIO_SetupPinMux(76, GPIO_MUX_CPU1, 6);
	   GPIO_SetupPinOptions(76, GPIO_OUTPUT, GPIO_PUSHPULL);


	   //step_left

	   //dir
	   GPIO_SetupPinMux(2, GPIO_MUX_CPU1, 0);
	   GPIO_SetupPinOptions(2, GPIO_OUTPUT, GPIO_PUSHPULL);

	   //speed
	   GPIO_SetupPinMux(0, GPIO_MUX_CPU1, 1);
	   GPIO_SetupPinOptions(0, GPIO_OUTPUT, GPIO_PUSHPULL);

	   //sleep
	   GPIO_SetupPinMux(8, GPIO_MUX_CPU1, 0);
	   GPIO_SetupPinOptions(8, GPIO_OUTPUT, GPIO_PUSHPULL);

	   //reset
	   GPIO_SetupPinMux(9, GPIO_MUX_CPU1, 0);
	   GPIO_SetupPinOptions(9, GPIO_OUTPUT, GPIO_PUSHPULL);

	   //enable
	   GPIO_SetupPinMux(10, GPIO_MUX_CPU1, 0);
	   GPIO_SetupPinOptions(10, GPIO_OUTPUT, GPIO_PULLUP);

	   //step_right

	   //dir
	   GPIO_SetupPinMux(6, GPIO_MUX_CPU1, 0);
	   GPIO_SetupPinOptions(6, GPIO_OUTPUT, GPIO_PUSHPULL);

	   //speed
	   GPIO_SetupPinMux(4, GPIO_MUX_CPU1, 1);
	   GPIO_SetupPinOptions(4, GPIO_OUTPUT, GPIO_PUSHPULL);



//	   //for absolute encoder
//	   //pan
//
//	   //data
//	   GPIO_SetupPinMux(20, GPIO_MUX_CPU1, 0);
//	   GPIO_SetupPinOptions(20, GPIO_INPUT, GPIO_PULLUP);
//
//	   //clock
//	   GPIO_SetupPinMux(94, GPIO_MUX_CPU1, 0);
//	   GPIO_SetupPinOptions(94, GPIO_OUTPUT, GPIO_PUSHPULL);
//
//	   //clock
//	   GPIO_SetupPinMux(47, GPIO_MUX_CPU1, 0);
//	   GPIO_SetupPinOptions(47, GPIO_OUTPUT, GPIO_PUSHPULL);
//
//
//	   //gpio_dir2
//	   GPIO_SetupPinMux(18, GPIO_MUX_CPU1, 0);
//	   GPIO_SetupPinOptions(18, GPIO_OUTPUT, GPIO_PUSHPULL);
//
//	   //gpio_dir3
//	   GPIO_SetupPinMux(19, GPIO_MUX_CPU1, 0);
//	   GPIO_SetupPinOptions(19, GPIO_OUTPUT, GPIO_PUSHPULL);
//
//
//	   //tilt
//
//	   //data
//	   GPIO_SetupPinMux(78, GPIO_MUX_CPU1, 0);
//	   GPIO_SetupPinOptions(78, GPIO_INPUT, GPIO_PUSHPULL);
//
//	   //clock
//	   GPIO_SetupPinMux(83, GPIO_MUX_CPU1, 0);
//	   GPIO_SetupPinOptions(83, GPIO_OUTPUT, GPIO_PUSHPULL);
//
//	   //clock
//	   GPIO_SetupPinMux(49, GPIO_MUX_CPU1, 0);
//	   GPIO_SetupPinOptions(49, GPIO_OUTPUT, GPIO_PUSHPULL);
//








	   GPIO_SetupPinMux(82, GPIO_MUX_CPU1, 0);
	   GPIO_SetupPinOptions(82, GPIO_OUTPUT, GPIO_PUSHPULL);


	   GPIO_SetupPinMux(75, GPIO_MUX_CPU1, 0);
	   GPIO_SetupPinOptions(75, GPIO_OUTPUT, GPIO_PUSHPULL);
	   GPIO_SetupPinMux(81, GPIO_MUX_CPU1, 0);
	   GPIO_SetupPinOptions(81, GPIO_OUTPUT, GPIO_PUSHPULL);


	   GPIO_SetupPinMux(88, GPIO_MUX_CPU1, 0);
	   GPIO_SetupPinOptions(88, GPIO_OUTPUT, GPIO_PUSHPULL);
}

void testprint(void)
{


    //printf("%i,%i,%i,%i \n",(int)(pos[1]*100), (int)(pos[3]*100),(int)(pos[5]*100), (int)(pos[7]*100));
	//sprintf(buf,"time: %f \n",0.01);
	//printf("%s \n",buf);
    //fflush(stdout);
}

void posinit(void)
{

	int avgcnt;
	for (avgcnt=0;avgcnt<10;avgcnt++)
	{
		bulkreaddx(0x24);
		DELAY_US(500);
		posavg[1]=posavg[1]+pos[1];
		posavg[3]=posavg[3]+pos[3];
		posavg[5]=posavg[5]+pos[5];
		posavg[7]=posavg[7]+pos[7];
	}

	posavg[1]=posavg[1]/10;
	posavg[3]=posavg[3]/10;
	posavg[5]=posavg[5]/10;
	posavg[7]=posavg[7]/10;

	oset[1]=-dir[1]*(-45-posavg[1])*gearrt[1]*4096/360;
	oset[3]=-dir[3]*(45-posavg[3])*gearrt[3]*4096/360;
	oset[5]=-dir[5]*(-45-posavg[5])*gearrt[5]*4096/360;
	oset[7]=-dir[7]*(45-posavg[7])*gearrt[7]*4096/360;

}


#ifdef __cplusplus
#pragma CODE_SECTION("ramfuncs");
#endif
__interrupt void cpu_timer1_isr(void)
{
	int jj;
	Uint16 templ,numout;
	char *tbufd;
	float Ts=0.01;
	float tmpl,tmpr;




	  prevtime=CpuTimer0.InterruptCount;


//		//for absencoder
//		Uint8 bit_count;
//		Uint32 u32result=0;
//		Uint8  u8portdata;
//		Uint32 u32result2=0;
//		Uint8  u8portdata2;



//	if(autoloop)
//	{
//   CpuTimer1.InterruptCount++;
//
//      if (CpuTimer1.InterruptCount==1)
//        {
//     	   cmd=1001;
//
//     	   strcpy(ATmsg,"AT+MRESET\r");
//
//    	   scib_msg(ATmsg,0);
//
//        }
//
//
//      else if (CpuTimer1.InterruptCount>1000)
//   {
//
//    	  temp=itoa(CpuTimer1.InterruptCount,itoabuf,10);
//    	  usprintf(ATmsg,"AT+SSEND=0,,,%u\r",ustrlen(temp)+7);
//    	  scib_msg(ATmsg,0);
//
//   	usprintf(ATmsg2,"Timer=%u\r",(unsigned long)CpuTimer1.InterruptCount);
//	   scib_msg(ATmsg2,0);
//   }
//
//
//
//	}





	  //ping(2);
	  //usprintf(dtxbuf,"%s",0xAD);


//
//

//

//
//
//
//  	ScidRegs.SCITXBUF.all =0xA;
//	DELAY_US(1);











//	  fflush(printbuf);
//	  fflush(strbuf);






//// --for abs encoder -------------
//	  for (bit_count=0;bit_count<17;bit_count++)
//	  {
//		  GPIO_WritePin(47, 0);
//
//		  u32result=(u32result << 1);
//		  u8portdata = (Uint8)GPIO_ReadPin(20);
//
//		  DELAY_US(1);
//		  GPIO_WritePin(47, 1);
//		  if (u8portdata!=0)
//		  {
//			  u32result=u32result | 0x1;
//		  }
//
//		  DELAY_US(1);
//	  }
//
//	  for (bit_count=0;bit_count<17;bit_count++)
//	  {
//		  GPIO_WritePin(49, 0);
//
//
//		  u32result2=(u32result2 << 1);
//		  u8portdata2 = (Uint8)GPIO_ReadPin(78);
//
//		  DELAY_US(1);
//		  GPIO_WritePin(49, 1);
//
//
//		  if (u8portdata2!=0)
//		  {
//			  u32result2=u32result2 | 0x1;
//		  }
//		  DELAY_US(1);
//	  }
//	  aben_p=u32result&0xFFFF;
//	  aben_t=u32result2&0xFFFF;
//
//	  GPIO_WritePin(18, 0);
//	  GPIO_WritePin(19, 0);

if(ena)
{
	if((ltraold-desvl)*(ltraold-desvl)>1e-6)
	{
		tmpl=(desvl-ltraold)/100;
		if(tmpl<0)
			tmpl=-tmpl;
		if(tmpl<0.1)
			tmpl=0.1;
		if(ttra<=tmpl)
		{
			ttra=ttra+Ts;
			ltra=tragen(ltraold,desvl,ttra,tmpl);

		}
		else
		{
			ttra=0;
			ltraold=ltra;
		}


	}

	if((rtraold-desvr)*(rtraold-desvr)>1e-6)
	{
		tmpr=(desvr-rtraold)/100;
		if(tmpr<0)
			tmpr=-tmpr;
		if(tmpr<0.1)
			tmpr=0.1;
		if(ttrar<=tmpr)
		{
			ttrar=ttrar+Ts;
			rtra=tragen(rtraold,desvr,ttrar,tmpr);

		}
		else
		{
			ttrar=0;
			rtraold=rtra;
		}


	}

}


checkrxd();

















	curtime=CpuTimer0.InterruptCount;
	excutetime=(curtime-prevtime);


writeflag=1;
	CpuTimer1.InterruptCount++;
}

#ifdef __cplusplus
#pragma CODE_SECTION("ramfuncs");
#endif

Uint16 ATpnt()
{
	int i;

	 if (atpntflag)
	 {
	 if(ScibRegs.SCIFFRX.bit.RXFFST!=0) return 0;
//     scia_msg(ATrxmsg,0);
	 atpntflag=0;
	 i=ATcnt2;
	 ATcnt2=0;
	 return i;
	 }
}

static char *i2a(unsigned i, char *a, unsigned r)
{
    if (i/r > 0) a = i2a(i/r,a,r);
    *a = "0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZ"[i%r];
    return a+1;
}
char *itoa(int i, char *a, int r)
{
    if ((r < 2) || (r > 36)) r = 10;
    if (i < 0)
    {
        *a = '-';
        *i2a(-(unsigned)i,a+1,r) = 0;
    }
    else *i2a(i,a,r) = 0;
    return a;
}





//===========================================================================
// No more.
//===========================================================================

