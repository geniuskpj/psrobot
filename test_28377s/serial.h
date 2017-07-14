/*
 * serial.h
 *
 *  Created on: 2016. 10. 14.
 *      Author: ps
 */

#ifndef SERIAL_H_
#define SERIAL_H_


//#include <stdarg.h>





Uint16 UARTgets(char *pcBuf, Uint16 Len,Uint16 base);
unsigned char UARTgetc(Uint16 base);
Uint16 UARTPeek(unsigned char ucChar, Uint16 base);

typedef enum {
    OK,            /* 0 */
    ERROR,        /* 1 */

} STATUS;


typedef struct Wizfi {

STATUS status;
char cmd[20];
} WIZ;

typedef struct Imu {

STATUS status;
char cmd[20];
char roll[10];
char pitch[10];
char yaw[10];
char vx[10];
char vy[10];
char vz[10];
char sx[10];
char sy[10];
char sz[10];
} IMU;

typedef struct Gps {

STATUS status;
char cmd[20];
char time[10];
char Lay[11];
char Long[12];

} GPS;



typedef struct Dx {

STATUS status;
Uint16 cmd;
Uint16 error;
Uint16 id;
Uint16 cksum;
int ipos;
int itor;
float desp;
float pos;
float tor;
char cmdin[5];
char posin[4];
char ENA[2];

} DX;

typedef struct Mx {

STATUS status;
char cmd[2];
char ENA[2];
char DIR[2];
char DESVBUF[4];
float desv;
} MX;








#endif /* SERIAL_H_ */
