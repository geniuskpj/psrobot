/*
 * psuart.h
 *
 *  Created on: 2017. 4. 14.
 *      Author: ps
 */

#ifndef PSUART_H_
#define PSUART_H_

#ifndef UART_RX_BUFFER_SIZE
#define UART_RX_BUFFER_SIZE     256
#endif
#ifndef UART_TX_BUFFER_SIZE
#define UART_TX_BUFFER_SIZE    256
#endif




///////////////ring buffer
#define UART_BUFFERED;
#ifdef UART_BUFFERED
// Output ring buffer.  Buffer is full if g_ui16UARTTxReadIndex is one ahead of
// g_ui16UARTTxWriteIndex.  Buffer is empty if the two indices are the same.
//

unsigned char a_pcUARTTxBuffer[UART_TX_BUFFER_SIZE];
Uint16 a_ui16UARTTxWriteIndex = 0;
Uint16 a_ui16UARTTxReadIndex = 0;

//
// Input ring buffer.  Buffer is full if g_ui16UARTTxReadIndex is one ahead of
// g_ui16UARTTxWriteIndex.  Buffer is empty if the two indices are the same.
//
unsigned char a_pcUARTRxBuffer[UART_RX_BUFFER_SIZE];
Uint16 a_ui16UARTRxWriteIndex = 0;
Uint16 a_ui16UARTRxReadIndex = 0;


//
// Macros to determine number of free and used bytes in the transmit buffer.
//
#define aTX_BUFFER_USED          (GetBufferCount(&a_ui16UARTTxReadIndex,  \
                                                &a_ui16UARTTxWriteIndex, \
                                                UART_TX_BUFFER_SIZE))
#define aTX_BUFFER_FREE          (UART_TX_BUFFER_SIZE - aTX_BUFFER_USED)
#define aTX_BUFFER_EMPTY         (IsBufferEmpty(&a_ui16UARTTxReadIndex,   \
                                               &a_ui16UARTTxWriteIndex))
#define aTX_BUFFER_FULL          (IsBufferFull(&a_ui16UARTTxReadIndex,  \
                                              &a_ui16UARTTxWriteIndex, \
                                              UART_TX_BUFFER_SIZE))
#define ADVANCE_TX_BUFFER_INDEX(Index) \
                                (Index) = ((Index) + 1) % UART_TX_BUFFER_SIZE

//
// Macros to determine number of free and used bytes in the receive buffer.
//
#define aRX_BUFFER_USED          (GetBufferCount(&a_ui16UARTRxReadIndex,  \
                                                &a_ui16UARTRxWriteIndex, \
                                                UART_RX_BUFFER_SIZE))
#define aRX_BUFFER_FREE          (UART_RX_BUFFER_SIZE - aRX_BUFFER_USED)
#define aRX_BUFFER_EMPTY         (IsBufferEmpty(&a_ui16UARTRxReadIndex,   \
                                               &a_ui16UARTRxWriteIndex))
#define aRX_BUFFER_FULL          (IsBufferFull(&a_ui16UARTRxReadIndex,  \
                                              &a_ui16UARTRxWriteIndex, \
                                              UART_RX_BUFFER_SIZE))
#define ADVANCE_RX_BUFFER_INDEX(Index) (Index) = ((Index) + 1) % UART_RX_BUFFER_SIZE








// Output ring buffer.  Buffer is full if g_ui16UARTTxReadIndex is one ahead of
// g_ui16UARTTxWriteIndex.  Buffer is empty if the two indices are the same.
//

unsigned char b_pcUARTTxBuffer[UART_TX_BUFFER_SIZE];
Uint16 b_ui16UARTTxWriteIndex = 0;
Uint16 b_ui16UARTTxReadIndex = 0;



// Input ring buffer.  Buffer is full if g_ui16UARTTxReadIndex is one ahead of
// g_ui16UARTTxWriteIndex.  Buffer is empty if the two indices are the same.
//
unsigned char b_pcUARTRxBuffer[UART_RX_BUFFER_SIZE];
Uint16 b_ui16UARTRxWriteIndex = 0;
Uint16 b_ui16UARTRxReadIndex = 0;


//
// Macros to determine number of free and used bytes in the transmit buffer.
//
#define bTX_BUFFER_USED          (GetBufferCount(&b_ui16UARTTxReadIndex,  \
                                                &b_ui16UARTTxWriteIndex, \
                                                UART_TX_BUFFER_SIZE))
#define bTX_BUFFER_FREE          (UART_TX_BUFFER_SIZE - bTX_BUFFER_USED)
#define bTX_BUFFER_EMPTY         (IsBufferEmpty(&b_ui16UARTTxReadIndex,   \
                                               &b_ui16UARTTxWriteIndex))
#define bTX_BUFFER_FULL          (IsBufferFull(&b_ui16UARTTxReadIndex,  \
                                              &b_ui16UARTTxWriteIndex, \
                                              UART_TX_BUFFER_SIZE))


//
// Macros to determine number of free and used bytes in the receive buffer.
//
#define bRX_BUFFER_USED          (GetBufferCount(&b_ui16UARTRxReadIndex,  \
                                                &b_ui16UARTRxWriteIndex, \
                                                UART_RX_BUFFER_SIZE))
#define bRX_BUFFER_FREE          (UART_RX_BUFFER_SIZE - bRX_BUFFER_USED)
#define bRX_BUFFER_EMPTY         (IsBufferEmpty(&b_ui16UARTRxReadIndex,   \
                                               &b_ui16UARTRxWriteIndex))
#define bRX_BUFFER_FULL          (IsBufferFull(&b_ui16UARTRxReadIndex,  \
                                              &b_ui16UARTRxWriteIndex, \
                                              UART_RX_BUFFER_SIZE))


// Output ring buffer.  Buffer is full if g_ui16UARTTxReadIndex is one ahead of
// g_ui16UARTTxWriteIndex.  Buffer is empty if the two indices are the same.
//

unsigned char c_pcUARTTxBuffer[UART_TX_BUFFER_SIZE];
Uint16 c_ui16UARTTxWriteIndex = 0;
Uint16 c_ui16UARTTxReadIndex = 0;



// Input ring buffer.  Buffer is full if g_ui16UARTTxReadIndex is one ahead of
// g_ui16UARTTxWriteIndex.  Buffer is empty if the two indices are the same.
//
unsigned char c_pcUARTRxBuffer[UART_RX_BUFFER_SIZE];
Uint16 c_ui16UARTRxWriteIndex = 0;
Uint16 c_ui16UARTRxReadIndex = 0;



//
// Macros to determine number of free and used bytes in the transmit buffer.
//
#define cTX_BUFFER_USED          (GetBufferCount(&c_ui16UARTTxReadIndex,  \
                                                &c_ui16UARTTxWriteIndex, \
                                                UART_TX_BUFFER_SIZE))
#define cTX_BUFFER_FREE          (UART_TX_BUFFER_SIZE - cTX_BUFFER_USED)
#define cTX_BUFFER_EMPTY         (IsBufferEmpty(&c_ui16UARTTxReadIndex,   \
                                               &d_ui16UARTTxWriteIndex))
#define cTX_BUFFER_FULL          (IsBufferFull(&c_ui16UARTTxReadIndex,  \
                                              &c_ui16UARTTxWriteIndex, \
                                              UART_TX_BUFFER_SIZE))


//
// Macros to determine number of free and used bytes in the receive buffer.
//
#define cRX_BUFFER_USED          (GetBufferCount(&c_ui16UARTRxReadIndex,  \
                                                &c_ui16UARTRxWriteIndex, \
                                                UART_RX_BUFFER_SIZE))
#define cRX_BUFFER_FREE          (UART_RX_BUFFER_SIZE - cRX_BUFFER_USED)
#define cRX_BUFFER_EMPTY         (IsBufferEmpty(&c_ui16UARTRxReadIndex,   \
                                               &c_ui16UARTRxWriteIndex))
#define cRX_BUFFER_FULL          (IsBufferFull(&c_ui16UARTRxReadIndex,  \
                                              &c_ui16UARTRxWriteIndex, \
                                              UART_RX_BUFFER_SIZE))




























// Output ring buffer.  Buffer is full if g_ui16UARTTxReadIndex is one ahead of
// g_ui16UARTTxWriteIndex.  Buffer is empty if the two indices are the same.
//

unsigned char d_pcUARTTxBuffer[UART_TX_BUFFER_SIZE];
Uint16 d_ui16UARTTxWriteIndex = 0;
Uint16 d_ui16UARTTxReadIndex = 0;



// Input ring buffer.  Buffer is full if g_ui16UARTTxReadIndex is one ahead of
// g_ui16UARTTxWriteIndex.  Buffer is empty if the two indices are the same.
//
unsigned char d_pcUARTRxBuffer[UART_RX_BUFFER_SIZE];
Uint16 d_ui16UARTRxWriteIndex = 0;
Uint16 d_ui16UARTRxReadIndex = 0;



//
// Macros to determine number of free and used bytes in the transmit buffer.
//
#define dTX_BUFFER_USED          (GetBufferCount(&d_ui16UARTTxReadIndex,  \
                                                &d_ui16UARTTxWriteIndex, \
                                                UART_TX_BUFFER_SIZE))
#define dTX_BUFFER_FREE          (UART_TX_BUFFER_SIZE - dTX_BUFFER_USED)
#define dTX_BUFFER_EMPTY         (IsBufferEmpty(&d_ui16UARTTxReadIndex,   \
                                               &d_ui16UARTTxWriteIndex))
#define dTX_BUFFER_FULL          (IsBufferFull(&d_ui16UARTTxReadIndex,  \
                                              &d_ui16UARTTxWriteIndex, \
                                              UART_TX_BUFFER_SIZE))


//
// Macros to determine number of free and used bytes in the receive buffer.
//
#define dRX_BUFFER_USED          (GetBufferCount(&d_ui16UARTRxReadIndex,  \
                                                &d_ui16UARTRxWriteIndex, \
                                                UART_RX_BUFFER_SIZE))
#define dRX_BUFFER_FREE          (UART_RX_BUFFER_SIZE - dRX_BUFFER_USED)
#define dRX_BUFFER_EMPTY         (IsBufferEmpty(&d_ui16UARTRxReadIndex,   \
                                               &d_ui16UARTRxWriteIndex))
#define dRX_BUFFER_FULL          (IsBufferFull(&d_ui16UARTRxReadIndex,  \
                                              &d_ui16UARTRxWriteIndex, \
                                              UART_RX_BUFFER_SIZE))




#endif



#endif /* PSUART_H_ */
