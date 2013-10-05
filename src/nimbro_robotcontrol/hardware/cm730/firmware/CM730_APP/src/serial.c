/************************* (C) COPYRIGHT 2010 ROBOTIS **************************
* File Name          : serial.c
* Author             : zerom
* Version            : V0.0.1
* Date               : 08/26/2010
* Description        : functions about serial control
*******************************************************************************/

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x_type.h"
#include "serial.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

void TxDString(u8 PORT, u8 *bData)
{
    while(*bData)
        TxDData(PORT, *bData++);
}

void TxDHex8(u16 bSentData)
{
    u16 bTmp;

    bTmp = ((bSentData>>4)&0x000f) + (u8)'0';
    if(bTmp > '9') bTmp += 7;
    //TxDData(USART_ZIGBEE,bTmp);
    TxDData(USART_ZIGBEE,bTmp);

    bTmp = (bSentData & 0x000f) + (u8)'0';
    if(bTmp > '9') bTmp += 7;
    //TxDData(USART_ZIGBEE,bTmp);
    TxDData(USART_ZIGBEE,bTmp);
}

void TxDHex16(u16 wSentData)
{
    TxDHex8((wSentData>>8)&0x00ff );
    TxDHex8( wSentData&0x00ff);
}

void TxDHex32(u32 lSentData)
{
    TxDHex16((lSentData>>16)&0x0000ffff );
    TxDHex16( lSentData&0x0000ffff);
}

void TxD_Dec_U8(u8 bByte)
{
    u8 bTmp;
    bTmp = bByte/100;
    /*if(bTmp)*/ TxDData( USART_PC, bTmp+'0');
    bByte -= bTmp*100;
    bTmp = bByte/10;
    /*if(bTmp)*/ TxDData( USART_PC, bTmp+'0');
    TxDData( USART_PC, bByte - bTmp*10+'0');
}

void TxD_Dec_U16(u16 wData)
{
    u8 bCount, bPrinted;
    u16 wTmp,wDigit;
    bPrinted = 0;

    wDigit = 10000;
    for(bCount = 0; bCount < 5; bCount++)
    {
        wTmp = (wData/wDigit);
        if(wTmp)
        {
            TxDData( USART_PC,((u8)wTmp)+'0');
            bPrinted = 1;
        }
        else
        {
            if(bPrinted) TxDData( USART_PC,((u8)wTmp)+'0');
            else
            {
                if(bCount < 4) TxDData( USART_PC,' ');
                else TxDData( USART_PC,'0');
            }
        }
        wData -= wTmp*wDigit;
        wDigit /= 10;
    }
}


void TxD_Dec_U32(u32 wData)
{
    u8 bCount, bPrinted;
    u32 wTmp,wDigit;
    bPrinted = 0;

    wDigit = 1000000000;

    for(bCount = 0; bCount < 10; bCount++)
    {
        wTmp = (wData/wDigit);
        if(wTmp)
        {
            TxDData( USART_PC,((u8)wTmp)+'0');
            bPrinted = 1;
        }
        else
        {
            if(bPrinted) TxDData( USART_PC,((u8)wTmp)+'0');
            else
            {
                if(bCount < 4) TxDData( USART_PC,' ');
                else TxDData( USART_PC,'0');
            }
        }
        wData -= wTmp*wDigit;
        wDigit /= 10;
    }
}


void TxD_Dec_S8(s8 wData)
{
    u8 bCount, bPrinted;
    u16 wTmp,wDigit;
    u8 bMinus = 0;

    bPrinted = 0;

    if (wData&0x80) {
        bMinus = 1;
        wData = -wData;
    }

    wDigit = 100;
    for(bCount = 0; bCount < 3; bCount++)
    {
        wTmp = (wData/wDigit);
        if(wTmp && !bPrinted)
        {
            if (bMinus) TxDData( USART_PC,'-');
            TxDData( USART_PC,((u8)wTmp)+'0');
            bPrinted = 1;
        }
        else
        {
            if(bPrinted) TxDData( USART_PC,((u8)wTmp)+'0');
            else
            {
                if(bCount < 4) TxDData( USART_PC,' ');
                else TxDData( USART_PC, '0');
            }
        }
        wData -= wTmp*wDigit;
        wDigit /= 10;
    }
}

void TxD_Dec_S16(s16 wData)
{
    u8 bCount, bPrinted;
    u16 wTmp,wDigit;
    u8 bMinus = 0;

    bPrinted = 0;

    if (wData&0x8000) {
        bMinus = 1;
        wData = -wData;
    }

    wDigit = 10000;
    for(bCount = 0; bCount < 5; bCount++)
    {
        wTmp = (wData/wDigit);
        if(wTmp && !bPrinted)
        {
            if (bMinus) TxDData( USART_PC,'-');
            TxDData( USART_PC,((u8)wTmp)+'0');
            bPrinted = 1;
        }
        else
        {
            if(bPrinted) TxDData( USART_PC,((u8)wTmp)+'0');
            else
            {
                if(bCount < 4) TxDData( USART_PC,' ');
                else TxDData( USART_PC, '0');
            }
        }
        wData -= wTmp*wDigit;
        wDigit /= 10;
    }
}


void TxD_Dec_S32(s32 lLong)
{
    u8 bCount, bPrinted;
    s32 lTmp,lDigit;
    bPrinted = 0;
    if(lLong < 0)
    {
        lLong = -lLong;
        TxDData( USART_PC, '-');
    }
    lDigit = 1000000000L;
    for(bCount = 0; bCount < 9; bCount++)
    {
        lTmp = (u8)(lLong/lDigit);
        if(lTmp)
        {
            TxDData( USART_PC,((u8)lTmp)+'0');
            bPrinted = 1;
        }
        else if(bPrinted) TxDData( USART_PC,((u8)lTmp)+'0');
        lLong -= ((u32)lTmp)*lDigit;
        lDigit = lDigit/10;
    }
    lTmp = (u8)(lLong/lDigit);
    /*if(lTmp)*/ TxDData( USART_PC, ((u8)lTmp)+'0');
}


void TxD16DecDigit(u16 wData,u8 bDigit)
{
    u8 bCount, bPrinted;
    u16 wTmp,wDigit;
    bPrinted = 0;

    wDigit = 10000;
    for(bCount = 0; bCount < 5; bCount++)
    {
        wTmp = (wData/wDigit);
        if(bDigit > 4-bCount)
        {
            if(wTmp)
            {
                TxDData( USART_PC,((u8)wTmp)+'0');
                bPrinted = 1;
            }
            else
            {
                if(bPrinted) TxDData( USART_PC,((u8)wTmp)+'0');
                else TxDData( USART_PC,'0');
            }
        }
        wData -= wTmp*wDigit;
        wDigit /= 10;
    }
}


/* // GB_HEX_MODE ﾄﾁﾆｮｷﾑ ﾅﾗﾀﾌｺ�ﾃﾟｰ｡ ﾀﾌﾈﾄ, ｱｸﾇ�ｿ萇ﾔ.
void TxD8Mode(byte bData)
{
  if(GB_HEX_MODE) TxD8Hex(bData);
  else TxD16DecDigit((word)bData,3);
}
void TxD16Mode(word wData)
{
  if(GB_HEX_MODE) TxD16Hex(wData);
  else TxD16DecDigit(wData,4);
}
*/

/******************* (C) COPYRIGHT 2010 ROBOTIS *****END OF FILE****/
