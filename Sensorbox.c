/*
;    Project:       Sensorbox 1.5, for use with Smart EVSE
;    Date:          10 July 2019
;
;    Changes:
;    1.0  Initial release
;
;    (C) 2013-2019  Michael Stegen / Stegen Electronics
;
;	 Current measurement calculations, from openenergymonitor.org
; 
;    set XC8 linker memory model settings to: double 32 bit, float 32 bit
;    extended instruction set is not used on XC8
;
;    XC8 compiler version 2.00 is used
;
; 
; Permission is hereby granted, free of charge, to any person obtaining a copy
; of this software and associated documentation files (the "Software"), to deal
; in the Software without restriction, including without limitation the rights
; to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
; copies of the Software, and to permit persons to whom the Software is
; furnished to do so, subject to the following conditions:
;
; The above copyright notice and this permission notice shall be included in
; all copies or substantial portions of the Software.
;
; THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
; IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
; FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
; AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
; LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
; OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
; THE SOFTWARE.
*/

#include <xc.h>
#include <string.h>
#include <math.h>
#include "Sensorbox.h"

// CONFIG1
#pragma config FOSC = INTOSC                                                    // Oscillator Selection Bits->INTOSC oscillator: I/O function on CLKIN pin
#pragma config WDTE = OFF                                                       // Watchdog Timer Enable->WDT disabled
#pragma config PWRTE = OFF                                                      // Power-up Timer Enable->PWRT disabled
#pragma config MCLRE = OFF                                                      // MCLR Pin Function Select->MCLR/VPP pin function is digital input
#pragma config CP = OFF                                                         // Flash Program Memory Code Protection->Program memory code protection is disabled
#pragma config BOREN = ON                                                       // Brown-out Reset Enable->Brown-out Reset enabled
#pragma config CLKOUTEN = OFF                                                   // Clock Out Enable->CLKOUT function is disabled. I/O or oscillator function on the CLKOUT pin
#pragma config IESO = ON                                                        // Internal/External Switchover Mode->Internal/External Switchover Mode is enabled
#pragma config FCMEN = ON                                                       // Fail-Safe Clock Monitor Enable->Fail-Safe Clock Monitor is enabled

// CONFIG2
#pragma config WRT = OFF                                                        // Flash Memory Self-Write Protection->Write protection off
#pragma config PPS1WAY = ON                                                     // Peripheral Pin Select one-way control->The PPSLOCK bit cannot be cleared once it is set by software
#pragma config ZCDDIS = ON                                                      // Zero-cross detect disable->Zero-cross detect circuit is disabled at POR
#pragma config PLLEN = ON                                                       // Phase Lock Loop enable->4x PLL is always enabled
#pragma config STVREN = ON                                                      // Stack Overflow/Underflow Reset Enable->Stack Overflow or Underflow will cause a Reset
#pragma config BORV = LO                                                        // Brown-out Reset Voltage Selection->Brown-out Reset Voltage (Vbor), low trip point selected.
#pragma config LPBOR = OFF                                                      // Low-Power Brown Out Reset->Low-Power BOR is disabled
#pragma config LVP = OFF                                                        // Low-Voltage Programming Enable->High-voltage on MCLR/VPP must be used for programming


// Global data
char TXbuffer[50];                                                              // RS485 Transmit buffer
char RXbuffer[50],RXpacket[50];                                                 // RS485 Receive and working buffer
char Tbuffer[50];                                                               // temp buffer
double Irms[3];
int lastSampleI, sampleI, tempI;                                                // sample holds the raw analog read value, lastSample holds the last sample
long filteredI, filtI_div4, tempL;
long sqI;
unsigned char RX1byte, Transmit=0, LegacyProtocol=1;
unsigned char idx = 0, ISRFLAG = 0, ISRTXFLAG = 0, ISRTXLEN = 0;
unsigned long Timer = 0;                                                        // mS counter
unsigned long SecTimer, ModbusTimer, LedTimer;


int sampleI_CT[3]={512,512,512};
long filteredI_CT[3]={0,0,0};


void __interrupt() ISR (void)
{
    while (PIR1bits.RCIF)                                                       // Uart1 receive interrupt? RS485
    {
        RX1byte = RCREG;                                                        // copy received byte

        if (Timer > (ModbusTimer + 3))                                          // last reception more then 3ms ago? 
        {
            idx = 0;                                                            // clear idx in RS485 RX handler
        }  
        if (idx == 50) idx--;                                                   // max 50 bytes in buffer
        RXbuffer[idx++] = RX1byte;                                              // Store received byte in buffer

        ModbusTimer = Timer;
    }

    if (PIR1bits.TXIF && PIE1bits.TXIE)                                         // Uart1 transmit interrupt? RS485
    {
        TXREG1 = TXbuffer[ISRTXFLAG++];                                         // send character
        if ((ISRTXFLAG == ISRTXLEN)|| ISRTXFLAG == 50)                          // end of buffer
        {
            PIE1bits.TXIE = 0;                                                  // clear transmit Interrupt for RS485 after sending last character
            ISRTXFLAG = 0;                                                      // end of transmission.
        }                                                                       // we switch off the transmitter in the ISR loop, after the final character has been sent..
    }
    
    // Timer 4 interrupt, called 1000 times/sec
    if (PIR2bits.TMR4IF)                                                     
    {
        Timer++;                                                                // mSec counter (overflows in 1193 hours)
        
        if (Timer > LedTimer+500) LED_SetLow;                                   // LED off after 0.5 second
                
        if (!ISRTXFLAG && TXSTAbits.TRMT && Transmit) {
            RS485_RECEIVE;                                                      // set RS485 transceiver to receive if the last character has been sent
            
            Transmit = 0;
            if (LegacyProtocol) {
                PIE1bits.RCIE = 0;                                              // Disable receive interrupt
                SP1BRGL = 0x40;                                                 // set baudrate to 9600 bps
                SP1BRGH = 0x03;                                                 // 
                PIE1bits.RCIE = 1;                                              // Enable Receive interrupt    
            }
        }    
        
        PIR2bits.TMR4IF = 0;                                                    // clear interrupt flag
    }
}

void initialize(void) {
    
 
    LATA = 0x00;                                                                // LATx registers
    LATC = 0x00;
    TRISA = 0x30;                                                               // Pins RA4-5 are inputs, RA0-1 are outputs 
    TRISC = 0x3A;                                                               // Pins RC1,RC3-5 are inputs, RC0,RC2 are outputs
    ANSELA = 0x17;                                                              // Ansel enabled for all digital outputs, saves a bit of power
    ANSELC = 0x3F;                                                              // 
    WPUA = 0x00;                                                                // Weak Pull up registers
    WPUC = 0x00;                                                                // No Pull ups
    OPTION_REGbits.nWPUEN = 1;
    ODCONA = 0x00;                                                              // Open Drain ODx registers
    ODCONC = 0x00;
    SLRCONA = 0x37;                                                             // SlewRate Control SLRCONx registers
    SLRCONC = 0x3F;
    RXPPS = 0x05;                                                               // RA5->EUSART:RX;    
    RA0PPS = 0x14;                                                              // RA0->EUSART:TX; 
    
    OSCCON = 0x70;                                                              // SCS FOSC; SPLLEN disabled; IRCF 8MHz_HF; 
    OSCSTAT = 0x00;                                                             // SOSCR disabled; 
    OSCTUNE = 0x00;                                                             // TUN 0; 
    BORCON = 0x00;                                                              // SBOREN disabled; BORFS disabled; 
    
    WDTCON = 0x16;                                                              // Interval 2s WDT disabled
                                                                                // setup DAC to half of VCC(3.3V) so 1.65V
    DAC1CON0 = 0x90;                                                            // DAC1EN enabled; DAC1NSS VSS; DAC1PSS VDD; DAC1OE1 disabled; DAC1OE2 enabled; 
    DAC1CON1 = 0x80;                                                            // DAC1R 128;
                                                                                // setup the ADC
    ADCON0 = 0x0D;                                                              // GO_nDONE stop; ADON enabled; CHS AN3; 
    ADCON1 = 0xA0;                                                              // ADFM right; ADPREF VDD; ADCS FOSC/32; 
    ADCON2 = 0x00;                                                              // TRIGSEL no_auto_trigger; 
    
    OPA1CON = 0xD2;                                                             // OPA1SP High_GBWP_mode; OPA1EN enabled; OPA1PCH DAC; OPA1UG OPA_Output; 
    
    // uart
    BAUD1CON = 0x08;                                                            // ABDOVF no_overflow; SCKP Non-Inverted; BRG16 16bit_generator; WUE disabled; ABDEN disabled; 
    RC1STA = 0x90;                                                              // SPEN enabled; RX9 8-bit; CREN enabled; ADDEN disabled; SREN disabled; 
    TX1STA = 0x24;                                                              // TX9 8-bit; TX9D 0; SENDB sync_break_complete; TXEN enabled; SYNC asynchronous; BRGH hi_speed; CSRC slave; 
    
    SP1BRGL = 0x40;                                                             // set baudrate to 9600 bps
    SP1BRGH = 0x03;                                                             // 
    //SP1BRGL = 0x0A;                                                             // set baudrate to 1200 bps
    //SP1BRGH = 0x1A;                                                             // 
    
    PR4 = 0x7C;                                                                 // Timer 4 frequency value -> 1Khz @ 32 Mhz
    T4CON = 0x07;                                                               // Timer 4 ON, prescaler 1:64
    
    PIE1bits.RCIE = 1;                                                          // enable receive interrupt
    PIE2bits.TMR4IE = 1;                                                        // enable timer 4 interrupt
    INTCONbits.PEIE = 1;                                                        // peripheral interrupts enabled
    INTCONbits.GIE = 1;                                                         // global interrupts enabled
}


unsigned int ReadAnalog(void)                                                   // Start ADC conversion, and return result
{
    ADCON0bits.ADGO = 1;                                                        // start next conversion on the selected channel
    while(ADCON0bits.ADGO);                                                     // wait for the adc conversion to finish
    return ADRES;                                                               // return result
}

// Poly used is x^16+x^15+x^2+x
// calculates 16-bit CRC of given data
// used for Frame Check Sequence on data frame
unsigned int crc16(unsigned char *buf, unsigned char len) {
    unsigned int crc = 0xffff;
    
    // Poly used is x^16+x^15+x^2+x
    for (int pos = 0; pos < len; pos++) {
        crc ^= (unsigned int)buf[pos];                                          // XOR byte into least sig. byte of crc

        for (int i = 8; i != 0; i--) {                                          // Loop over each bit
            if ((crc & 0x0001) != 0) {                                          // If the LSB is set
                crc >>= 1;                                                      // Shift right and XOR 0xA001
                crc ^= 0xA001;
            } else                                                              // Else LSB is not set
                crc >>= 1;                                                      // Just shift right
        }
    }        

    return crc;
}

unsigned int crc16sensorbox1(unsigned char *buf, unsigned char len) {
    unsigned int crc = 0xffff;
    
    // Poly used is x^16+x^12+x^5+x
    unsigned int c;
    int i;
    while (len--) {
        c = *buf;
        for (i = 0; i < 8; i++) {
            if ((crc ^ c) & 1) crc = (crc >> 1)^0x8408;
            else crc >>= 1;
            c >>= 1;
        }
        buf++;
    }
    crc = (unsigned int) (crc ^ 0xFFFF);

    return crc;
}




// Create HDLC/modbus frame from data, and copy to output buffer
// Start RS485 transmission, by enabling TX interrupt
void RS485SendBuf(char *buffer, unsigned char len, unsigned char protocol) {
    char ch, index = 0;
    
    while (ISRTXFLAG) {}                                                        // wait if we are already transmitting on the RS485 bus (blocking)
    
    LED_SetHigh;                                                                // LED on
    LedTimer = Timer;                                                           // Set LED timer
    
    if (protocol) {                                                             // Legacy protocol?
        
        PIE1bits.RCIE = 0;                                                      // Disable receive interrupt
        SP1BRGL = 0x0A;                                                         // set baudrate to 1200 bps
        SP1BRGH = 0x1A;                                                         // 
        PIE1bits.RCIE = 1;                                                      // Enable Receive interrupt      
        
        TXbuffer[index++] = 0x7E;                                               // start with sync flag in buffer

        while(len--) {
            ch = *buffer++;                                                     // load next byte

            if ((ch == 0x11) || (ch == 0x12) || (ch == 0x13) || (ch == 0x7E) || (ch == 0x7D)) {	// check for escape character
                ch = ch^0x20;
                TXbuffer[index++] = 0x7D;                                       // insert escape character
            }
            TXbuffer[index++] = ch;                                             // load data in buffer
        }
        TXbuffer[index++] = 0x7E;                                       		// end with sync flag in buffer
    } else {                                                                    // Modbus protocol
        
        PIE1bits.RCIE = 0;                                                      // Disable receive interrupt
        SP1BRGL = 0x40;                                                         // set baudrate to 9600 bps
        SP1BRGH = 0x03;                                                         // 
        PIE1bits.RCIE = 1;                                                      // Enable Receive interrupt 
        
        while (len--) {     
            TXbuffer[index++] = *buffer++;                                      // load next byte
        }
    }

    ISRTXLEN = index;                                                           // number of bytes to transfer
    RS485_TRANSMIT;                                         					// set RS485 transceiver to transmit, will be disabled in main loop

    NOP(); NOP(); NOP();                                                        // small delay
    
    Transmit = 1;
    PIE1bits.TXIE = 1;                                                          // enable transmit Interrupt for RS485
}




// Ch should be 0,1 or 2
double ReadCTnew(unsigned char Ch)
{
	unsigned int n;
    long sumI = 0;
    unsigned char input;
    
    // select the A/D channel
    if (Ch==0) input=3;
    else if (Ch==1) input=7;
    else input=5;
    
    ADCON0bits.CHS = input;                                                        
	    
	            
	sampleI=sampleI_CT[Ch];                                                     // Get Sample and Filter values
	filteredI=filteredI_CT[Ch];	
           
	for (n = 0; n < SAMPLES; n++)
  	{
        lastSampleI = sampleI;
    	sampleI = ReadAnalog();                                                 // Read analog input
              
        tempI = sampleI-lastSampleI;                                            // the most recent input change
        tempL = (long)tempI<<8;                                                 // re-scale the input change (x256)
        tempL+= filteredI;                                                      // combine with the previous filtered value
        filteredI = tempL-(tempL>>8);                                           // subtract 1/256, same as x255/256
 
        filtI_div4 = filteredI>>2;                                              // now x64
        // Root-mean-square method current
        // 1) square current values
        sqI = filtI_div4 * filtI_div4;
        sqI = sqI >>12;                                                         // scale back
        // 2) sum
        sumI += sqI;
    }
     
    sampleI_CT[Ch]=sampleI;
	filteredI_CT[Ch]=filteredI;                                                 // Store Sample and Filter values
    
    return sqrt((double)sumI/SAMPLES);                                          // Return squareroot of uncalibrated value

}



void main(void)
{
    char *pBytes;
	char x,n, Second=0, DataReady=0;
	unsigned int cs;

    
    initialize();
    
    SecTimer = Timer;                                                           // initialize one second Timer
    LegacyProtocol = 1;                                                         // Start with old protocol at 1200 bps
    
    while (1)
    {
    
                
        if (RC1STAbits.OERR)                                                    // Uart1 Overrun Error?
        {
            RC1STAbits.CREN = 0;
            RC1STAbits.CREN = 1;                                                // Restart Uart
            
            SecTimer = Timer+5000;                                               // Wait 5 seconds before sending anything
        }

        // Receive data from modbus
        // last reception more then 3ms ago?                                    // complete packet detected?
        if (idx>6 && Timer > (ModbusTimer + 3)) {
            // store received data packet
            memcpy(RXpacket, RXbuffer, idx);                                    // make local copy
            // set flag to length of data packet
            ISRFLAG = idx;
            idx = 0;                                                            // and make buffer available for new data
            
            cs = crc16(RXpacket, ISRFLAG);                                      // calculate checksum over all data (including crc16)
            if (RXpacket[0]==0x0a && RXpacket[1]==0x04 && RXpacket[5]==0x14 && !cs)   // check CRC
            {
                SecTimer = Timer;                                               // take new measurement after one second.
                
                LegacyProtocol = 0;                                             // Stay at 9600 bps, Sensorbox 2 modbus
                
                                                                                // Setup Modbus data
                Tbuffer[0]= 0x0a;                                               // Fixed Address 10 (0x0a) is Sensorbox
                Tbuffer[1]= 0x04;                                               // function byte
                Tbuffer[2]= 0x28;                                               // takes the bytes from the request. 28h bytes will follow
                Tbuffer[3]= 0x00;                                               // 
                Tbuffer[4]= 0x0F;                                               // Sensorbox version 1.5 = 0x0f
                Tbuffer[5]= 0x00;                                               // DSMR Version (unused)
                Tbuffer[6]= 0x03;                                               // 0x80 = P1, 0x03= 3CT's ,0x83 = P1+ 3CT

                n=7;
                for (x=0; x<(6*4) ;x++) {                                       // P1 data. Volts and Current set to 0
                    Tbuffer[n++] = 0;
                }
                for (x=0; x<3 ;x++) {
                    Irms[x] = Irms[x]* CAL;
                    pBytes = (char*)&Irms[x];                                   // get raw 4 byte Double 
                    Tbuffer[n++] = pBytes[3];                                   // Send MSB first
                    Tbuffer[n++] = pBytes[2];
                    Tbuffer[n++] = pBytes[1];
                    Tbuffer[n++] = pBytes[0];                                   // Send LSB last
                }
                cs = crc16(Tbuffer, n);                                         // calculate CRC16 from data			
                Tbuffer[n++] = ((unsigned char)(cs));
                Tbuffer[n++] = ((unsigned char)(cs>>8));	

                RS485SendBuf(Tbuffer, n, 0);                                    // send buffer to RS485 port
                DataReady = 0;
            }    
        }
        
        if (Timer > SecTimer+900 ) {                                            // Every 0.9 second this is executed
            
            SecTimer = Timer;
            
            if (!DataReady) {
                Irms[0] = ReadCTnew(0);                                         // Read the CT's
                Irms[1] = ReadCTnew(1);
                Irms[2] = ReadCTnew(2);
                DataReady = 1;
            }

            if (LegacyProtocol && ++Second >= 2) {                              // Every ~2 seconds Sensorbox 1 data is sent.
                
                Second = 0;
                // Sensorbox 1 code
                
                Tbuffer[0]= 0xff;                                               // Address Field = ff
                Tbuffer[1]= 0x03;                                               // Control Field = 03
                Tbuffer[2]= 0x50;                                               // Protocol = 0x5001
                Tbuffer[3]= 0x01;
                Tbuffer[4]= 0x01;
                Tbuffer[5]= 0x03;                                               // 3 CTs

                n=6;
                for (x=0; x<3; x++) {
		      	pBytes = (char*)&Irms[x];	
            	Tbuffer[n++] = pBytes[0];
                Tbuffer[n++] = pBytes[1];
                Tbuffer[n++] = pBytes[2];
                Tbuffer[n++] = pBytes[3];
                }
                                                        						// Frame Check Sequence (FCS) Field
                cs = crc16sensorbox1(Tbuffer, n);                            	// calculate CRC16 from data			
                Tbuffer[n++] = ((unsigned char)(cs));
                Tbuffer[n++] = ((unsigned char)(cs>>8));	

                RS485SendBuf(Tbuffer, n, 1);                                 	// send buffer to RS485 port
                DataReady = 0;
            }
        } 
        
    } // while(1)
}
/*
 End of File
*/