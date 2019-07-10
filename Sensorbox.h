/*
;	 Project:       Smart EVSE Sensorbox 1.5
;    Date:          10 July 2019
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

#ifndef __EVSE_SENSORBOX
#define __EVSE_SENSORBOX

#include <xc.h>
#include <string.h>
#include <math.h>

#define _XTAL_FREQ  32000000

#define CAL 0.3
#define SAMPLES 2048                                                            // Number of times to sample CT1, CT2, and CT3

#define RS485_RECEIVE {LATAbits.LATA1 = 0;}
#define RS485_TRANSMIT {LATAbits.LATA1 = 1;}

#define LED_SetHigh { LATCbits.LATC0 = 1;}
#define LED_SetLow { LATCbits.LATC0 = 0;}


#endif	


