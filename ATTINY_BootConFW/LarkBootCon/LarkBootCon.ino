/*************************************************************************

  ██╗  ██╗ █████╗ ███╗   ███╗███████╗██╗     ██╗ ██████╗███████╗███████╗
  ██║  ██║██╔══██╗████╗ ████║██╔════╝██║     ██║██╔════╝██╔════╝██╔════╝
  ███████║███████║██╔████╔██║███████╗██║     ██║██║     █████╗  ███████╗
  ██╔══██║██╔══██║██║╚██╔╝██║╚════██║██║     ██║██║     ██╔══╝  ╚════██║
  ██║  ██║██║  ██║██║ ╚═╝ ██║███████║███████╗██║╚██████╗███████╗███████║
  ╚═╝  ╚═╝╚═╝  ╚═╝╚═╝     ╚═╝╚══════╝╚══════╝╚═╝ ╚═════╝╚══════╝╚══════╝
                                                          HamSlices 2025
**************************************************************************

  Lark Power Management and Boot Controller Firmware (ATTINY)   
    11% of program space used, 2% of dynamic memory
    (untested 8/16/2025)

  - Debounce is done in hardware
  
  --------------------------------------------------------------------------
 
  @attention
 
  Copyright (C) 2025 HamSlices
 
  This program is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation.
 
  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.
 
  You should have received a copy of the GNU General Public License
  along with this program.  If not, see <https://www.gnu.org/licenses/>.  

*************************************************************************/

#define PowerUpWait 300

/* Pin Definitions */
#define PowerEnable 0
#define nReset      1
#define Boot0       2
#define DebugLED    3
#define PushBTN     4

/* Global Vars */
int BootFlag   = 0;
int PowerState = 0;

void setup() {

  /* IO Setup */
  pinMode(PowerEnable, OUTPUT);  //Output
  pinMode(nReset, OUTPUT);       //Output
  pinMode(Boot0, OUTPUT);        //Output
  pinMode(DebugLED, OUTPUT);     //Output
  pinMode(PushBTN, INPUT);       //Input

  /* Set IO States */
  digitalWrite(PowerEnable, LOW);  //Power off
  digitalWrite(nReset, LOW);       //Place in Reset
  digitalWrite(Boot0, LOW);        //Boot From App
  digitalWrite(DebugLED, HIGH);    //Turn LED OFF
  delay(200);

  /*
  * If the push button is held durring power on
  * the ATTINY will set the STM32 to bootloader mode
  */

  /* Check Button Press */
  if (digitalRead(PushBTN) == 0) {
    BootFlag = 1;                  //set bootflag to 1
    digitalWrite(DebugLED, LOW);   //Turn LED ON
    while (digitalRead(PushBTN) == 0)
      ;                            //Hold for Release
    digitalWrite(DebugLED, HIGH);  //Turn LED OFF
  }
}

void loop() {

  /* Check BootFlag */
  if (BootFlag) {
    digitalWrite(Boot0, HIGH);        //Boot From Bootloader
    digitalWrite(PowerEnable, HIGH);  //Power on
    delay(PowerUpWait);               //Wait for power up
    digitalWrite(nReset, LOW);        //Remove from Reset
    BootFlag = 0;
    PowerState = 1;
  }

  /* Check Button Press */
  if (digitalRead(PushBTN) == 0) {
    digitalWrite(Boot0, LOW);           //Boot From App
    if (PowerState == 1) {              //Power State = ON
      digitalWrite(nReset, LOW);        //Place in Reset
      digitalWrite(PowerEnable, LOW);   //Power off
      PowerState = 0;                   //State to Off
    } else {                            //Power State = OFF
      digitalWrite(PowerEnable, HIGH);  //Power on
      delay(PowerUpWait);               //Wait for power up
      digitalWrite(nReset, LOW);        //Remove from Reset
      PowerState = 1;                   //State to On
    }
    digitalWrite(DebugLED, LOW);   //Turn LED ON
    while (digitalRead(PushBTN) == 0)
      ;                            //Hold for Release
    digitalWrite(DebugLED, HIGH);  //Turn LED OFF
  }
}


//Line 116