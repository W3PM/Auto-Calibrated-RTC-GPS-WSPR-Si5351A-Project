/*
  Si5351 bandswitched WSPR source designed for 2200 - 10 meters ustilizing
  a 1pps from an RTC module for timing and frequency adjustment.

  This sketch uses a frequency resolution down to 0.01 Hz to provide optimum
  reolution for the1.4846 Hz WSPR tone separation specification. This limits
  the upper frequency to 40 MHz.

  For additional time and frequency accuracy, the DS3231 RTC crystal oscillator
  aging offset may adjusted. Refer to calibration instructions at the end of
  this sketch.

  This sketch includes a function "Ant Tune" to be used as an indicator to help
  tune the connected antenna. Although this sketch is hardware compatible with
  the W3PM Multifunction Project found in the May/June QEX magazine, additional
  circuitry is necessary to use this function. Refer to the documentation for
  additonal information.


  Permission is granted to use, copy, modify, and distribute this software
  and documentation for non-commercial purposes.

  Copyright (C) 2019,  Gene Marcus W3PM GM4YRE

  14 July, 2019

  14 November, 2019  v2_0 This is v1_0 without GPS function
  16 November, 2019  This is w3pm_WSPR_v2_0 renamed w3pm_RTC_WSPR_v2_0 to
                     differentiate from the GPS version
  7 April, 2020      v2_1 Added AntennaTune algorithm from GPS WSPR sketch
  11 April, 2020     v2_2 Clean up pushbutton documentation and include date display
  16 April, 2020     v2_3 Improved callsign and gridsquare editing
  24 April, 2020     v3_0 Included GPS sychronization alogorithm using SoftwareSerial
  29 April, 2020     v3_1 GPS sync via 9600 baud serial input port
  4  May, 2020       v3_2 Added GPS sync write delay correction factor
  7  May, 2020       v3_3 Updated autocal algorithm
  20 July, 2020      v4_4 Revised Si5351 algorithm
  21 July, 2020      v4_4 Corrected Si5351 algorithm bug
  29 July, 2020      v4_5 Included GPS SYNC pushbutton comments
  12 August, 2020    v4-6 Corrected "GPS SYNC' bug
  11 October, 2020   v4_7 Corrected "Calibrate" bug


  The code was sucessfully compiled using Arduino 1.8.12

   NOTE! Uses library SSD1306Ascii by Bill Greiman
        Load from: Sketch > Include Library > Manage Libraries

   EEPROM address
         0-29 WSPR band (10) and mutifunction project data
         30-40 Callsign
         50-57 Grid square
         60    Power
         62    TX interval
         64    Reserved
         66    TX offset
         70    Callsign change flag
         72    Grid square change flag
         74    Power change flag
         76    TX interval change flag
         78    TX hop change flag
         80    TX offset change flag


  ---------------------------------------------------------------------------------------------------------------
  Five pushbuttons are used to control the functions.
  Pushbutton pin allocations follow:
  ---------------------------------------------------------------------------------------------------------------
          WSPR             CLOCK SET                DATE SET                         EDIT CALL
  PB1   N/A            Time sync / Set Hour*     Set Day*                     Select alphanumeric
  PB2   ON / OFF       Set Minute*               Set Month*                   Depress with PB3 to save data
  PB3   Band Select    N/A                       Set Year*                    Depress with PB2 to save data
  PB4   N/A            *Hold to change time      *Hold to change date         Select character position
                                                                             Note: end CALL with "#" before saving

            EDIT GRID                                EDIT POWER                     TX INTERVAL
  PB1   Select alphanumeric                     increase power level             increase TX interval
  PB2   Depress with PB3 to save data           Depress with PB3 to save data    Depress with PB3 to save data
  PB3   Depress with PB2 to save data           Depress with PB2 to save data    Depress with PB2 to save data
  PB4   Select character position                Decrease power level             Decrease TX interval
        Note: end GRID with "#" before saving


            SET OFFSET                             TX FREQ HOP                     ANT TUNE
  PB1   increase offset                        "YES"                              N/A
  PB2   Depress with PB3 to save data          Depress with PB3 to save data      N/A
  PB3   Depress with PB2 to save data          Depress with PB2 to save data      Band Select
  PB4   Decrease offset                        "NO"                               N/A


           GPS SYNC
  PB1   N/A
  PB2   Depress with PB3 to save data
  PB3   Depress with PB2 to save data 
  PB4   N/A


  MENU - exit function and return to menu when not transmitting


  ------------------------------------------------------------------------
  Nano Digital Pin Allocations follow:
  ------------------------------------------------------------------------
  D0/RX
  D1
  D2  RTC 1pps input
  D3  Sync top-of-the-even minute for basic operation w/o display
  D4
  D5  2.5 MHz input from Si5351 CLK0 pin
  D6  pushButton 1
  D7  pushButton 2
  D8  pushButton 3
  D9  pushButton 4
  D10 MENU
  D11
  D12
  D13

  A0/D14 XmitLED
  A1/D15 MinLED
  A2/D16 SecLED
  A3/D17
  A4/D18 Si5351 & OLED SDA
  A5/D19 Si5351 & OLED SCL

*/

/*******************************************************************************************

                                 OPTIONAL RTC CALIBRATION

 *******************************************************************************************


   While the default RTC is already very accurate its accuracy can be pushed even higher
   by adjusting its aging offset register.

   For normal operation calibration is not required. The default 2 parts per million accuracy
   of the RTC will result in an uncertainty of less than +/- 30 Hz on 20 meters.

   If WSPR is used, the time will require synchronization every 7 – 10 days without calibration.
   The re-synchronization timeframe can be stretched to a month or more with calibration.


   There are three options to perfom DS3231 RTC aging offset calibration:
   1) Measure and adjust the 32 kHz output
   2) Set VFO frequency to 10 MHz and use the frequency delta as the aging offset (not
      practical in this WSPR only version)
   3) Track the time over a period of days.

   A calibration function is provided view the current aging offset and enter a new offset.
   To enter the calibration funtion, hold down pushbutton 4 and invoke a reset. Invoke a
   reset to exit the calibration function.

   IMPORTANT NOTE: When using methods 2 and 3 above, any change will not take place until
                   the auto-calibration algorithm calculates the correction factor for the
                   Si5351A’s 25 MHz clock and the DS3231's temperature compensation algorithm
                   performs its calculation. This may take a minute or two before any
                   change appears. Additionally, the auto-calibration routine uses an
                   averaging alogithm over a few minutes to reduce the effects of 1pps gate
                   time jitter. Any adjustment (other than the 32 KHz method) will be an
                   iterative process and will take some time.


   32 kHz & FREQUENCY COUNTER METHOD:
   Attach a 10 kohm pull-up resistor from the VCC to the 32K pins of the RTC. Attach a high
   resolution accurate counter from the 32K pin to ground.
   Enter the calibration function (see above) and use pushbuttons 1 and 2 to adjust the
   frequency as closely to 32768 Hz as possible.


   VFO & FREQUENCY COUNTER METHOD:
   Set the VFO to 10 Mhz and measure the frequency from CLK1 with an accurate frequency
   counter. Measure to the nearest Hz and subract this number from 10 MHz. This will be
   the aging offset. Enter the calibration function (see above). If this is the first
   time you performed a calibration you should see "CF = 0" on the display. If not, add
   the measured aging factor to the displayed number.  Use pushbuttons 1 and 2 to set the
   device to the aging factor. Invoke a reset to exit.

   Note: Per the DS3231 datasheet at  +25°C,  one LSB (in the offset register) typically
         provides about 0.1ppm change in frequency. At 10MHx 0.1ppm equates to 1 Hz.
         Theoretically, the process described above should produce get you right on
         target. Practically, I found that I had to go back and forth to obtain the
         greatest accuracy. Allow the system a few minutes to stabilize before making
         any adjustments (refer to the note above).


   TIME TRACKING METHOD:
   Sychronize the device's displayed time with an accuate time source such as WWV.
   After a few days compare the displayed time the reference time source. The
   following formula should bring you close to nominal:

     aging offset = (change in seconds / (86700 x number of days)) x 10^7

   Use a positive integer if the clock is running fast and a negative integer if it is
   running slow.

   Enter the calibration function (see above). If this is the first time you
   performed a calibration you should see "CF = 0" on the display. If not, add the
   measured aging offset to the displayed number.  Use pushbuttons 1 and 2 to set the
   device to the aging factor. Invoke a reset to exit.

 ********************************************************************************************
*/

//----------------------------------------------------------------------------------------------------
//                         WSPR configuration data follows:
//----------------------------------------------------------------------------------------------------
// Enter your callsign below:
// Note: Upper or lower case characters are acceptable. Compound callsigns e.g. W3PM/4 W4/GM4YRE
//       are supported.
char call2[12] = "A9AA";
char locator[8] = "AA99";

// Enter TX power in dBm below: (e.g. 0 = 1 mW, 10 = 10 mW, 20 = 100 mw, 30 = 1 W)
// Min = 0 dBm, Max = 43 dBm, steps 0,3,7,10,13,17,20,23,27,30,33,37,40,43
int  ndbm = 7;

// Enter desired transmit interval below:
// Avoid entering "1" as the unit will transmit every 2 minutes not allowing for
// autocal updates between transmissions
int TXinterval = 3; // Transmit interval e.g. 3 = transmit once every 3rd 2 minute transmit slot

// Enter desired transmit offset below:
// Transmit offset frequency in Hz. Range = 1400-1600 Hz (used to determine TX frequency within WSPR window)
unsigned long TXoffset = 1500UL;

// Enter In-band frequency hopping option:
// This option ignores TXoffset above and frequency hops within the WSPR 200 Hz wide window
// Before using this option be sure the system is calibrated to avoid going outside band edges.
// In-band transmit frequency hopping? (true = Yes, false = No)
bool FreqHopTX = false;

// Do not change AutoCalibrate - for development use only
//_________________________Auto calibrate using DS3231? (false = NO, true = YES)______________________
bool AutoCalibrate = true;

// Do not change basicOperation - for development use only
//_________________________Basic operation without LCD/OLED? (false = NO, true = YES)______________________
bool basicOperation = false;

//___________________________Enter calibration factor:__________________________________________
//    This entry is normally not required if the DS3231 RTC us used, but may be used to
//    enhance DS3231 system accuracy.
//
//  - Connect VFO to a frequency counter
//  - Set CalFactor variable to zero
//  - Set VFO to 25 MHz
//  - Annotate counter frequency in Hz
//  - Subtract 25 MHz from counter reading
//  - Enter the difference in Hz (i.e. -245)
int CalFactor = 0;

const unsigned long RXdialFreq [] =
{
  136000      ,
  474200      ,
  1836600      ,
  3592600      ,
  5287200      ,
  7038600      ,
  10138700      ,
  14095600      ,
  18104600      ,
  21094600      ,
  24924600      ,
  28124600      ,
  50293000      ,
  144489000      ,

  0
};

// include the library code:
#include <Wire.h>
#include "SSD1306Ascii.h"
#include "SSD1306AsciiAvrI2c.h"
#include <EEPROM.h>

// Set up MCU pins
#define InterruptPin             2
#define InterruptPin3            3
#define Synchronize              6
#define CFup                     7
#define CFdown                   6
#define calibrate                9
#define endCalibrate             8
#define Power_dBm                3
#define NanoLED                 13
#define XmitLED                 14
#define MinLED                  15
#define SecLED                  16
#define pushButton1              6
#define pushButton2              7
#define pushButton3              8
#define pushButton4              9
#define menu                    10

// Set DS3231 I2C address
#define DS3231_addr           0x68

// Set sI5351A I2C address
#define Si5351A_addr          0x60

// Define OLED address
#define I2C_ADDRESS           0x3C

// initialize oled display
SSD1306AsciiAvrI2c oled;

// Define Si5351A register addresses
#define CLK_ENABLE_CONTROL       3
#define CLK0_CONTROL            16
#define CLK1_CONTROL            17
#define CLK2_CONTROL            18
#define SYNTH_PLL_A             26
#define SYNTH_PLL_B             34
#define SYNTH_MS_0              42
#define SYNTH_MS_1              50
#define SYNTH_MS_2              58
#define PLL_RESET              177
#define XTAL_LOAD_CAP          183

typedef struct {  // Used in conjunction with PROGMEM to reduce RAM usage
  char description [4];
} descriptionType;

// configure variables
bool data_changeFlag = false, clockTimeChange = false, WSPRflag = false, suspendUpdateFlag = false, scrollFlag = false, MenuExitFlag = true;
bool toggle = false, WSPR_toggle = false, timeSet_toggle = false, StartCalc = true, startFlag = false;
byte sym[170], c[11], symt[170], calltype, msg_type, tcount3;
volatile byte ii;
int nadd, nc, n, ntype, TXcount, WSPRband;
int p, m, dBm, AutoCalFactor[50], CFcount, tcount2, SWRval, column;
int hours, minutes, seconds, xday, xdate, xmonth, xyear, startCount = 0;
int character, i;
char call1[7], grid4[5];
long MASK15 = 32767, ihash;
unsigned long XtalFreq = 25000000, tcount = 2, time_now = 0, LEDtimer;
unsigned long t1, ng, n2, cc1, n1,  mult = 0;
const descriptionType daysOfTheWeek [7] PROGMEM = { {"SUN"}, {"MON"}, {"TUE"}, {"WED"}, {"THU"}, {"FRI"}, {"SAT"},};
const descriptionType monthOfTheYear [12] PROGMEM = { {"JAN"}, {"FEB"}, {"MAR"}, {"APR"}, {"MAY"}, {"JUN"}, {"JUL"}, {"AUG"}, {"SEP"}, {"OCT"}, {"NOV"}, {"DEC"},};
const byte daysInMonth [] PROGMEM = { 31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31 };
byte SecondGPS, MinuteGPS, HourGPS;
char tempGPS[7] = "AA99AA";
byte Buffer[10], bufCounter, tempBuf;

// Load WSPR symbol frequency offsets
int OffsetFreq[4] = {
  -219, // 0 Hz
  -73,  // 1.46 Hz
  73,   // 2.93 Hz
  219   // 4.39 Hz
};

const char SyncVec[162] = {
  1, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 1, 1, 1, 0, 0, 0, 1, 0, 0, 1, 0, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 1, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0,
  1, 1, 0, 0, 1, 1, 0, 1, 0, 0, 0, 1, 1, 0, 1, 0, 0, 0, 0, 1, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 0, 1, 0, 0, 1, 0, 1, 1, 0, 0, 0, 1, 1, 0, 1, 0, 1, 0,
  0, 0, 1, 0, 0, 0, 0, 0, 1, 0, 0, 1, 0, 0, 1, 1, 1, 0, 1, 1, 0, 0, 1, 1, 0, 1, 0, 0, 0, 1, 1, 1, 0, 0, 0, 0, 0, 1, 0, 1, 0, 0, 1, 1, 0, 0, 0, 0,
  0, 0, 0, 1, 1, 0, 1, 0, 1, 1, 0, 0, 0, 1, 1, 0, 0, 0
};


//***********************************************************************
// This interrupt is used for Si5351 25MHz crystal frequency calibration
// Called every second by from the DS3231 RTC 1PPS to Nano pin D2
//***********************************************************************
void Interrupt()
{
  tcount++;
  tcount2++;
  if (tcount == 4)                                // Start counting the 2.5 MHz signal from Si5351A CLK0
  {
    TCCR1B = 7;                                   //Clock on falling edge of pin 5
  }
  else if (tcount == 44)                         //Total count is = XtalFreq x 4 - stop counting
  {
    TCCR1B = 0;                                   //Turn off counter
    unsigned long TempFreq = (mult * 0x10000 + TCNT1);     //Calculate corrected XtalFreq
    TCNT1 = 0;                                    //Reset count to zero
    mult = 0;
    tcount = 0;                                   //Reset the seconds counter

    // The following is an averageing alorithm used to smooth out DS3231 1pps jitter
    // Note: The upper and lower frequecny constraint prevents autocalibration data corruption
    //       which may occur if the COUNTER switch is in the wrong position.
    if (TempFreq > 99988000L & TempFreq < 100012000L) // Check bounds
    {
      int N = 12;
      if (CFcount == N)
      {
        CFcount = 0;
        StartCalc = true;
      }
      if (StartCalc == false)                       // This is the initial warm-up period
      {
        AutoCalFactor[CFcount] = 100000000UL - TempFreq;
        if (suspendUpdateFlag == true) CFcount++;   // Don't update corrected crystal frequency while transmitting

        else
        {
          XtalFreq = TempFreq / 4UL;                // Update corrected crystal frequency when not transmitting
          CFcount++;
        }
      }
      else                                          // Warm-up period completed, go into averaging mode
      {
        long temp = 0;
        AutoCalFactor[CFcount] = 100000000UL - TempFreq;
        for (int i = 0; i < N ; i++) temp = temp + AutoCalFactor[i];
        if (suspendUpdateFlag == false) XtalFreq = (100000000UL - (round(temp / N))) / 4UL; //Average the correction factors and update crystal frequency
        CFcount++;
      }
    }
  }
  digitalWrite(SecLED, LOW);
  digitalWrite(NanoLED, LOW);
  LEDtimer = millis() + 500;
  if (basicOperation == true & tcount2 == 120)
  {
    tcount2 = 0;
  }
}


//******************************************************************
// Basic operation - interrupt routine used to set timing to zero
// at the top of a two miniute WSPR slot.  Input on MCU pin 3
//******************************************************************
void Interrupt2()
{
  tcount2 = 0;
}


//******************************************************************
// Timer 1 overflow intrrupt vector.
// Called upon counter overflow from timer 1
//******************************************************************
ISR(TIMER1_OVF_vect)
{
  mult++;                                          //Increment multiplier
  TIFR1 = (1 << TOV1);                             //Clear overlow flag
}


//----------------------------------------------------------------------------------------------------
//                         Initial sketch setup follows:
//----------------------------------------------------------------------------------------------------
void setup()
{
  Wire.begin();                                 // join I2C bus

  // Initialize the Si5351
  Si5351_write(XTAL_LOAD_CAP, 0b11000000);      // Set crystal load to 10pF
  Si5351_write(CLK_ENABLE_CONTROL, 0b00000000); // Enable CLK0, CLK1 and CLK2
  Si5351_write(CLK0_CONTROL, 0b01001111);       // Set PLLA to CLK0, 8 mA output, INT mode
  Si5351_write(CLK1_CONTROL, 0b01101111);       // Set PLLB to CLK1, 8 mA output, INT mode
  Si5351_write(CLK2_CONTROL, 0b01101111);       // Set PLLB to CLK2, 8 mA output, INT mode
  Si5351_write(PLL_RESET, 0b10100000);          // Reset PLLA and PLLB


  // Set up DS3231 for 1 Hz squarewave output
  // Needs only be written one time provided DS3231 battery is not removed
  Wire.beginTransmission(DS3231_addr);
  Wire.write(0x0E);
  Wire.write(0);
  Wire.endTransmission();

  //Set up Timer1 as a frequency counter - input at pin 5
  TCCR1B = 0;                                    //Disable Timer5 during setup
  TCCR1A = 0;                                    //Reset
  TCNT1  = 0;                                    //Reset counter to zero
  TIFR1  = 1;                                    //Reset overflow
  TIMSK1 = 1;                                    //Turn on overflow flag

  // Inititalize 1 Hz input pin
  pinMode(InterruptPin, INPUT);
  digitalWrite(InterruptPin, HIGH);              // internal pull-up enabled

  // Set pin 2 for external 1 Hz interrupt input
  attachInterrupt(digitalPinToInterrupt(InterruptPin), Interrupt, FALLING);

  if (basicOperation == true)
  {
    pinMode(InterruptPin3, INPUT);
    digitalWrite(InterruptPin3, HIGH);           // internal pull-up enabled
    attachInterrupt(digitalPinToInterrupt(InterruptPin3), Interrupt2, FALLING);
    WSPRflag = true;
  }

  // Add CalFactor to the Si5351 crystal frequency for non-autocal frequency updates
  if (AutoCalibrate == false)
  {
    XtalFreq += CalFactor;  // Use corection factor if 1pps not used.
    detachInterrupt(digitalPinToInterrupt(InterruptPin)); // Disable the 1pps interrupt
  }

  // Allow time for autocalibration and Si5351 warm up for upon initial start
  // The first WSPR transmission will be delayed by one transmit interval
  TXcount = TXinterval + 1;

  // Set up push buttons
  pinMode(Synchronize, INPUT);
  digitalWrite(Synchronize, HIGH);      // internal pull-up enabled
  pinMode(Power_dBm, INPUT);
  digitalWrite(Power_dBm, HIGH);        // internal pull-up enabled
  pinMode(pushButton2, INPUT);
  digitalWrite(pushButton2, HIGH);      // internal pull-up enabled
  pinMode(pushButton3, INPUT);
  digitalWrite(pushButton3, HIGH);      // internal pull-up enabled
  pinMode(pushButton4, INPUT);
  digitalWrite(pushButton4, HIGH);      // internal pull-up enabled
  pinMode(menu, INPUT);
  digitalWrite(menu, HIGH);             // internal pull-up enabled

  // Set up LEDs
  pinMode(NanoLED, INPUT);
  digitalWrite(NanoLED, HIGH);           // internal pull-up enabled
  pinMode(XmitLED, OUTPUT);              // Use with dropping resistor on pin D14
  digitalWrite(XmitLED, LOW);
  pinMode(SecLED, OUTPUT);               // Use with dropping resistor on pin D16
  digitalWrite(SecLED, LOW);
  pinMode(MinLED, OUTPUT);               // Use with dropping resistor on pin D15
  digitalWrite(MinLED, LOW);

  TCCR1B = 0;                             //Disable Timer1

  // Enable the Si5351
  Si5351_write(CLK_ENABLE_CONTROL, 0b00000010); // Enable CLK0 and CLK2 - disable CLK1

  // Set CLK0 to 2.5 MHz for autocalibration
  si5351aSetFreq(SYNTH_MS_0, 2500000UL, 0);
  
  // Set CLK2 to 2.5 MHz for frequency stabilization
  si5351aSetFreq(SYNTH_MS_2, 2500000UL, 0);

  // Ensure input data is upper case
  for (int i = 0; i < 11; i++)if (call2[i] >= 97 && call2[i] <= 122)call2[i] = call2[i] - 32;
  for (int i = 0; i < 7; i++)if (locator[i] >= 97 && locator[i] <= 122)locator[i] = locator[i] - 32;

  // Get stored WSPR band
  WSPRband = EEPROM.read(10);
  if (WSPRband > 12 | WSPRband < 0) WSPRband = 1; //Ensures valid EEPROM data - if not valid will default to 630m

  int actionFlag; // Used to determine if new data is stored in EEPROM
  // If true replace variable with EEPROM cata
  // If false use compiled variable data

  // Get stored callsign
  EEPROM.get(70, actionFlag);
  if (actionFlag == 1) EEPROM.get(30, call2);

  // Get stored gridsquare
  EEPROM.get(72, actionFlag);
  if (actionFlag == 1) EEPROM.get(50, locator);

  // Get stored power level
  EEPROM.get(74, actionFlag);
  if (actionFlag == 1) EEPROM.get(60, ndbm);

  // Get stored transmit interval
  EEPROM.get(76, actionFlag);
  if (actionFlag == 1) EEPROM.get(62, TXinterval);

  // Get stored TX hopping variable
  EEPROM.get(78, FreqHopTX);

  // Get stored TX offset variable
  EEPROM.get(80, actionFlag);
  if (actionFlag == 1) EEPROM.get(66, TXoffset);

  // Set oled font size and type
  oled.begin(&Adafruit128x64, I2C_ADDRESS);
  oled.setFont(fixed_bold10x15);

  // Setup OLED initial display
  oled.clear();
  oled.setCursor(0, 4);
  oled.println(F("PB3: scroll"));
  oled.print(F("PB2: select"));

  // Generate unique WSPR message for your callsign, locator, and power
  wsprGenCode(); // WSPR message calculation

  // Store time reference for sketch timing i.e. delays, EEPROM writes, display toggles
  // The Arduino "delay" function is not used because of critical timing conflicts
  time_now = millis();

  // Check for calibration function
  if (digitalRead(pushButton4) == LOW) Calibrate(); //Used to enter calibration function upon reset


}



//******************************************************************
// Loop starts here:
// Loops consecutively to check MCU pins for activity
//******************************************************************
void loop()
{
  // Loop through the  menu rota until a selection is made
  while (digitalRead(pushButton2) == HIGH & startFlag == false)
  {
    if (digitalRead(pushButton3) == LOW)
    {
      altDelay(500);
      startCount = startCount + 1;;
      if (startCount > 10) startCount = 0;
    }

    switch (startCount) {
      case 0:
        oled.setCursor(0, 0);
        oled.println(F("   WSPR   "));
        break;
      case 1:
        oled.setCursor(0, 0);
        oled.println(F(" SET TIME"));
        break;
      case 2:
        oled.setCursor(0, 0);
        oled.println(F(" SET DATE "));
        break;
      case 3:
        oled.setCursor(0, 0);
        oled.println(F(" EDIT CALL"));
        break;
      case 4:
        oled.setCursor(0, 0);
        oled.println(F(" EDIT GRID"));
        break;
      case 5:
        oled.setCursor(0, 0);
        oled.println(F("EDIT POWER"));
        break;
      case 6:
        oled.setCursor(0, 0);
        oled.println(F("TX INTERVAL"));
        break;
      case 7:
        oled.setCursor(0, 0);
        oled.println(F(" SET OFFSET"));
        break;
      case 8:
        oled.setCursor(0, 0);
        oled.println(F("TX FREQ HOP"));
        break;
      case 9:
        oled.setCursor(0, 0);
        oled.println(F(" ANT TUNE  "));
        break;
      case 10:
        oled.setCursor(0, 0);
        oled.println(F(" GPS SYNC "));
        break;
    }
  }

  if (startFlag == false)
  {
    oled.clear();
    //oled.set1X();
    startFlag = true;
  }
  switch (startCount) {
    case 0: // Begin WSPR processing
      WSPR();
      break;
    case 1: // RTC clock adjustment begins here:
      adjClock();
      break;
    case 2: // RTC date adjustment begins here:
      setDATE();
      break;
    case 3: // Enter new callsign begins here:
      // column = 0;
      //  character = 65;
      EditCall();
      break;
    case 4: // Enter new gridsquare begins here:
      EditGrid();
      break;
    case 5: // Enter power level adjustment begins here:
      EditPower();
      break;
    case 6: // Enter transmit interval begins here:
      EditTXinterval();
      break;
    case 7: // Transmit offset frequency adjustment begins here:
      EditTXoffset();
      break;
    case 8: // Transmit TX frequency hopping selection begins here:
      EditTXhop();
      break;
    case 9: // Antenna tune algorithm begins here:
      AntennaTune();
      break;
    case 10: // GPS synchronization algorithm begins here:
      SyncGPS();
      break;

      // Selection is made at this point, now go to the selected function
  }

  if (digitalRead (menu) == LOW)
  {
    Serial.end();
    startFlag = false;
    Si5351_write(CLK_ENABLE_CONTROL, 0b00000010); // Enable CLK0 CLK2 - disable CLK1
    si5351aSetFreq(SYNTH_MS_2, 2500000UL, 0);        // CLK2 is enabled to balance thermal drift between transmissions
    suspendUpdateFlag = false;
    data_changeFlag = false;
    column = 0;
    character = 0;
    digitalWrite(XmitLED, LOW);
    oled.clear();
    oled.setCursor(0, 4);
    oled.println(F("PB3: scroll"));
    oled.print(F("PB2: select"));
    // Get stored callsign
    int actionFlag;
    EEPROM.get(70, actionFlag);
    if (actionFlag == 1) EEPROM.get(30, call2);

    // Get stored gridsquare
    EEPROM.get(72, actionFlag);
    if (actionFlag == 1) EEPROM.get(50, locator);

    // Get stored power level
    EEPROM.get(74, actionFlag);
    if (actionFlag == 1) EEPROM.get(60, ndbm);

    // Get stored transmit interval
    EEPROM.get(76, actionFlag);
    if (actionFlag == 1) EEPROM.get(62, TXinterval);

    // Get stored TX hopping variable
    EEPROM.get(78, FreqHopTX);

    // Get stored TX offset variable
    EEPROM.get(80, actionFlag);
    if (actionFlag == 1) EEPROM.get(66, TXoffset);

  }

  // LED operation
  if (LEDtimer < millis())
  {
    digitalWrite(SecLED, HIGH);
    digitalWrite(NanoLED, HIGH);
  }
  if (seconds == 0  & basicOperation == false)digitalWrite(MinLED, HIGH);
  if (seconds != 0  & basicOperation == false)digitalWrite(MinLED, LOW);
  if ((tcount2 == 0 | tcount2 == 60) & basicOperation == true)digitalWrite(MinLED, HIGH);
  if ((tcount2 == 1 | tcount2 == 61) & basicOperation == true)digitalWrite(MinLED, LOW);
}




//******************************************************************
// WSPR function follows:
// Used to select band, determine if it is time to transmit, and
// blink WSPR related LEDs.
// Called called by "switch case0" in loop
//******************************************************************
void WSPR()
{
  // Generate unique WSPR message for your callsign, locator, and power
  wsprGenCode(); // WSPR message calculation

  // Print callsign, frequency, and status to the display
  oled.setCursor(0, 0);
  oled.println(call2);
  setfreq();
  WSPRstatus();
  getTime();

  // Update for new day
  if (hours == 0 & minutes == 0 & seconds == 0) WSPRstatus();

  // Determine if it is time to transmit
  if (bitRead(minutes, 0) == 0 & seconds == 0 & WSPRflag == true) transmit();

  //Turn WSPR tranmit inhibit on and off during non-transmit period
  if (digitalRead(pushButton2) == LOW & suspendUpdateFlag == false)
  {
    altDelay(500);
    WSPR_toggle = !WSPR_toggle;
    if (WSPR_toggle == false)
    {
      WSPRflag = false;
      WSPRstatus();
    }
    else
    {
      WSPRflag = true;
      WSPRstatus();
    }
    altDelay(1000);
  }

  // Band selection during non-transmit period
  if (digitalRead(pushButton3) == LOW & suspendUpdateFlag == false)
  {
    altDelay(500);
    WSPRband++;
    if (RXdialFreq [WSPRband] == 0)WSPRband = 0;
    EEPROM.write(10, WSPRband);
    setfreq();
    altDelay(500);
    WSPRstatus();
  }

  // Update time during non-transmit period
  if (suspendUpdateFlag == false) displayClock();
}




//******************************************************************
// Clock adjust function follows:
// Used to adjust the system time
// Note: Quickly depressing pushbutton 1 will synchronize the clock
//       to the top of the minute "0"
//       Holding down pushbutton 4 while depressing pushbutton 1
//       will advance the hours.
//       Holding down pushbutton 3 while depressing pushbutton 2
//       will advance the minutes.
// Called called by "switch case4" in loop
//******************************************************************
void adjClock()
{
  getTime();

  // Display current time
  int tempHour = hours;
  int tempMinute = minutes;
  int tempSecond = seconds;
  oled.setCursor(0, 0);
  oled.print(F("  "));
  int a = hours;
  int b = a % 10;
  a = a / 10;
  oled.print(a);
  oled.print(b);
  oled.print(F(":"));
  a = minutes;
  b = a % 10;
  a = a / 10;
  oled.print(a);
  oled.print(b);
  oled.print(F(":"));
  a = seconds;
  b = a % 10;
  a = a / 10;
  oled.print(a);
  oled.print(b);

  // Display legend
  oled.setCursor(0, 2);
  oled.print(F("Hold PB4"));
  oled.setCursor(0, 4);
  oled.print(F("PB1: Hours"));
  oled.setCursor(0, 6);
  oled.print(F("PB2: Minutes"));

  // Start one button time synchronization routine
  if (digitalRead(pushButton1) == LOW)
  {
    if (tempSecond > 30) tempMinute++;
    if (tempMinute > 59) tempMinute = 0;
    tempSecond = 0;
    updateTime(tempSecond, tempMinute, tempHour);
    altDelay(500);
  }
  timeSet_toggle = false;

  // Start set time routine
  while (digitalRead(pushButton4) == LOW)
  {
    if (digitalRead(pushButton1) == LOW)
    {
      altDelay(500);
      tempHour++;
      if (tempHour > 23) tempHour = 0;
      timeSet_toggle = true;
    }

    if (digitalRead(pushButton2) == LOW)
    {
      altDelay(500);
      tempMinute++;
      if (tempMinute > 59) tempMinute = 0;
      timeSet_toggle = true;
    }

    // Display set time
    oled.setCursor(0, 0);
    oled.print(F("  "));
    int a = tempHour;
    int b = a % 10;
    a = a / 10;
    oled.print(a);
    oled.print(b);
    oled.print(F(":"));
    a = tempMinute;
    b = a % 10;
    a = a / 10;
    oled.print(a);
    oled.print(b);
    oled.print(F(":00"));
  }

  // Update time if change is made
  if (timeSet_toggle == true)
  {
    int tempSecond = 0;
    updateTime(tempSecond, tempMinute, tempHour);
    timeSet_toggle = false;
  }
}




//******************************************************************
// Date adjust function follows:
// Used to adjust the system date
// Note:
//       Holding down pushbutton 4 while depressing pushbutton 1
//       will advance the date.
//       Holding down pushbutton 4 while depressing pushbutton 2
//       will advance the month.
//       Holding down pushbutton 4 while depressing pushbutton 3
//       will advance the year.
// Called called by "switch case5" in loop
//******************************************************************
void setDATE()
{
  getDate();

  int updateDate = xdate;
  int updateMonth = xmonth;
  int updateYear = xyear;

  // Display currently stored date
  oled.setCursor(0, 0);
  oled.print(updateDate);
  oled.print(F(" "));
  descriptionType oneItem;
  memcpy_P (&oneItem, &monthOfTheYear [updateMonth - 1], sizeof oneItem);
  oled.print (oneItem.description);
  oled.print(F(" "));
  oled.print(updateYear);

  // Display legend
  oled.setCursor(0, 2);
  oled.print(F("Hold PB4"));
  oled.setCursor(0, 4);
  oled.print(F("1:Date"));
  oled.setCursor(0, 6);
  oled.print(F("2:Month 3:Yr"));

  // Start update
  while (digitalRead(pushButton4) == LOW)
  {
    if (digitalRead(pushButton1) == LOW)
    {
      altDelay(500);
      updateDate++;
      if (updateDate > 31) updateDate = 0;
      timeSet_toggle = true;
    }

    if (digitalRead(pushButton2) == LOW)
    {
      altDelay(500);
      updateMonth++;
      if (updateMonth > 12) updateMonth = 1;
      timeSet_toggle = true;
    }

    if (digitalRead(pushButton3) == LOW)
    {
      altDelay(500);
      updateYear++;
      if (updateYear > 30) updateYear = 19;
      timeSet_toggle = true;
    }

    // Display updates
    oled.setCursor(0, 0);
    //if (xdate < 10) oled.print(F("0"));
    oled.print(updateDate);
    oled.print(F(" "));
    descriptionType oneItem;
    memcpy_P (&oneItem, &monthOfTheYear [updateMonth - 1], sizeof oneItem);
    oled.print (oneItem.description);
    oled.print(F(" "));
    oled.print(updateYear);
  }

  // Save data if updated
  if (timeSet_toggle == true)
  {
    // Convert DEC to BCD
    updateDate = ((updateDate / 10) * 16) + (updateDate % 10);
    updateMonth = ((updateMonth  / 10) * 16) + (updateMonth % 10);
    updateYear = ((updateYear / 10) * 16) + (updateYear % 10);

    // Write the data
    Wire.beginTransmission(DS3231_addr);
    Wire.write((byte)0x04); // start at register 4
    Wire.write(updateDate);
    Wire.write(updateMonth);
    Wire.write(updateYear);
    Wire.endTransmission();
  }
}




//******************************************************************
//  Edit callsign follows:
//  Used to cchange callsign
//
//  Called by loop()
//******************************************************************
void EditCall()
{
  if (data_changeFlag == false)
  {
    SelectionHeader();

    char temp = 35;   // Set to "#" to indicate end of text
    oled.setCursor((column * 11), 0);
    oled.print("_");
    oled.setCursor(0, 2);
    oled.print(call2);

    if (digitalRead(pushButton1) == LOW)
    {
      character++;
      altDelay(300);
      if ((character > 0) & (character < 47)) character = 47;
      if ((character > 57) & (character < 65)) character = 65;
      call2[column] = character;

      if (character > 90)
      {
        call2[column] = 35;
        character = 0;
      }

    }
    if (digitalRead(pushButton4) == LOW)
    {
      altDelay(300);
      oled.setCursor((column * 11), 0);
      oled.print(" ");
      column++;
      character = call2[column];
      if (column > 11) column = 0;
    }
    if ((digitalRead(pushButton2) == LOW) & (digitalRead(pushButton3) == LOW))
    {
      for (int i = 0; i < 12; i++)
      {
        if (call2[i] == temp | call2[i] == "") data_changeFlag = true;
        if (data_changeFlag == true) call2[i] = 0;
      }
      if (data_changeFlag == true)
      {
        oled.setCursor(0, 0);
        oled.print(call2);
        oled.setCursor(0, 2);
        oled.print(F("  SAVED  "));
        EEPROM.put(30, call2); // Save callsign
        EEPROM.put(70, 1);     // Now set callsign change flag
        altDelay(4000);
      }
      else
      {
        oled.setCursor(0, 0);
        oled.print(F("NOT SAVED "));
        oled.setCursor(0, 2);
        oled.print(F("END WITH #"));
        //oled.print(temp);
        altDelay(4000);
      }
      column = 0;
      oled.setCursor(0, 0);
      oled.print(F("          "));
      oled.setCursor(0, 2);
      oled.print(call2);
      oled.print(F("          "));
      character = 0;
      data_changeFlag = false;
    }
  }

}


//******************************************************************
//  Edit grid square follows:
//  Used to change grid square
//
//  Called by loop()
//******************************************************************
void EditGrid()
{
  if (data_changeFlag == false)
  {
    SelectionHeader();

    char temp = 35; // Set to "#" to indicate end of text
    oled.setCursor((column * 11), 0);
    oled.print("_");
    oled.setCursor(0, 2);
    oled.print(locator);

    if (digitalRead(pushButton1) == LOW)
    {
      altDelay(300);
      character++;
      if ((character > 0) & (character < 47)) character = 47;
      if ((character > 57) & (character < 65)) character = 65;
      locator[column] = character;
      if (character > 90)
      {
        locator[column] = 35;
        character = 0;
      }

    }
    if (digitalRead(pushButton4) == LOW)
    {
      altDelay(300);
      oled.setCursor((column * 11), 0);
      oled.print(" ");
      column++;
      character = locator[column];
      if (column > 7) column = 0;
    }
    if ((digitalRead(pushButton2) == LOW) & (digitalRead(pushButton3) == LOW))
    {
      for (int i = 0; i < 8; i++)
      {
        if (locator[i] == temp) data_changeFlag = true;
        if (data_changeFlag == true) locator[i] = 0;
      }
      if (data_changeFlag == true)
      {
        oled.setCursor(0, 0);
        oled.print(locator);
        oled.setCursor(0, 2);
        oled.print(F("  SAVED  "));
        EEPROM.put(50, locator);
        EEPROM.put(72, 1);     // Now set grid square change flag
        altDelay(4000);
      }
      else
      {
        oled.setCursor(0, 0);
        oled.print(F("NOT SAVED "));
        oled.setCursor(0, 2);
        oled.print(F("END WITH #"));
        //oled.print(temp);
        altDelay(4000);
      }
      column = 0;
      oled.setCursor(0, 0);
      oled.print(F("          "));
      oled.setCursor(0, 2);
      oled.print(locator);
      oled.print(F("          "));
      character = 0;
      data_changeFlag = false;
    }
  }
}




//******************************************************************
//  Edit power level follows:
//  Used to change power level
//
//  Called by loop()
//******************************************************************
void EditPower()
{
  SelectionHeader();

  oled.setCursor(0, 0);
  oled.print(ndbm);
  double pWatts = pow( 10.0, (ndbm - 30.0) / 10.0);
  oled.setCursor(0, 2);
  oled.print(pWatts, 3);
  oled.print(F(" W"));

  if (digitalRead(pushButton4) == LOW)
  {
    altDelay(200);
    oled.setCursor(0, 0);
    oled.print(F("   "));
    oled.setCursor(0, 2);
    oled.print(F("          "));
    ndbm++;
  }

  if (digitalRead(pushButton1) == LOW)
  {
    altDelay(200);
    oled.setCursor(0, 0);
    oled.print(F("   "));
    oled.setCursor(0, 2);
    oled.print(F("          "));
    ndbm--;
  }

  if ((digitalRead(pushButton2) == LOW) & (digitalRead(pushButton3) == LOW))
  {
    data_changeFlag = true;
    oled.setCursor(0, 0);
    oled.print(ndbm);
    oled.setCursor(0, 2);
    oled.print(F("SAVED   "));
    EEPROM.put(60, ndbm);
    EEPROM.put(74, 1);
    altDelay(2000);
  }
}



//******************************************************************
//  Edit transmit interval follows:
//  Used to determine time between transmissions
//
//  Called by loop()
//******************************************************************
void EditTXinterval()
{
  SelectionHeader();

  oled.setCursor(0, 0);
  oled.print(TXinterval);

  if (digitalRead(pushButton4) == LOW)
  {
    altDelay(200);
    oled.setCursor(0, 0);
    oled.print(F("   "));
    oled.setCursor(0, 2);
    oled.print(F("          "));
    TXinterval++;
  }

  if (digitalRead(pushButton1) == LOW)
  {
    altDelay(200);
    oled.setCursor(0, 0);
    oled.print(F("   "));
    oled.setCursor(0, 2);
    oled.print(F("          "));
    TXinterval--;
  }

  if ((digitalRead(pushButton2) == LOW) & (digitalRead(pushButton3) == LOW))
  {
    oled.setCursor(0, 0);
    oled.print(TXinterval);
    oled.setCursor(0, 2);
    oled.print(F("SAVED   "));
    EEPROM.put(62, TXinterval);
    EEPROM.put(76, 1);
    altDelay(2000);
  }
}



//******************************************************************
//  Edit transmit frequency hop follows:
//  Used to engage/disengage frequency hopping within the WSPR window
//
//  Called by loop()
//******************************************************************
void EditTXhop()
{
  SelectionHeader();

  oled.setCursor(0, 0);
  if (FreqHopTX == true) oled.print(F("YES"));
  else oled.print(F("NO "));

  if (digitalRead(pushButton4) == LOW)
  {
    altDelay(200);
    oled.setCursor(0, 0);
    oled.print(F("NO "));
    FreqHopTX = false;
  }


  if (digitalRead(pushButton1) == LOW)
  {
    altDelay(200);
    oled.setCursor(0, 0);
    oled.print(F("YES"));
    FreqHopTX = true;
  }

  if ((digitalRead(pushButton2) == LOW) & (digitalRead(pushButton3) == LOW))
  {
    oled.setCursor(0, 2);
    oled.print(F("SAVED   "));
    EEPROM.put(78, FreqHopTX);
  }
}



//******************************************************************
//  Edit WSPR offset follows:
//  Used to determine the WSPR offset frequency (1400 - 1600) Hz
//
//  Called by loop()
//******************************************************************
void EditTXoffset()
{
  SelectionHeader();
  oled.setCursor(0, 0);
  oled.print(TXoffset);

  if (digitalRead(pushButton4) == LOW)
  {
    altDelay(200);
    TXoffset++;
    if (TXoffset > 1600UL) TXoffset = 1600UL;
  }

  if (digitalRead(pushButton1) == LOW)
  {
    altDelay(200);
    TXoffset--;
    if (TXoffset < 1400UL) TXoffset = 1400UL;
  }

  if ((digitalRead(pushButton2) == LOW) & (digitalRead(pushButton3) == LOW))
  {
    oled.setCursor(0, 2);
    oled.print(F("SAVED   "));
    EEPROM.put(66, TXoffset);
    EEPROM.put(80, 1);
    altDelay(2000);
  }
}



//******************************************************************
//  Antenna tune alogrithm follows:
//  Used to set up antenna tuning parameters
//
//  Called by loop()
//******************************************************************
void AntennaTune()
{
  do
  {
    // Band selection during non-transmit period
    //if (digitalRead(pushButton3) == LOW & suspendUpdateFlag == false)
    if (digitalRead(pushButton3) == LOW)
    {
      altDelay(500);
      WSPRband++;
      if (RXdialFreq [WSPRband] == 0)WSPRband = 0;
      EEPROM.write(10, WSPRband);
      altDelay(500);
    }
    setfreq();

    if (digitalRead(menu) == LOW) MenuExitFlag = true;

    oled.setCursor(0, 0);
    oled.print(F(" ANT TUNE "));
    digitalWrite(XmitLED, HIGH);
    Si5351_write(CLK_ENABLE_CONTROL, 0b00000100); // Enable CLK0 CLK1 - disable CLK2
    si5351aSetFreq(SYNTH_MS_1, (RXdialFreq [WSPRband] + TXoffset), 0);
    SWRval = analogRead(A3);  // read the input pin
    oled.setCursor(0, 6);
    SWRval = SWRval / 10;
    oled.setFont(Adafruit5x7);
    for (int i = 0; i < 21; i++)
    {
      if (i < SWRval) oled.print(F("*"));
      else oled.print(F(" "));
    }
    oled.setFont(fixed_bold10x15);
    altDelay(100);
    oled.setCursor(0, 6);
    oled.print(F("                      "));
  } while (MenuExitFlag == false);

  Si5351_write(CLK_ENABLE_CONTROL, 0b00000010); // Enable CLK0 CLK2 - disable CLK1
}


//******************************************************************
//  Synchronize GPS follows:
//  Used to update time and grid square via i2c from an external
//  GPS receiver
//
//  Called by loop()
//******************************************************************
void SyncGPS()
{
  oled.setCursor(0, 0);
  oled.print(F("Sync w/ GPS"));
  oled.setCursor(0, 6);
  oled.print(F("2:3 - save"));
  
  Serial.begin(9600);

  if (Serial.available() > 0)
  {
    tempBuf = Serial.read();
    Buffer[bufCounter] = tempBuf;
    bufCounter++;
    if (bufCounter > 10)bufCounter = 0;
    if (tempBuf == 35)
    {

      HourGPS = Buffer[0];
      MinuteGPS = Buffer[1];
      SecondGPS = Buffer[2];
      for (int i = 3; i < 9; i++)
      {
        tempGPS[i - 3] = Buffer[i];
      }
      bufCounter = 0;
    }
  }


  oled.setCursor(0, 2);
  oled.print(tempGPS);
  oled.setCursor(0, 4);
  if (HourGPS < 10) oled.print(F("0"));
  oled.print(HourGPS);
  oled.print(":");
  if (MinuteGPS < 10) oled.print(F("0"));
  oled.print(MinuteGPS);
  oled.print(":");
  if (SecondGPS < 10) oled.print(F("0"));
  oled.print(SecondGPS);

  if ((digitalRead(pushButton2) == LOW) & (digitalRead(pushButton3) == LOW))
  {
    SecondGPS++;              // Corection factor to make up for write delays
    if (SecondGPS > 59)
    {
      SecondGPS = 0;
      MinuteGPS++;
    }

    if (MinuteGPS > 59)
    {
      MinuteGPS = 0;
      HourGPS++;
    }

    if (HourGPS > 23) HourGPS = 0;


    updateTime(SecondGPS, MinuteGPS, HourGPS);
    for (int i = 0; i < 7; i++)
    {
      locator[i] = tempGPS[i];
    }
    EEPROM.put(50, locator);
    EEPROM.put(72, 1);     // Now set grid square change flag
    oled.setCursor(0, 2);
    oled.print(F("SAVED   "));
    altDelay(2000);
  }
}


//******************************************************************
//  Selection Header:
//  Used to display the common header used in some menu selections
//
//  Called by loop()
//******************************************************************
void SelectionHeader()
{
  oled.setCursor(0, 4);
  oled.print("1&4 - edit");
  oled.setCursor(0, 6);
  oled.print("2:3 - save");
}




//******************************************************************
//  Time update function follows:
//  Used to retrieve the correct time from the DS3231 RTC
//
//  Called by displayClock(), and loop()
//******************************************************************
void  getTime()
{
  // Send request to receive data starting at register 0
  Wire.beginTransmission(DS3231_addr); // DS3231_addr is DS3231 device address
  Wire.write((byte)0); // start at register 0
  Wire.endTransmission();
  Wire.requestFrom(DS3231_addr, 3); // request three bytes (seconds, minutes, hours)

  while (Wire.available())
  {
    seconds = Wire.read(); // get seconds
    minutes = Wire.read(); // get minutes
    hours = Wire.read();   // get hours

    seconds = (((seconds & 0b11110000) >> 4) * 10 + (seconds & 0b00001111)); // convert BCD to decimal
    minutes = (((minutes & 0b11110000) >> 4) * 10 + (minutes & 0b00001111)); // convert BCD to decimal
    hours = (((hours & 0b00100000) >> 5) * 20 + ((hours & 0b00010000) >> 4) * 10 + (hours & 0b00001111)); // convert BCD to decimal (assume 24 hour mode)
  }
}


//******************************************************************
//  Date update function follows:
//  Used to retrieve the correct date from the DS3231 RTC
//
//  The day of the week algorithm is a modified version
//  of the open source code found at:
//  Code by JeeLabs http://news.jeelabs.org/code/
//
//  Called by displayClock()
//******************************************************************
void  getDate()
{
  int nowDay;
  int nowDate;
  int tempdate;
  int nowMonth;
  int nowYear;

  // send request to receive data starting at register 3
  Wire.beginTransmission(DS3231_addr); // DS3231_addr is DS3231 device address
  Wire.write((byte)0x03); // start at register 3
  Wire.endTransmission();
  Wire.requestFrom(DS3231_addr, 4); // request four bytes (day date month year)

  while (Wire.available())
  {
    nowDay = Wire.read();    // get day (serves as a placeholder)
    nowDate = Wire.read();   // get date
    nowMonth = Wire.read();  // get month
    nowYear = Wire.read();   // get year

    xdate = (((nowDate & 0b11110000) >> 4) * 10 + (nowDate & 0b00001111)); // convert BCD to decimal
    tempdate = xdate;
    xmonth = (((nowMonth & 0b00010000) >> 4) * 10 + (nowMonth & 0b00001111)); // convert BCD to decimal
    xyear = ((nowYear & 0b11110000) >> 4) * 10 + ((nowYear & 0b00001111)); // convert BCD to decimal

    if (xyear >= 2000) xyear -= 2000;
    for (byte i = 1; i < xmonth; ++i)
      tempdate += pgm_read_byte(daysInMonth + i - 1);
    if (xmonth > 2 && xyear % 4 == 0)
      ++tempdate;
    tempdate = tempdate + 365 * xyear + (xyear + 3) / 4 - 1;
    xday = (tempdate + 6) % 7; // Jan 1, 2000 is a Saturday, i.e. returns 6
  }
}


//******************************************************************
//  Time set function follows:
//  Used to set the DS3231 RTC time
//
//  Called by loop()
//******************************************************************
void updateTime(int updateSecond, int updateMinute, int updateHour)
{
  // Convert BIN to BCD
  updateSecond = updateSecond + 6 * (updateSecond / 10);
  updateMinute = updateMinute + 6 * (updateMinute / 10);
  updateHour = updateHour + 6 * (updateHour / 10);

  // Write the data
  Wire.beginTransmission(DS3231_addr);
  Wire.write((byte)0); // start at location 0
  Wire.write(updateSecond);
  Wire.write(updateMinute);
  Wire.write(updateHour);
  Wire.endTransmission();
}


//******************************************************************
// Alternate delay function follows:
// altDelay is used because the command "delay" causes critical
// timing errors.
//
// Called by all functions
//******************************************************************
unsigned long altDelay(unsigned long delayTime)
{
  time_now = millis();
  while (millis() < time_now + delayTime) //delay 1 second
  {
    __asm__ __volatile__ ("nop");
  }
}


//******************************************************************
// WSPR status function follows:
// Displays WSPR transmitter ON/OFF status
//
// Called by loop()
//******************************************************************
void WSPRstatus()
{
  if (WSPRflag == false)
  {
    oled.setCursor(97, 0);
    oled.print(F("OFF"));
  }
  else
  {
    oled.setCursor(97, 0);
    oled.print(F(" ON"));
  }
}



//******************************************************************
// displayClock follows:
// Displays the time and date during WSPR function operation
//
// Called by loop()
//******************************************************************
void displayClock()
{
  oled.setCursor(12, 4);
  getTime();

  if (hours < 10) oled.print(F("0"));
  oled.print(hours);
  oled.print(F(":"));

  if (minutes < 10) oled.print(F("0"));
  oled.print(minutes);
  oled.print(F(":"));

  if (seconds < 10) oled.print(F("0"));
  oled.print(seconds);

  if (millis() > time_now + 2000)
  {
    if (toggle == true)
    {
      tcount3++;
      if (tcount3 > 5) tcount3 = 0;
      oled.setCursor(0, 6);
      oled.print(F("            "));
    }
    toggle = !toggle;
    time_now = millis();

  }

  if (toggle == false)
  {
    getDate();
    oled.setCursor(0, 6);
    switch (tcount3)
    {
      case 0: // Display WSPR TX offset
        oled.print (F("Offset:"));
        oled.print(TXoffset);
        break;

      case 1: // Display grid square
        oled.print(F("Grid: "));
        oled.print (locator);
        break;

      case 2: // Display power
        oled.print(F("PWR: "));
        oled.print (ndbm);
        oled.print(F("dBm "));
        break;

      case 3: // Display transmit interval
        oled.print(F("TXint: "));
        oled.print (TXinterval);
        break;

      case 4: // Display frequency hopping YES/NO
        oled.print(F("TXhop: "));
        if (FreqHopTX == false) oled.print(F("NO"));
        else oled.print(F("YES"));
        break;

      case 5: // Display date
        descriptionType oneItem;
        memcpy_P (&oneItem, &daysOfTheWeek [xday], sizeof oneItem);
        oled.print (oneItem.description);
        oled.print(F(" "));

        if (xdate < 10) oled.print(F("0"));
        oled.print(xdate);
        oled.print(F(" "));
        memcpy_P (&oneItem, &monthOfTheYear [xmonth - 1], sizeof oneItem);
        oled.print (oneItem.description);
        oled.print(F("  "));
        break;
    }

  }
}



//******************************************************************
// Set WSPR frequency function follows:
// Calculates the frequency data to be sent to the Si5351 clock generator
// and displays the frequency on the olded display
//
// Called by loop(), setup, and transmit()
//******************************************************************
void setfreq()
{
  unsigned long  frequency = RXdialFreq [WSPRband] + TXoffset; // Temporarily store Freq_1

  oled.setCursor(0, 2);
  char buf[11];
  if (frequency >= 1000000UL)
  {
    int MHz = int(frequency / 1000000UL);
    int kHz = int ((frequency - (MHz * 1000000UL)) / 1000UL);
    int Hz = int (frequency % 1000UL);
    snprintf(buf, sizeof(buf), "%2u,%03u,%03u", MHz, kHz, Hz);
  }

  else if (frequency >= 1000UL & frequency < 1000000UL)
  {
    int kHz = int (frequency / 1000UL);
    int Hz = int (frequency % 1000UL);
    snprintf(buf, sizeof(buf), "%6u,%03u", kHz, Hz);
  }
  else if (frequency < 1000UL)
  {
    int Hz = int (frequency);
    snprintf(buf, sizeof(buf), "%10u", Hz);
  }
  oled.print(buf);
}


//******************************************************************
// WSPR transmit algorithm follows:
// Configures WSPR data and timing and then sends data to the Si5351 clock timer
//
// Called by loop()
//******************************************************************
void transmit()
{
  TXcount++;
  time_now = millis();
  altDelay(1000); // Delay 1 second
  if (TXcount > TXinterval) TXcount = 1;
  if (TXcount != TXinterval) return;
  suspendUpdateFlag = true;
  if (FreqHopTX == true) // Enables in-band TX frequency hopping in incremental 15Hz steps
  {
    TXoffset = TXoffset + 10UL;
    if (TXoffset > 1550UL) TXoffset = 1440UL;
  }
  Si5351_write(CLK_ENABLE_CONTROL, 0b00000100); // Enable CLK0 CLK1 - disable CLK2 (to enhance Si5351 freq stability)
  setfreq();
  oled.setCursor(0, 4);
  oled.print(F(" XMITTING "));
  digitalWrite(XmitLED, HIGH);
  unsigned long currentTime = millis();
  for (int count = 0; count < 162; count++)
  {
    unsigned long timer = millis();
    si5351aSetFreq(SYNTH_MS_1, (RXdialFreq [WSPRband] + TXoffset), OffsetFreq[sym[count]]);
    while ((millis() - timer) <= 682UL) {
      __asm__("nop\n\t");
    };
  }
  Si5351_write(CLK_ENABLE_CONTROL, 0b00000010); // Enable CLK0 CLK2 - disable CLK1 (drift compensation)
  si5351aSetFreq(SYNTH_MS_2, 2500000UL, 0);        // CLK2 is enabled to balance thermal drift between transmissions
  suspendUpdateFlag = false;
  digitalWrite(XmitLED, LOW);
  if (calltype == 2)
  {
    msg_type = !msg_type;
    wsprGenCode();
  }
}




//******************************************************************
//  Si5351 processing follows:
//  Generates the Si5351 clock generator frequency message
//
//  Called by sketch setup() and loop()
//******************************************************************
void si5351aSetFreq(int synth, unsigned long  frequency, int symbolOffset)
{
  unsigned long long  CalcTemp;
  unsigned long PLLfreq, divider, a, b, c, p1, p2, p3, PLL_P1, PLL_P2, PLL_P3;
  byte dividerBits = 0;
  int PLL, multiplier = 1;
  if (synth == SYNTH_MS_0) PLL = SYNTH_PLL_A;
  else PLL = SYNTH_PLL_B;

  frequency = frequency * 100UL + symbolOffset;
  c = 0xFFFFF;  // Denominator derived from max bits 2^20

  divider = 90000000000ULL / frequency;
  while (divider > 900UL) {           // If output divider out of range (>900) use additional Output divider
    dividerBits++;
    divider = divider / 2UL;
    multiplier = multiplier * 2;
    //multiplier = dividerBits << 4;
  }

  dividerBits = dividerBits << 4;
  if (divider % 2) divider--;
//  PLLfreq = (divider * multiplier * frequency) / 10UL;

  unsigned long long PLLfreq2, divider2 = divider, multiplier2 = multiplier;
  PLLfreq2 = (divider2 * multiplier2 * frequency) / 100UL;
  PLLfreq = PLLfreq2;

  a = PLLfreq / XtalFreq;
  CalcTemp = PLLfreq2 - a * XtalFreq;
  CalcTemp *= c;
  CalcTemp /= XtalFreq ;
  b = CalcTemp;  // Calculated numerator


  p1 = 128UL * divider - 512UL;
  CalcTemp = 128UL * b / c;
  PLL_P1 = 128UL * a + CalcTemp - 512UL;
  PLL_P2 = 128UL * b - CalcTemp * c;
  PLL_P3 = c;


  // Write data to PLL registers
  Si5351_write(PLL, 0xFF);
  Si5351_write(PLL + 1, 0xFF);
  Si5351_write(PLL + 2, (PLL_P1 & 0x00030000) >> 16);
  Si5351_write(PLL + 3, (PLL_P1 & 0x0000FF00) >> 8);
  Si5351_write(PLL + 4, (PLL_P1 & 0x000000FF));
  Si5351_write(PLL + 5, 0xF0 | ((PLL_P2 & 0x000F0000) >> 16));
  Si5351_write(PLL + 6, (PLL_P2 & 0x0000FF00) >> 8);
  Si5351_write(PLL + 7, (PLL_P2 & 0x000000FF));


  // Write data to multisynth registers
  Si5351_write(synth, 0xFF);
  Si5351_write(synth + 1, 0xFF);
  Si5351_write(synth + 2, (p1 & 0x00030000) >> 16 | dividerBits);
  Si5351_write(synth + 3, (p1 & 0x0000FF00) >> 8);
  Si5351_write(synth + 4, (p1 & 0x000000FF));
  Si5351_write (synth + 5, 0);
  Si5351_write (synth + 6, 0);
  Si5351_write (synth + 7, 0);

}



//******************************************************************
// Write I2C data function for the Si5351A follows:
// Writes data over the I2C bus to the appropriate device defined by
// the address sent to it.

// Called by sketch setup, WSPR, transmit(), si5351aSetFreq, and
// si5351aStart functions.
//******************************************************************
uint8_t Si5351_write(uint8_t addr, uint8_t data)
{
  Wire.beginTransmission(Si5351A_addr);
  Wire.write(addr);
  Wire.write(data);
  Wire.endTransmission();
}





//******************************************************************
// Set the DS3231 RTC crystal oscillator aging offset function follows:
// This is effectively the system calibration routine. Small
// capacitors can be swithed in or out within the DS3231 RTC to control
// frequency accuracy. Positive aging values add capacitance to the
// array, slowing the oscillator frequency. Negative values remove
// capacitance from the array, increasing the oscillator frequency.
// One offset count provides about 0.1 ppm change in frequency.

// To enter the calibration function hold down pushbutton 4 and invoke
// a reset. Refer to the calibration notes below.
//******************************************************************

void Calibrate()
{
  int CF, tempCF, busy;
  //oled.setFont(fixed_bold10x15);
  //oled.set1X();
  oled.clear();
  oled.print(F("Cal CF ="));
  oled.setCursor(0, 2);
  oled.println(F("PB1 = Down"));
  oled.println(F("PB2 = Up"));
  oled.print(F("reset = Exit"));

  while (digitalRead(endCalibrate) == HIGH)
  {
    // send request to receive data starting at register 3
    Wire.beginTransmission(DS3231_addr); // DS3231_addr is DS3231 device address
    Wire.write((byte)0x10); // start at register 0x10
    Wire.endTransmission();
    Wire.requestFrom(DS3231_addr, 1); // request one byte (aging factor)
    while (Wire.available())
    {
      CF = Wire.read();
    }
    if (CF > 127) CF -= 256;
    tempCF = CF;
    oled.setCursor(96, 0);
    oled.print(F("    "));
    oled.setCursor(96, 0);
    oled.println(CF);
    if (digitalRead(CFdown) == LOW) CF -= 1;
    if (digitalRead(CFup) == LOW) CF += 1;

    Wire.beginTransmission(DS3231_addr); // DS3231_addr is DS3231 device address
    Wire.write((byte)0x0F); // start at register 0x0F
    Wire.endTransmission();
    Wire.requestFrom(DS3231_addr, 1); // request one byte to determine DS3231 update status
    while (Wire.available())
    {
      busy = Wire.read();
    }
    busy = bitRead(busy, 2);
    if (CF != tempCF & bitRead(busy, 2) == 0)
    {
      setAgingOffset(CF);
      forceConversion();
    }
    altDelay(500);
  }
}


void setAgingOffset(int offset)
{
  if (offset < 0) offset += 256;

  Wire.beginTransmission(DS3231_addr);
  Wire.write(0x10);
  Wire.write(offset);
  Wire.endTransmission();
}


void forceConversion()
{
  Wire.beginTransmission(DS3231_addr);
  Wire.write(0x0E);

  Wire.write(B00111100);
  Wire.endTransmission();
}



//***********************************************************************
// WSPR message generation algorithm follows:
// Configures the unique WSPR message based upon your callsign, location, and power.
// Note: Type 2 callsigns (e.g. GM/W3PM) are supported in this version
//
// Called by sketch setup.
//***********************************************************************
void wsprGenCode()
{
  for (i = 0; i < 11; i++) {
    if (call2[i] == 47)calltype = 2;
  };
  if (calltype == 2) type2();
  else
  {
    for (i = 0; i < 7; i++) {
      call1[i] = call2[i];
    };
    for (i = 0; i < 5; i++) {
      grid4[i] = locator[i];
    };
    packcall();
    packgrid();
    n2 = ng * 128 + ndbm + 64;
    pack50();
    encode_conv();
    interleave_sync();
  }
}

//******************************************************************
void type2()
{
  if (msg_type == 0)
  {
    packpfx();
    ntype = ndbm + 1 + nadd;
    n2 = 128 * ng + ntype + 64;
    pack50();
    encode_conv();
    interleave_sync();
  }
  else
  {
    hash();
    for (ii = 1; ii < 6; ii++)
    {
      call1[ii - 1] = locator[ii];
    };
    call1[5] = locator[0];
    packcall();
    ntype = -(ndbm + 1);
    n2 = 128 * ihash + ntype + 64;
    pack50();
    encode_conv();
    interleave_sync();
  };

}

//******************************************************************
void packpfx()
{
  char pfx[3];
  int Len;
  int slash;

  for (i = 0; i < 7; i++)
  {
    call1[i] = 0;
  };
  Len = strlen(call2);
  for (i = 0; i < 11; i++)
  {
    if (call2[i] == 47) slash = i;
  };
  if (call2[slash + 2] == 0)
  { //single char add-on suffix
    for (i = 0; i < slash; i++)
    {
      call1[i] = call2[i];
    };
    packcall();
    nadd = 1;
    nc = int(call2[slash + 1]);
    if (nc >= 48 && nc <= 57) n = nc - 48;
    else if (nc >= 65 && nc <= 90) n = nc - 65 + 10;
    else if (nc >= 97 && nc <= 122) n = nc - 97 + 10;
    else n = 38;
    ng = 60000 - 32768 + n;
  }
  else if (call2[slash + 3] == 0)
  {
    for (i = 0; i < slash; i++)
    {
      call1[i] = call2[i];
    };
    packcall();
    n = 10 * (int(call2[slash + 1]) - 48) +  int(call2[slash + 2]) - 48;
    nadd = 1;
    ng = 60000 + 26 + n;
  }
  else
  {
    for (i = 0; i < slash; i++)
    {
      pfx[i] = call2[i];
    };
    if (slash == 2)
    {
      pfx[2] = pfx[1];
      pfx[1] = pfx[0];
      pfx[0] = ' ';
    };
    if (slash == 1)
    {
      pfx[2] = pfx[0];
      pfx[1] = ' ';
      pfx[0] = ' ';
    };
    ii = 0;
    for (i = slash + 1; i < Len; i++)
    {
      call1[ii] = call2[i];
      ii++;
    };
    packcall();
    ng = 0;
    for (i = 0; i < 3; i++)
    {
      nc = int(pfx[i]);
      if (nc >= 48 && nc <= 57) n = nc - 48;
      else if (nc >= 65 && nc <= 90) n = nc - 65 + 10;
      else if (nc >= 97 && nc <= 122) n = nc - 97 + 10;
      else n = 36;
      ng = 37 * ng + n;
    };
    nadd = 0;
    if (ng >= 32768)
    {
      ng = ng - 32768;
      nadd = 1;
    };
  }
}

//******************************************************************
void packcall()
{
  // coding of callsign
  if (chr_normf(call1[2]) > 9)
  {
    call1[5] = call1[4];
    call1[4] = call1[3];
    call1[3] = call1[2];
    call1[2] = call1[1];
    call1[1] = call1[0];
    call1[0] = ' ';
  }

  n1 = chr_normf(call1[0]);
  n1 = n1 * 36 + chr_normf(call1[1]);
  n1 = n1 * 10 + chr_normf(call1[2]);
  n1 = n1 * 27 + chr_normf(call1[3]) - 10;
  n1 = n1 * 27 + chr_normf(call1[4]) - 10;
  n1 = n1 * 27 + chr_normf(call1[5]) - 10;
}

//******************************************************************
void packgrid()
{
  // coding of grid4
  ng = 179 - 10 * (chr_normf(grid4[0]) - 10) - chr_normf(grid4[2]);
  ng = ng * 180 + 10 * (chr_normf(grid4[1]) - 10) + chr_normf(grid4[3]);
}

//******************************************************************
void pack50()
{
  // merge coded callsign into message array c[]
  t1 = n1;
  c[0] = t1 >> 20;
  t1 = n1;
  c[1] = t1 >> 12;
  t1 = n1;
  c[2] = t1 >> 4;
  t1 = n1;
  c[3] = t1 << 4;
  t1 = n2;
  c[3] = c[3] + ( 0x0f & t1 >> 18);
  t1 = n2;
  c[4] = t1 >> 10;
  t1 = n2;
  c[5] = t1 >> 2;
  t1 = n2;
  c[6] = t1 << 6;
}

//******************************************************************
//void hash(string,len,ihash)
void hash()
{
  int Len;
  uint32_t jhash;
  int *pLen = &Len;
  Len = strlen(call2);
  byte IC[12];
  byte *pIC = IC;
  for (i = 0; i < 12; i++)
  {
    pIC + 1;
    &IC[i];
  }
  uint32_t Val = 146;
  uint32_t *pVal = &Val;
  for (i = 0; i < Len; i++)
  {
    IC[i] = int(call2[i]);
  };
  jhash = nhash_(pIC, pLen, pVal);
  ihash = jhash & MASK15;
  return;
}
//******************************************************************
// normalize characters 0..9 A..Z Space in order 0..36
char chr_normf(char bc )
{
  char cc = 36;
  if (bc >= '0' && bc <= '9') cc = bc - '0';
  if (bc >= 'A' && bc <= 'Z') cc = bc - 'A' + 10;
  if (bc >= 'a' && bc <= 'z') cc = bc - 'a' + 10;
  if (bc == ' ' ) cc = 36;

  return (cc);
}


//******************************************************************
// convolutional encoding of message array c[] into a 162 bit stream
void encode_conv()
{
  int bc = 0;
  int cnt = 0;
  int cc;
  unsigned long sh1 = 0;

  cc = c[0];

  for (int i = 0; i < 81; i++) {
    if (i % 8 == 0 ) {
      cc = c[bc];
      bc++;
    }
    if (cc & 0x80) sh1 = sh1 | 1;

    symt[cnt++] = parity(sh1 & 0xF2D05351);
    symt[cnt++] = parity(sh1 & 0xE4613C47);

    cc = cc << 1;
    sh1 = sh1 << 1;
  }
}

//******************************************************************
byte parity(unsigned long li)
{
  byte po = 0;
  while (li != 0)
  {
    po++;
    li &= (li - 1);
  }
  return (po & 1);
}

//******************************************************************
// interleave reorder the 162 data bits and and merge table with the sync vector
void interleave_sync()
{
  int ii, ij, b2, bis, ip;
  ip = 0;

  for (ii = 0; ii <= 255; ii++) {
    bis = 1;
    ij = 0;
    for (b2 = 0; b2 < 8 ; b2++) {
      if (ii & bis) ij = ij | (0x80 >> b2);
      bis = bis << 1;
    }
    if (ij < 162 ) {
      sym[ij] = SyncVec[ij] + 2 * symt[ip];
      ip++;
    }
  }
}


//_____________________________________________________________________________
// Note: The parts of the routine that follows is used for WSPR type 2 callsigns
//(i.e. GM/W3PM) to generate the required hash code.
//_____________________________________________________________________________

/*
  -------------------------------------------------------------------------------
  lookup3.c, by Bob Jenkins, May 2006, Public Domain.

  These are functions for producing 32-bit hashes for hash table lookup.
  hashword(), hashlittle(), hashlittle2(), hashbig(), mix(), and final()
  are externally useful functions.  Routines to test the hash are included
  if SELF_TEST is defined.  You can use this free for any purpose.  It's in
  the public domain.  It has no warranty.

  You probably want to use hashlittle().  hashlittle() and hashbig()
  hash byte arrays.  hashlittle() is is faster than hashbig() on
  little-endian machines.  Intel and AMD are little-endian machines.
  On second thought, you probably want hashlittle2(), which is identical to
  hashlittle() except it returns two 32-bit hashes for the price of one.
  You could implement hashbig2() if you wanted but I haven't bothered here.

  If you want to find a hash of, say, exactly 7 integers, do
  a = i1;  b = i2;  c = i3;
  mix(a,b,c);
  a += i4; b += i5; c += i6;
  mix(a,b,c);
  a += i7;
  final(a,b,c);
  then use c as the hash value.  If you have a variable length array of
  4-byte integers to hash, use hashword().  If you have a byte array (like
  a character string), use hashlittle().  If you have several byte arrays, or
  a mix of things, see the comments above hashlittle().

  Why is this so big?  I read 12 bytes at a time into 3 4-byte integers,
  then mix those integers.  This is fast (you can do a lot more thorough
  mixing with 12*3 instructions on 3 integers than you can with 3 instructions
  on 1 byte), but shoehorning those bytes into integers efficiently is messy.
  -------------------------------------------------------------------------------
*/

//#define SELF_TEST 1

//#include <stdio.h>      /* defines printf for tests */
//#include <time.h>       /* defines time_t for timings in the test */
//#ifdef Win32
//#include "win_stdint.h"  /* defines uint32_t etc */
//#else
//#include <stdint.h> /* defines uint32_t etc */
//#endif

//#include <sys/param.h>  /* attempt to define endianness */
//#ifdef linux
//# include <endian.h>    /* attempt to define endianness */
//#endif

#define HASH_LITTLE_ENDIAN 1

#define hashsize(n) ((uint32_t)1<<(n))
#define hashmask(n) (hashsize(n)-1)
#define rot(x,k) (((x)<<(k)) | ((x)>>(32-(k))))

/*
  -------------------------------------------------------------------------------
  mix -- mix 3 32-bit values reversibly.

  This is reversible, so any information in (a,b,c) before mix() is
  still in (a,b,c) after mix().

  If four pairs of (a,b,c) inputs are run through mix(), or through
  mix() in reverse, there are at least 32 bits of the output that
  are sometimes the same for one pair and different for another pair.
  This was tested for:
  pairs that differed by one bit, by two bits, in any combination
  of top bits of (a,b,c), or in any combination of bottom bits of
  (a,b,c).
  "differ" is defined as +, -, ^, or ~^.  For + and -, I transformed
  the output delta to a Gray code (a^(a>>1)) so a string of 1's (as
  is commonly produced by subtraction) look like a single 1-bit
  difference.
  the base values were pseudorandom, all zero but one bit set, or
  all zero plus a counter that starts at zero.

  Some k values for my "a-=c; a^=rot(c,k); c+=b;" arrangement that
  satisfy this are
    4  6  8 16 19  4
    9 15  3 18 27 15
   14  9  3  7 17  3
  Well, "9 15 3 18 27 15" didn't quite get 32 bits diffing
  for "differ" defined as + with a one-bit base and a two-bit delta.  I
  used http://burtleburtle.net/bob/hash/avalanche.html to choose
  the operations, constants, and arrangements of the variables.

  This does not achieve avalanche.  There are input bits of (a,b,c)
  that fail to affect some output bits of (a,b,c), especially of a.  The
  most thoroughly mixed value is c, but it doesn't really even achieve
  avalanche in c.

  This allows some parallelism.  Read-after-writes are good at doubling
  the number of bits affected, so the goal of mixing pulls in the opposite
  direction as the goal of parallelism.  I did what I could.  Rotates
  seem to cost as much as shifts on every machine I could lay my hands
  on, and rotates are much kinder to the top and bottom bits, so I used
  rotates.
  -------------------------------------------------------------------------------
*/
#define mix(a,b,c) \
  { \
    a -= c;  a ^= rot(c, 4);  c += b; \
    b -= a;  b ^= rot(a, 6);  a += c; \
    c -= b;  c ^= rot(b, 8);  b += a; \
    a -= c;  a ^= rot(c,16);  c += b; \
    b -= a;  b ^= rot(a,19);  a += c; \
    c -= b;  c ^= rot(b, 4);  b += a; \
  }

/*
  -------------------------------------------------------------------------------
  final -- final mixing of 3 32-bit values (a,b,c) into c

  Pairs of (a,b,c) values differing in only a few bits will usually
  produce values of c that look totally different.  This was tested for
  pairs that differed by one bit, by two bits, in any combination
  of top bits of (a,b,c), or in any combination of bottom bits of
  (a,b,c).
  "differ" is defined as +, -, ^, or ~^.  For + and -, I transformed
  the output delta to a Gray code (a^(a>>1)) so a string of 1's (as
  is commonly produced by subtraction) look like a single 1-bit
  difference.
  the base values were pseudorandom, all zero but one bit set, or
  all zero plus a counter that starts at zero.

  These constants passed:
  14 11 25 16 4 14 24
  12 14 25 16 4 14 24
  and these came close:
  4  8 15 26 3 22 24
  10  8 15 26 3 22 24
  11  8 15 26 3 22 24
  -------------------------------------------------------------------------------
*/
#define final(a,b,c) \
  { \
    c ^= b; c -= rot(b,14); \
    a ^= c; a -= rot(c,11); \
    b ^= a; b -= rot(a,25); \
    c ^= b; c -= rot(b,16); \
    a ^= c; a -= rot(c,4);  \
    b ^= a; b -= rot(a,14); \
    c ^= b; c -= rot(b,24); \
  }

/*
  -------------------------------------------------------------------------------
  hashlittle() -- hash a variable-length key into a 32-bit value
  k       : the key (the unaligned variable-length array of bytes)
  length  : the length of the key, counting by bytes
  initval : can be any 4-byte value
  Returns a 32-bit value.  Every bit of the key affects every bit of
  the return value.  Two keys differing by one or two bits will have
  totally different hash values.

  The best hash table sizes are powers of 2.  There is no need to do
  mod a prime (mod is sooo slow!).  If you need less than 32 bits,
  use a bitmask.  For example, if you need only 10 bits, do
  h = (h & hashmask(10));
  In which case, the hash table should have hashsize(10) elements.

  If you are hashing n strings (uint8_t **)k, do it like this:
  for (i=0, h=0; i<n; ++i) h = hashlittle( k[i], len[i], h);

  By Bob Jenkins, 2006.  bob_jenkins@burtleburtle.net.  You may use this
  code any way you wish, private, educational, or commercial.  It's free.

  Use for hash table lookup, or anything where one collision in 2^^32 is
  acceptable.  Do NOT use for cryptographic purposes.
  -------------------------------------------------------------------------------
*/

//uint32_t hashlittle( const void *key, size_t length, uint32_t initval)
#ifdef STDCALL
uint32_t __stdcall NHASH( const void *key, size_t *length0, uint32_t *initval0)
#else
uint32_t nhash_( const void *key, int *length0, uint32_t *initval0)
#endif
{
  uint32_t a, b, c;                                        /* internal state */
  size_t length;
  uint32_t initval;
  union {
    const void *ptr;
    size_t i;
  } u;     /* needed for Mac Powerbook G4 */

  length = *length0;
  initval = *initval0;

  /* Set up the internal state */
  a = b = c = 0xdeadbeef + ((uint32_t)length) + initval;

  u.ptr = key;
  if (HASH_LITTLE_ENDIAN && ((u.i & 0x3) == 0)) {
    const uint32_t *k = (const uint32_t *)key;         /* read 32-bit chunks */
    const uint8_t  *k8;

    k8 = 0;                                   //Silence compiler warning
    /*------ all but last block: aligned reads and affect 32 bits of (a,b,c) */
    while (length > 12)
    {
      a += k[0];
      b += k[1];
      c += k[2];
      mix(a, b, c);
      length -= 12;
      k += 3;
    }

    /*----------------------------- handle the last (probably partial) block */
    /*
       "k[2]&0xffffff" actually reads beyond the end of the string, but
       then masks off the part it's not allowed to read.  Because the
       string is aligned, the masked-off tail is in the same word as the
       rest of the string.  Every machine with memory protection I've seen
       does it on word boundaries, so is OK with this.  But VALGRIND will
       still catch it and complain.  The masking trick does make the hash
       noticably faster for short strings (like English words).
    */
#ifndef VALGRIND

    switch (length)
    {
      case 12: c += k[2]; b += k[1]; a += k[0]; break;
      case 11: c += k[2] & 0xffffff; b += k[1]; a += k[0]; break;
      case 10: c += k[2] & 0xffff; b += k[1]; a += k[0]; break;
      case 9 : c += k[2] & 0xff; b += k[1]; a += k[0]; break;
      case 8 : b += k[1]; a += k[0]; break;
      case 7 : b += k[1] & 0xffffff; a += k[0]; break;
      case 6 : b += k[1] & 0xffff; a += k[0]; break;
      case 5 : b += k[1] & 0xff; a += k[0]; break;
      case 4 : a += k[0]; break;
      case 3 : a += k[0] & 0xffffff; break;
      case 2 : a += k[0] & 0xffff; break;
      case 1 : a += k[0] & 0xff; break;
      case 0 : return c;              /* zero length strings require no mixing */
    }

#else /* make valgrind happy */

    k8 = (const uint8_t *)k;
    switch (length)
    {
      case 12: c += k[2]; b += k[1]; a += k[0]; break;
      case 11: c += ((uint32_t)k8[10]) << 16; /* fall through */
      case 10: c += ((uint32_t)k8[9]) << 8; /* fall through */
      case 9 : c += k8[8];                 /* fall through */
      case 8 : b += k[1]; a += k[0]; break;
      case 7 : b += ((uint32_t)k8[6]) << 16; /* fall through */
      case 6 : b += ((uint32_t)k8[5]) << 8; /* fall through */
      case 5 : b += k8[4];                 /* fall through */
      case 4 : a += k[0]; break;
      case 3 : a += ((uint32_t)k8[2]) << 16; /* fall through */
      case 2 : a += ((uint32_t)k8[1]) << 8; /* fall through */
      case 1 : a += k8[0]; break;
      case 0 : return c;
    }

#endif /* !valgrind */

  } else if (HASH_LITTLE_ENDIAN && ((u.i & 0x1) == 0)) {
    const uint16_t *k = (const uint16_t *)key;         /* read 16-bit chunks */
    const uint8_t  *k8;

    /*--------------- all but last block: aligned reads and different mixing */
    while (length > 12)
    {
      a += k[0] + (((uint32_t)k[1]) << 16);
      b += k[2] + (((uint32_t)k[3]) << 16);
      c += k[4] + (((uint32_t)k[5]) << 16);
      mix(a, b, c);
      length -= 12;
      k += 6;
    }

    /*----------------------------- handle the last (probably partial) block */
    k8 = (const uint8_t *)k;
    switch (length)
    {
      case 12: c += k[4] + (((uint32_t)k[5]) << 16);
        b += k[2] + (((uint32_t)k[3]) << 16);
        a += k[0] + (((uint32_t)k[1]) << 16);
        break;
      case 11: c += ((uint32_t)k8[10]) << 16; /* fall through */
      case 10: c += k[4];
        b += k[2] + (((uint32_t)k[3]) << 16);
        a += k[0] + (((uint32_t)k[1]) << 16);
        break;
      case 9 : c += k8[8];                    /* fall through */
      case 8 : b += k[2] + (((uint32_t)k[3]) << 16);
        a += k[0] + (((uint32_t)k[1]) << 16);
        break;
      case 7 : b += ((uint32_t)k8[6]) << 16;  /* fall through */
      case 6 : b += k[2];
        a += k[0] + (((uint32_t)k[1]) << 16);
        break;
      case 5 : b += k8[4];                    /* fall through */
      case 4 : a += k[0] + (((uint32_t)k[1]) << 16);
        break;
      case 3 : a += ((uint32_t)k8[2]) << 16;  /* fall through */
      case 2 : a += k[0];
        break;
      case 1 : a += k8[0];
        break;
      case 0 : return c;                     /* zero length requires no mixing */
    }

  } else {                        /* need to read the key one byte at a time */
    const uint8_t *k = (const uint8_t *)key;

    /*--------------- all but the last block: affect some 32 bits of (a,b,c) */
    while (length > 12)
    {
      a += k[0];
      a += ((uint32_t)k[1]) << 8;
      a += ((uint32_t)k[2]) << 16;
      a += ((uint32_t)k[3]) << 24;
      b += k[4];
      b += ((uint32_t)k[5]) << 8;
      b += ((uint32_t)k[6]) << 16;
      b += ((uint32_t)k[7]) << 24;
      c += k[8];
      c += ((uint32_t)k[9]) << 8;
      c += ((uint32_t)k[10]) << 16;
      c += ((uint32_t)k[11]) << 24;
      mix(a, b, c);
      length -= 12;
      k += 12;
    }

    /*-------------------------------- last block: affect all 32 bits of (c) */
    switch (length)                  /* all the case statements fall through */
    {
      case 12: c += ((uint32_t)k[11]) << 24;
      case 11: c += ((uint32_t)k[10]) << 16;
      case 10: c += ((uint32_t)k[9]) << 8;
      case 9 : c += k[8];
      case 8 : b += ((uint32_t)k[7]) << 24;
      case 7 : b += ((uint32_t)k[6]) << 16;
      case 6 : b += ((uint32_t)k[5]) << 8;
      case 5 : b += k[4];
      case 4 : a += ((uint32_t)k[3]) << 24;
      case 3 : a += ((uint32_t)k[2]) << 16;
      case 2 : a += ((uint32_t)k[1]) << 8;
      case 1 : a += k[0];
        break;
      case 0 : return c;
    }
  }

  final(a, b, c);
  return c;
}
