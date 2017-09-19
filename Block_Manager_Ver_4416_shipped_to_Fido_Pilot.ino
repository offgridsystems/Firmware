
// Block Manager PCB Rev D production code for Teensy LC  -    10/26/2015  T Economu

// Date       Who     New Ver   WAS/IS changes
// 1Nov2015   TBE               Added software RTC from Chateau Economu code for secs, mins, and hours, and DO NOT roll at with 4 billion in
//                              UNSIGNED LONG integers, we get over 127 years!
// Jun 2016   TBE               WAS: >1% accuracy at best/ IS: 0.1% average accuracy, by simply calibrating output and adding 1uf cap to Vcell inputs.
// 1JUL2016   TBE               WAS: Only wake from sleep was charge/discharge cell voltage delta Is: added comm wake, so when good ACK from server come, wake up
// 5Jul       TBE      V2816    WAS: Found blocks locking up w/both GRN LED's on  IS: a) reverted to older good tested software and b) added COP watchdog timer = 1 sec timeout,
// 7JUL2016   TE       same     Added checksum to sending packet to enable higher quality comm. Bogus bytes were 'leaking' through                    
//  7/8/2016  TE        same    Changed checksum from unsigned 16 bit to float for better checksum accuracy
//  7/13/2016 TBE     no chg    Idea - not acted on. Change Vbalance from 4.11 to 4.00 or 4.05 to allow more room for balancing
//  11/4/2016 TBE     V4416     No changes except Server address shipped was 5,6,7,8 (4 ea 10 block servers), so next new number will be 9, unless maybe new customer gets 1 again?
/*

  Modules needed:
  1. Fan(s)
  2. Heaters-shunts
  3. radio comms
  4. Mode tracking and switching - note that switch to sleep mode is here
  4.5 Low power mode when in sleep mode
  6. Data acquistion and storage
  5. LEDS-Pots-switches

  Loop time: 50 msecs
*/

const uint16_t VERSION = 2816;   // 5215 = 52th week of 2015

#include "Arduino.h"

/*     Comms library from Radio Head ver 1.5.1 at http://www.airspayce.com/mikem/arduino/RadioHead/examples.html
  nrf24_reliable_datagram_client.pde
  Example sketch showing how to create a simple addressed, reliable messaging client
  with the RHReliableDatagram class, using the RH_NRF24 driver to control a NRF24 radio.
  It is designed to work with the other example nrf24_reliable_datagram_server
  Tested wirh Addicore nRF24L01+ module */


#include <RHReliableDatagram.h>
#include <RH_NRF24.h>
#include <SPI.h>

#define CLIENT_ADDRESS 9      // BLOCK number ALWAYS start from 1 (NOT 0) 255 MAX!!
//#define CLIENT_ADDRESS 1
#define SERVER_ADDRESS   1     // PACK SUPERVISOR CHANNEL
//#define SERVER_ADDRESS RH_BROADCAST_ADDRESS // If the destination address is the broadcast address RH_BROADCAST_ADDRESS (255), the message will
/// be sent as a broadcast, but receiving nodes do not acknowledge, and sendtoWait() returns true immediately
/// without waiting for any acknowledgements.
// Singleton instance of the radio driver
RH_NRF24 driver(9, 10); // define SPI pins for Teensy LC/ 3.1
// Class to manage message delivery and receipt, using the driver declared above
RHReliableDatagram manager(driver, CLIENT_ADDRESS);




// use Duffs latest snooze library version 5.5.0 https://github.com/duff2013/Snooze
#include <Snooze.h>   // used for deep sleep mode for IDD of around 150uA for (Teensy LC).
// must be global
SnoozeBlock config;

//#include "fans.c"
//#include "Heaters-Shunts"
//#include "Radio_Comms"
//#include "Mode_Tracking"
//#include "DAQ_&_Storage"
//#include "LEDs"
//#include "POTs"
//#include "Switches"

#define GLOW (LOW)      // define ON for miswired common anode LED's
#define DARK (HIGH)      // define OFF for miswired common anode LED's


// declare BlockMode types
const byte SLEEPING = 0;
const byte PAUSING = 5;
const byte CHARGING = 10;
const byte DISCHARGING = 15;
const byte FAULTSHUTDOWN = 20;
const byte MODE0 = 25;     // temporary rename when have english names
const byte MODE1 = 30;
const byte MODE2 = 35;
const byte MODE3 = 40;
const byte MODE4 = 45;


//declare FaultMode types    // faults that may cause shutdown or current limit reduction
const byte NOFAULT = 0;     // fault is cleared
const byte OVERTEMP = 1;
const byte UNDERTEMP = 5;
const byte OVERVOLT_BLOCK = 10;
const byte OVERVOLT_CELL = 15;
const byte UNDERVOLT_BLOCK = 20;
const byte UNDERVOLT_CELL = 25;
const byte DEADV_BLOCK = 30;
const byte DEADV_CELL = 35;
const byte FAULTMODE0 = 40;     // temporary... rename when have english names
const byte FAULTMODE1 = 45;
const byte FAULTMODE2 = 50;
const byte FAULTMODE3 = 55;

// declare WarningMode types  == warnings that will not cause shutdown or current reductions
const byte W_OPEN_NTC  = 200;
const byte W_SHORTED_NTC  = 205;  // crushed pack?


#define ON (1)
#define OFF (0)
const bool YES = 1;
const bool NO = 0;

// turn debugging feature on or off. debug = YES for production test or debug, NO for Production
bool debugg = NO;
//bool debugg = YES;
bool debugsleep = NO;
//bool debugsleep = YES;




//#define VREF (3.266)         // ADC reference voltage
//#define VINPUT (2.171)       // ADC input voltage from resistive divider to VREF
//#define ADCMAX (65535)       // maximum possible reading from ADC
//#define EXPECTED (ADCMAX*(VINPUT/VREF))     // expected ADC reading
#define SAMPLES (200)      // how many samples to use for ADC read - 200 seemed to work best
#define ADC_RESOLUTION (12)  // ADC resolution in bits, usable from 10-16 on this chip

// cell types
const byte  LG_MH1 = 0;          // 3200mah
const byte  SANYO_NCR18650B = 5;  // 3400mah
const byte  CELLTYPE0 = 10;        //
const byte  CELLTYPE1 = 15;        //
const byte  CELLTYPE2 = 20;        //


// Declare output pins
const byte LFANCNTL = 3;
const byte HFANCNTL = 4 ;
const byte LSHUNTCNTL = 5;
const byte HSHUNTCNTL = 6;

const byte LED2red = 7;
const byte LED2green = 8;
const byte LED1red = 2;
const byte LED1green = 13;      // do not use this in a program ...needs to be dedicated to SPI port after power up

//const byte led = 13; //temp use of led on teensy

// Declare input pins
const byte LNTC4 = A0;      // port 14= low side (3.7V) NTC4
const byte LNTC3 = A2;      // port 16
const byte LNTC2 = A7;      // port 21
const byte LNTC1 = A5;      //port 19

// new thermistors added Nov 2015
const byte HNTC3 = A1;      // port 15    = high side (7.4V) NTC3
const byte HNTC2 = A6;      // port 20
const byte HNTC1 = A4;        // port 18

const byte NTCambient = A3;      // port 17

const byte SCALED3pt7V = A8;      // port 22
const byte SCALED7pt4V = A9;      // port 23
const byte VBALANCE = A10;      // port 24 = POT
const byte SCALEDAMPS = A11;      // port 25

// declare default cell vars:
float Lowest_CellV;      // value of lowest cell of 2s block
float Highest_CellV;      // value of highest cell of 2s block

float Vcell_HVDO_Spec = 4.4;	// threshold where alarm is set and LED goes red.
float Vcell_Balance = 4.10;      // voltage above where Cells will balance
float Vcell_Nominal_Spec = 3.7;
float Vcell_Low_DK;       // close to threshold where fans and heaters are turned off to save power except bottom balance shunt
float Vcell_Low_Spec;    // threshold where sleep modes and software LVD happens
float Vcell_LVDO_Spec = 2.25;  	// threshold where all electronics turns off......about 2VPC
// cell temperature vars
//const int Tcell_SMOKE = 95;  // At 95C and above there may be smoke ...danger shut off all fans and electronics
const int NTC_SMOKE = 111;  // ADC counts for 100C
const int NTC_HOT = 416;  // ADC counts for 60C
const int NTC_PHOENIX = 590;   // ADC counts for 50C, reasoning is that at 117F in Phoenix we do not want fan coming on (50 = 120F)
const int NTC_WARM = 987;  // ADC counts for 35C
const int NTC_AMBIENT = 1365;  // ADC counts for 25C
const int NTC_COLD = 3000;  // ADC counts for -10C
const int NTC_SHORTED = 20; // ADC counts for 140C


//const int Tcell_hot = 60;  // Data sheet says 60C max temperature
//const int Tcell_warm = 38;  // Warm cells at 38C and up
//const int Tcell_cold = -10;  // Coldest cell spec = use heaters t50o warm
const int OPEN_NTC = 4000;  // open NTC (or colder than -40C which 3910 counts

// declare default cell vars:

//float Vblock  ; // block voltage
float CellV_Hiside ;  //hi side cell voltage
float CellV_Loside  ;  //low side cell voltage
float    Vbal ;    // balance pot voltage
float  Vcellamps  ;
float Hist_Hiside_CellV ;  // T_MODECHECK secs historical high cell voltage
float Hist_Loside_CellV  ;    // T_MODECHECK secs historical low cell voltage
int Temp ;     // used for temporary storage
int Temp1;
int Temp2;


// software real time clock vars
unsigned long currentmicros = 0;
unsigned long nextmicros = 0;
unsigned long interval = 1000278UL; // about 1000000 uS (increase constant to slow clock)
int seconds = 0;    // at reset, set clock to 0:00:00
int minutes = 0;
int hours = 0;    // at reset, set clock to 0:00:00
int ModeTimer = 0;    // use for off timer = 10 mins get set later
//byte T_MODECHECK = 10;    // update historical vars every minute for a 10 min running average
byte T_MODECHECK = 10;    // update historical vars every minute for a 10 min running average
const int T_HISTORYCHECK = 60;    // update historical vars every n secs
int HistoryTimer = T_HISTORYCHECK;


// declare global var
uint8_t  BADFLAG = 0;     // debug testing send data
int NTCchosen;            // choose which NTC input to test
int sensorValue = 0;        // value read from the ADC input

byte Cell_Type = 0;      // default to LG MH1 - change this if compliling for Sanyo or other cells
byte BlockMode = 0;     // default mode is sleeping
byte FaultMode = 0;    // default to no faults
byte WarningMode = 0;  // default no warnings

// temperature params
//short Tcells = 25;         // Fan(s) turn on when Tcells above 38C, AND below 100C if charging or discharging (2 byte unsigned)
short Tcells = 25;      // temporarily set all temps to 25C
short Thottest;
short  Tcoldest;
short Tambient = 25;

// fan params
byte LoFan = OFF;    // default to both fans off
byte HiFan = OFF;

// heater params      default to heater/balance resistors off
byte LoShunt = OFF;
byte HiShunt = OFF;

/********************************************************
  Set Low Power Timer for deepsleep mode wake up vars and config below
********************************************************/
const int TIMEBETWEENSLEEP = 4;    // n seconds between deepsleep events (4 sec between hibernate session if sleepmode, or 80 sec if chg/disch
//const int TIMEBETWEENSLEEP = 1;    // n seconds between deepsleep events
int DpSleep_timer = TIMEBETWEENSLEEP;

/***********************************************************
     Set one second (longest and default time with lib) watchdog enable  by redefining the startup_early_hook which by default
     disables the COP (TURN OFF WHEN DEBUGGING). Must be before void setup();
 *************************************************************/
#ifdef __cplusplus
extern "C" {
#endif
void startup_early_hook() {
  // empty
}
#ifdef __cplusplus
}
#endif





void setup() {    // ==============================================================

  // Deep sleep time setup
  config.setTimer(64000);// number of milliseconds, = 64 seconds of deep sleep
  /********************************************************
       Define digital pins for waking the teensy up. This combines pinMode and attachInterrupt in one function.
       Teensy 3.x =  Digtal pins: 2,4,6,7,9,10,11,13,16,21,22,26,30,33

       Teensy LC = Digtal pins: 2,6,7,9,10,11,16,21,22
       Configure the pin like the normal pinMode and append the interrupt type for the third parameter.
     ********************************************************/


  // nRF24 comms setup
  Serial.begin(9600);
  delay(700);   // delay for serial debug comm
  WatchdogReset();  // reset the watchdog timer
  delay(700);   // delay for serial debug comm
  WatchdogReset();  // reset the watchdog timer
  delay(700);   // delay for serial debug comm
  WatchdogReset();  // reset the watchdog timer

  // initialize Rf24 chip for 2.4Mhz comms
  if (!manager.init()) Serial.println("init failed");
  // Defaults after init are 2.402 GHz (channel 2), 2Mbps, 0dBm
  //if (!manager.setChannel(1)) Serial.println("setChannel failed");
  //if (!manager.setRF(RH_NRF24::DataRate2Mbps, RH_NRF24::TransmitPower0dBm)) Serial.println("setRF failed");

  CellV_Hiside = CellV_Loside = Hist_Hiside_CellV = Hist_Loside_CellV = 3.70;  // make all Vcell vars = nominal cell voltage

  WatchdogReset();  // reset the watchdog timer


  if (Cell_Type == LG_MH1)  {  //cell parameters for LG MH1 3200mah
    Vcell_HVDO_Spec = 4.25;
    Vcell_Balance = 4.11;            // top balance point
    Vcell_Nominal_Spec = 3.67;
    Vcell_Low_DK = 3.0;           // voltage where bottom balancing is started
    Vcell_Low_Spec = 2.9;
    Vcell_LVDO_Spec = 2.9;

  }
  else {                        // else choose Panasonic/Sanyo NCR18650B
    Vcell_HVDO_Spec = 4.5;
    Vcell_Nominal_Spec = 3.7;
    Vcell_Low_Spec = 2.5;
    Vcell_LVDO_Spec = 2.0;
    Vcell_Balance = 3.9;
  }



  NTCchosen = NTCambient; // start with A0
  // FANs
  pinMode(HFANCNTL, OUTPUT);      // enable digital output for turning on fan on board
  pinMode(LFANCNTL, OUTPUT);      // enable digital output for turning on fan on board

  // Heaters
  pinMode(LSHUNTCNTL, OUTPUT);      // enable digital output for turning on low  shunt
  pinMode(HSHUNTCNTL, OUTPUT);      // enable digital output for turning on high shunt


  // analog I/O
  pinMode(NTCambient, INPUT);      //
  analogReference(EXTERNAL);  // set analog reference to external ref
  //  analogReference(INTERNAL);  // set analog reference to internal ref
  analogReadRes(ADC_RESOLUTION);          // Teensy LC set ADC resolution to this many bits


  // LEDs
  // pinMode(LED1green, OUTPUT);      // enable digital output for turning on LED indicator
  pinMode(LED1red, OUTPUT);      // enable digital output for turning on LED indicator
  pinMode(LED2green, OUTPUT);      // enable digital output for turning on LED indicator
  pinMode(LED2red, OUTPUT);      // enable digital output for turning on LED indicator


  Serial.println("1: Green LED1 ");

  WatchdogReset();  // reset the watchdog timer

  // digitalWrite(LED1green, GLOW);   delay(1000);  // LED on for 1 second
  Serial.println("1: then go RED ");
  //digitalWrite(LED1green, DARK);
  digitalWrite(LED1red, GLOW);   delay(800);  // LED on for 1 second
  WatchdogReset();  // reset the watchdog timer

  Serial.println("2: Green LED2");
  digitalWrite(LED2green, GLOW);   delay(800);  // LED on for 1 second
  WatchdogReset();  // reset the watchdog timer

  Serial.println("3: Now go RED ");
  digitalWrite(LED2red, GLOW);   delay(800);  // LED on for 1 second
  WatchdogReset();  // reset the watchdog timer

  digitalWrite(LED2red, GLOW);   delay(800);  // LED on for 1 second
  digitalWrite(LED2green, DARK);
  // wait for slow human to get serial capture running
  // digitalWrite(LED1green, DARK); //leds all off
  digitalWrite(LED1red, DARK);
  digitalWrite(LED2green, DARK);
  digitalWrite(LED2red, DARK);
  Serial.println("4:Now test NTCs");
  WatchdogReset();  // reset the watchdog timer

  // default RF24 manager settings are 2000ms timeout and 3 retries. Timeout can be 2x to avoid collisions. These are too long for COP, and we don't want
  // COP reset in comm routine in case the program hangs in that code... (*just* barely works at 110ms/3 retries, and 50ms/9)
  //manager.setTimeout(60);
  //manager.setRetries(4);
  manager.setTimeout(100);      // can be about 200msec
  manager.setRetries(2);        // 200msec x 2 - 400msec max seems to work well,660msec is too much.


  Serial.println("Main Loop Start: ");
} // ==== end setup() ===========


void WatchdogReset (void) {     // use COP registers on the Teensy LC
  SIM_SRVCOP = 0x55;
  SIM_SRVCOP = 0xAA;
}


void loop() {  // ================================================================

  //startup:
  WatchdogReset();  // reset the watchdog timer

  // make clock tick tock
  currentmicros = micros(); // read the time.

  // when 1000 millisecond clock ticks, run real time clock
  if ((currentmicros - nextmicros) >= interval)  // if 1000000 microseconds have gone by...
  {
    nextmicros = nextmicros + interval; // update for the next comparison
    seconds = seconds + 1;              // and run real time clock
    if (DpSleep_timer > 0) DpSleep_timer --;    // run deepsleep timer every second
    if (HistoryTimer > 0) HistoryTimer --;    // run history timer every second

    if (seconds == 60)
    {
      seconds = 0;
      minutes = minutes + 1;
      if (ModeTimer > 0) ModeTimer = ModeTimer - 1;   // run mode changer timer
      if (minutes == 60)
      {
        minutes = 0;
        hours = hours + 1;
      }
      if (hours == 24) hours = 0;
    }
    Serial.println();
    Serial.print("Secs = ");  Serial.println(seconds);          // debug printout to pc (open serial monitor window)
    Serial.print("Mins = ");  Serial.println(minutes);
    Serial.print("Hrs = ");  Serial.println(hours);
    Serial.print("ModeTimer = ");  Serial.println(ModeTimer);
    Serial.print("DpSleep_timer = ");  Serial.println(DpSleep_timer);
    Serial.print("HistoryTimer = ");  Serial.println(HistoryTimer);
  }

  Serial.print("Firmware Ver: ");  Serial.println(VERSION);
  Serial.print("Client Address: ");  Serial.println(CLIENT_ADDRESS);


  digitalWrite(LED2green, GLOW); // led blink for adc sample

  // get cell voltages - test both hi and low voltage in block
  const byte NUMBVINs = (A8 + 4);          // 4 voltage inputs total in block

  for (int vinsnum = A8; vinsnum < NUMBVINs; vinsnum++) {
    float datSum = 0;  // reset our accumulated sum of input values to zero
    long n = 0;            // count of how many readings so far
    double x = 0;
    float Volts_LosideRaw ;
    float Tempv;
    
    for (int i = 0; i < SAMPLES; i++) {
      x = analogRead(vinsnum);      // take the analog sample

      if ((i < 10 ) || (i > 150)) goto throwaway_v;  // throw away the first 10 and last 50 samples

      datSum += x;        // sum data
      n++;
throwaway_v:       ;

    } //end Vin sample loop

    int datAvg = (datSum) / n;  // find the mean
    float Volts;

    // get scaled voltage value then ...
    // ...save voltage channels
    if (vinsnum == SCALED3pt7V) {
      //  Volts = ((datAvg*3.3)/4096) * 2.05;    // 3.7V low side input = prototypes
      // Volts = ((datAvg*3.3)/4096) * 2.015;    // 3.7V low side input - calibrate 3.7V low side first
      // Volts = ((datAvg*3.3)/4096) * 2.037;    // 3.7V low side input - calibrate 3.7V low side first
      // Volts = ((datAvg*3.3)/4096) * 2.018;    // 3.7V low side input - calibrate 3.7V low side first
      Volts = ((datAvg * 3.3) / 4096) * 2.006; // JUL 2016 3.7V low side input - calibrate 3.7V low side first - after 1uf cap added


      Volts_LosideRaw = Volts;   // save raw unaveraged signal for 7.4V math later...

      Tempv = Volts + CellV_Loside + CellV_Loside;
      //Volts = Volts + CellV_Loside + CellV_Loside;
      Volts = Tempv;
      CellV_Loside = Volts / 3.00;    // Use running average

      Volts = CellV_Loside;
      Serial.print("  3.7V lo side cell VDC  ");
    }


    if (vinsnum == SCALED7pt4V) {  // now read whole block voltage ~ 7.4V nominal
      //Volts = ((datAvg*3.3)/4096) * 3.675;    // 7.4V high side input = prototypes
      //Volts = ((datAvg*3.3)/4096) * 3.600;    // 7.4V high side input
      //Volts = ((datAvg*3.3)/4096) * 3.601;    // calibrate 7.4V high side input after low side input
      //Volts = ((datAvg*3.3)/4096) * 3.602;    // calibrate 7.4V high side input after low side input
      Volts = ((datAvg * 3.3) / 4096) * 3.564; // calibrate 7.4V high side input after low side input - after 1uf cap added

      Serial.print("  7.4VDC block  ");     Serial.println(Volts, 3);  // Show block voltage before cell math

      // problem here is you have 7.4V but you need top cell voltage which is 3.7V nom, so top cell VDC = block -lo side cell
      Volts = Volts - Volts_LosideRaw;        // Do math to get high side cell voltage of 3.7V nom

      Volts = Volts + CellV_Hiside + CellV_Hiside;
      Volts = Volts / 3.00;
      // now do fine calibration on signal
      // Volts = Volts - 0.010;    // add offset to signal

      CellV_Hiside =  Volts;
      Serial.print("  3.7V hi side cell VDC  ");
    }

    digitalWrite(LED2green, DARK); // end of led blink



    if (vinsnum == VBALANCE) {
      Volts = ((datAvg * 3.3) / 4096); // this works for no scaling like with the pot
      Vbal = Volts;
      Serial.print("  Balance pot  ");
    }
    if (vinsnum == SCALEDAMPS) {
      Volts = ((datAvg * 3.3) / 4096); // this works for no scaling like with the pot
      Vcellamps = Volts;
      Serial.print("  Block amps  ");
    }


    Serial.print("(") ;
    Serial.print(vinsnum);
    Serial.print(") Avg value: ");  Serial.print(Volts, 3);
    Serial.println ();


  }
  Serial.println ();


  WatchdogReset();  // reset the watchdog timer


  // get cell and ambient temperatures
  //test all NTCs in one loop
  // byte ntcnum = A0;    // Start with A0 and go to A7 to get all 8 NTCs
  const byte NUMBNTCs = (A0 + 8);          // 8 NTC's total in block

  for (int ntcnum = A0; ntcnum < NUMBNTCs; ntcnum++) {
    float datSum = 0;  // reset our accumulated sum of input values to zero
    long n = 0;            // count of how many readings so far
    double x = 0;

    for (int i = 0; i < SAMPLES; i++) {
      x = analogRead(ntcnum);
      if ((i < 10 ) || (i > 150)) goto throwaway_t;  // throw away the first 10 and last 50 samples

      datSum += x;        // sum data
      n++;
throwaway_t:      ;
    } //end ntc sample loop

    //float datAvg = (1.0 * datSum) / n;  // find the mean
    int datAvg = (datSum) / n;  // find the mean

    Serial.print(ntcnum);
    Serial.print("NTC avg value: ");  Serial.print(datAvg); if (datAvg < NTC_SHORTED)Serial.print("---->>> NTC shorted ");



    // filter for open (> 4000) and short NTC
    if (datAvg > OPEN_NTC) datAvg = Tcoldest;    // if open ntc, ignore sample


    // find highest and lowest temperature
    if (ntcnum == A0) Tcoldest = Thottest = datAvg;    // first pass set hi and low to current sample
    if (datAvg > Tcoldest) Tcoldest = datAvg;    // save coldest NTC
    else if (datAvg < Thottest) Thottest = datAvg;  // else save hottest


    // save ambient temp
    if (ntcnum == NTCambient) {
      Tambient = datAvg;
      Serial.print("  Tambient");
    }
    Serial.println ();

  } // end all ntc tested loop

  Serial.print("NTC coldest: ");     Serial.print(Tcoldest);
  Serial.print("     NTC hottest: ");     Serial.print(Thottest);
  Serial.println ();
  Serial.println ();


  WatchdogReset();  // reset the watchdog timer

  //   Mode_Tracking();
  const float DELTAV = 0.010;    // was 15mv Jul4,2016
  {
    //...find what the block is doing by using cell voltage to determine charge or discharge
    if (((CellV_Hiside - DELTAV) > Hist_Hiside_CellV) || ((CellV_Loside - DELTAV) > Hist_Loside_CellV) )
      Temp = CHARGING;      // if new sample value is higher than old, we are charging

    else if ((CellV_Hiside + DELTAV < Hist_Hiside_CellV) || (CellV_Loside + DELTAV < Hist_Loside_CellV))
      Temp = DISCHARGING;   // else if more than 50mv less, we are discharging

    else if ((CellV_Hiside > Vcell_Balance) || (CellV_Loside > Vcell_Balance))  Temp = CHARGING;      // Voltage above nominal, must be charging
    // Voltage below nominal, could be going into deep discharge WATCH OUT!!!
    //else if ((CellV_Hiside < (Vcell_Low_Spec + 0.015)) || (CellV_Loside < (Vcell_Low_Spec + 0.015)))  Temp = DISCHARGING;
    // Voltage below nominal, could be going into deep discharge WATCH OUT!!!

    else  Temp = SLEEPING;                            // not charging not discharging must be sleeping

    // when Vcell falls off fast for self discharge, it must not look like discharge
    //if ((CellV_Hiside < (Vcell_Low_Spec + 0.45)) || (CellV_Loside < (Vcell_Low_Spec + 0.45)))  Temp = SLEEPING;
    if ((CellV_Hiside < (Vcell_Low_Spec + 0.15)) || (CellV_Loside < (Vcell_Low_Spec + 0.15)))  Temp = SLEEPING;

    BlockMode = Temp;

    Serial.print("...... Historical high side voltage = ");  Serial.print(Hist_Hiside_CellV, 3); Serial.print("    ");
    Serial.print(CellV_Hiside, 3); Serial.print(" = Hiside cell voltage  ");    Serial.println ();

    Serial.print("...... Historical low side voltage = ");  Serial.print(Hist_Loside_CellV, 3); Serial.print("    ");
    Serial.print(CellV_Loside, 3);  Serial.print(" = Loside cell voltage  ");    Serial.println ();



    if (ModeTimer == 0) {        // update vars every minute for a 10 min running avg for mode checks
      ModeTimer = T_MODECHECK;  // set timer for next check
    }

    Serial.println(); Serial.print("........... Block Mode is = ");  Serial.print(BlockMode); Serial.println();
    //Serial.println(); Serial.print("----------------------------------------"); Serial.println();
  }

  if (HistoryTimer == 0) {
    HistoryTimer = T_HISTORYCHECK;    // set timer for next check
    //take a voltage sample every minute and have running average over last 10 min
    Hist_Hiside_CellV = (CellV_Hiside + Hist_Hiside_CellV + Hist_Hiside_CellV + Hist_Hiside_CellV + Hist_Hiside_CellV + Hist_Hiside_CellV + Hist_Hiside_CellV + Hist_Hiside_CellV + Hist_Hiside_CellV + Hist_Hiside_CellV + Hist_Hiside_CellV) / 11;        // load historical vars with current values
    Hist_Loside_CellV = (CellV_Loside + Hist_Loside_CellV + Hist_Loside_CellV + Hist_Loside_CellV + Hist_Loside_CellV + Hist_Loside_CellV + Hist_Loside_CellV + Hist_Loside_CellV + Hist_Loside_CellV + Hist_Loside_CellV + Hist_Loside_CellV) / 11;
  }

  // reset historical vars at first power up for first 30 seconds
  if ((seconds < 20) && (minutes == 0) && (hours == 0) and (ModeTimer == T_MODECHECK)) {
    Hist_Hiside_CellV = CellV_Hiside;        // load historical vars with current values
    Hist_Loside_CellV = CellV_Loside;
  }



  if (BlockMode == SLEEPING) {
    digitalWrite(LED2green, DARK);
    digitalWrite(LED2red, DARK);
    digitalWrite(LED1red, DARK);
  }
  else {          // not sleeping
    DpSleep_timer = TIMEBETWEENSLEEP * 20;  // reset timer if not in sleepmode to 80 sec (or 4 sec if sleeping)
  }



  // If product test:
  if (debugg == YES) {
    if (seconds < 30) {
      //    LoFan = ON;
      //   HiFan = ON;
      HiShunt = OFF;
      LoShunt = OFF;
    }
    else {
      LoFan = OFF;
      HiFan = OFF;
      //   HiShunt = ON;
      //   LoShunt = ON;
    }
    ModeTimer = 0;     // might need this for actual production unit to load values in all historical vars
    goto endoftest;
  }


  // Fan logic
  // Fan(s) turn on when Tcells above 40C, AND below 100C, if charging or discharging.
  LoFan = OFF;
  HiFan = OFF;
  if ((BlockMode == CHARGING) || (BlockMode == DISCHARGING))
  {
    if ((Thottest < NTC_WARM) && (Thottest > NTC_SMOKE)) // if chg/discharging, AND getting warmer AND not too hot...
    {
      if ((CellV_Hiside > Vcell_Low_Spec) &&  (CellV_Loside > Vcell_Low_Spec))  // AND not low cell voltage, turn on both fans
      {
        LoFan = ON;
        HiFan = ON;
      }
    }
  }
  else 
  {
    if (Thottest < NTC_PHOENIX) { //blocks might be sitting in very hot place, so check and run fans if so
      LoFan = ON;
      HiFan = ON;
    }
  }


  // Balance and heating resistors logic
  // Cells go into balancing above Vcell_nominal if charging
  LoShunt = OFF;
  HiShunt = OFF;
  if (CellV_Loside > Vcell_Balance) LoShunt = ON;
  if (CellV_Hiside > Vcell_Balance) HiShunt = ON;

  if ((HiShunt == ON) || (LoShunt == ON)) {
    digitalWrite(LED2green, GLOW);    // light green LED and...
    digitalWrite(LED2red, GLOW);    // light red LED if balancing
    if ((HiShunt == ON) && (LoShunt == ON)) {   // both shunts need to be on for FANs to be on
      LoFan = ON;      HiFan = ON;
    }
  }
  else {
    digitalWrite(LED2green, DARK);
    digitalWrite(LED2red, DARK);
  }

  // another reason for operating the shunts is bottom balance, so if one cell is at 3.2 and the other at 1.8 they willtry  be balanced
  if ((CellV_Loside < CellV_Hiside) && (CellV_Loside < Vcell_Low_DK))
  {
    if (CellV_Hiside > Vcell_Low_Spec)  HiShunt = ON;     // bott balance till Low V spec
    Serial.print("High side bottom balance");
  }
  if ((CellV_Hiside < CellV_Loside) && (CellV_Hiside < Vcell_Low_DK))
  {
    if (CellV_Loside > Vcell_Low_Spec) LoShunt = ON;
    Serial.print("Low side bottom balance");
  }


endoftest:

  // switch fans on/off (later come back and PWM for better quieter cooling
  if (HiFan == ON) digitalWrite(HFANCNTL, HIGH);     // high side fan on
  else  if (ModeTimer < 2) digitalWrite(HFANCNTL, LOW);                   // or not

  if (LoFan == ON) digitalWrite(LFANCNTL, HIGH);     // low side fan on
  else if (ModeTimer < 2) digitalWrite(LFANCNTL, LOW);

  // switch heaters on/off
  if ( HiShunt == ON) digitalWrite(HSHUNTCNTL, HIGH);     // high heater on
  else digitalWrite(HSHUNTCNTL, LOW);                      // else not

  if ( LoShunt == ON) digitalWrite(LSHUNTCNTL, HIGH);     // low heater on
  else digitalWrite(LSHUNTCNTL, LOW);                      // else not



  Serial.println ();

  Serial.print("........... High side fan is: "); Serial.print(HiFan);
  Serial.println ();
  Serial.print("........... Low side fan is: "); Serial.print(LoFan);
  Serial.println ();
  Serial.print("........... High side heater is: "); Serial.print(HiShunt);
  Serial.println ();
  Serial.print("........... Low side heater is: "); Serial.print(LoShunt);
  Serial.println ();


  Serial.println(); Serial.print("----------------------------------------"); Serial.println();
  Serial.println(); Serial.print("----------------------------------------"); Serial.println();


  WatchdogReset();  // reset the watchdog timer


  if (debugsleep == NO) {
    // if mode in sleep, turn all perpherals off and put micro to deep sleep
    /********************************************************
      feed the sleep function its wakeup parameters. Then go to deepSleep.
    ********************************************************/
    if ((BlockMode == SLEEPING) && (DpSleep_timer == 0)) {
      DpSleep_timer = TIMEBETWEENSLEEP;    // set the time between deepsleep events
      // Set all I/O and registers to min current settings
      digitalWrite(HFANCNTL, LOW);       //  fans and shunts off
      digitalWrite(LFANCNTL, LOW);
      digitalWrite(HSHUNTCNTL, LOW);
      digitalWrite(LSHUNTCNTL, LOW);

      digitalWrite(LED1red, DARK);
      digitalWrite(LED2green, DARK);
      digitalWrite(LED2red, DARK);

      pinMode(LED1green, INPUT);        // put spi port to sleep and turn off led
      void CPU(uint32_t TWO_MHZ);    // slow CPU clock
      int What_INT = 0;

      // if sleepmode, just go to deepsleep, except if either cell is low, hibernate until charge > 3.2V/cell
      // if ((CellV_Hiside < (Vcell_Low_Spec)) ||  (CellV_Loside < Vcell_Low_Spec)) {
      if ((Hist_Hiside_CellV < (Vcell_Low_Spec + 0.35)) ||  (Hist_Loside_CellV < Vcell_Low_Spec + 0.35)) {
        // must hibernate for low current   (tried 7.4V input, but that is not compatible with a deep sleep or hibernate library)
        config.pinMode(22, INPUT, RISING);     // make 3.7V sense input into a digital interrupt pin, rising edge trigger (1.60V @pin)
        What_INT = Snooze.hibernate( config ); // put into hibernate and then return module that woke processor
        for ( ; What_INT != 22; ) {  // loop until input trigger wakes up block
          What_INT = Snooze.hibernate( config ); // put into hibernate and then return module that woke processor
        }
      }
      else  {
        What_INT = Snooze.hibernate( config );// not low cell V so go hibernate for 60 seconds and return module that woke processor
        // Deep sleep/hibernate modes = 700ua with nRF24 and no comms
      }



      // Reset all I/O and registers to min current settings
      //SPI.begin();      // re-init all SPI registers



      Serial.begin(9600);

      digitalWrite(LED2green, GLOW); // green LED on
      pinMode(LED2green, OUTPUT);
      pinMode(NTCambient, INPUT); // now coming out of sleep...

      Serial.println(); Serial.print("----------------------------------------"); Serial.println();
      Serial.println(); Serial.print("----------------------------------------"); Serial.println();
      Serial.println(); Serial.print("----------------------------------------"); Serial.println();
      Serial.println(); Serial.print("----------------------------------------"); Serial.println();
      Serial.println(); Serial.print("----------------------------------------"); Serial.println();
      Serial.println(); Serial.print("----------------------------------------"); Serial.println();
      Serial.println(); Serial.print("----------------------------------------"); Serial.println();
      Serial.println(); Serial.print("----------------------------------------"); Serial.println();


      if (What_INT == 36)   {
        Serial.print("..WAKE UP is from Low Power Timer : "); Serial.print(What_INT);
      }
      else   {  //input from pin wakeup (22)
        Serial.print("..WAKE UP is from 3.7V interrupt input pin at 3.2V : "); Serial.print(What_INT);
      }

    }
  }

  //WatchdogReset();  // reset the watchdog timer

  uint16_t Commdelay = 500;      // 500 msec delay if comms is ok, else reduce to get retry comms
  // Comms from client to server
  // send high andde cell voltage and...highest and lowest cell temp

  SPI.begin();      // re-init all SPI registers after sleep and before every transmission for industrial strength
  // send highest cell voltage if >3V, else lowest cell voltage
  // send highest cell temp unless ambient is < 25C then send lowest cell temp
  struct DATA {
    float sCellV_Hiside ;
    uint16_t sThottest;
    uint16_t sTcoldest;
    float sCellV_Loside;
    //
    float schecksum ;
  };
  DATA data;      // 14 byte struct (float is 4 bytes)


  Serial.print("Sending to nrf24_reliable_datagram_server:  ");
  Serial.print(data.sCellV_Hiside); Serial.print("   ");
  Serial.print(data.sThottest); Serial.print("   ");
  Serial.print(data.sTcoldest); Serial.print("   ");
  Serial.print(data.sCellV_Loside); Serial.print("   ");
  
  Serial.print(data.schecksum,3); Serial.print("   ");
  Serial.println(BADFLAG);

 // if (CLIENT_ADDRESS != 4) BADFLAG = 1;
  if ((data.sCellV_Hiside > 4.10) || (data.sCellV_Hiside < 3.9))BADFLAG = 1;
  if ((data.sThottest > 1500) || (data.sCellV_Hiside < 1200))BADFLAG = 2;
  if ((data.sTcoldest > 1500) || (data.sCellV_Hiside < 1200))BADFLAG = 3;
  if ((data.sCellV_Loside > 4.10) || (data.sCellV_Loside < 3.9)) BADFLAG = 4;


  data.sCellV_Hiside = CellV_Hiside ;
  data.sThottest = Thottest;
  data.sTcoldest = Tcoldest;
  data.sCellV_Loside = CellV_Loside ;
  data.schecksum = CellV_Hiside+Thottest+Tcoldest+CellV_Loside;    // compute checksum

  WatchdogReset();  // reset the watchdog timer

  // Dont put this on the stack:
 // uint8_t buf[RH_NRF24_MAX_MESSAGE_LEN];    //32 byes maximum RF24 minus 4 bytes of RadioHead headers = 28 bytes for me


  // Send a message to manager_server
  if (manager.sendtoWait((uint8_t*)&data, sizeof(data), SERVER_ADDRESS))
  {
     // turn off sleep mode...Pack Supervisor is receiving packets
      DpSleep_timer = TIMEBETWEENSLEEP * 30;  // restore sleep timer to 120 sec

     // TX was success, now wait for a reply from the server
    //  uint8_t len = sizeof(buf);
    //  uint8_t from;
    // if (manager.recvfromAckTimeout(buf, &len, 2000, &from))
    // {
    //   Serial.print("got reply from : 0x");
    //   Serial.print(from, HEX);
    //   Serial.print(": ");
    //   Serial.println((char*)buf);
    // }
    // else
    // {
    //   Serial.println("No reply, is nrf24_reliable_datagram_server running?");
    // }
  }
  else {
    //goto startup;
    Serial.println("sendtoWait failed");    // its' possible there was tx but no ACK
    // turn on red LED if no ACK from server
    digitalWrite(LED1red, GLOW);
    Commdelay = 0;
    manager.init(); // init once each pass in case of flakey hardware (if nRF24 disconnects, radio loses it's mind)
  }

  digitalWrite(LED1red, DARK);   // RED led off for low power
  pinMode(LED1green, INPUT);        //  turn off led for low power

  WatchdogReset();  // reset the watchdog timer
  //goto startup;


  // end of main loop now setup 50 msec loop (variable 1 sec +/- for now)
  delay(Commdelay + 200 + (CLIENT_ADDRESS * 10)) ; // be careful here...long delays will trip the COP watchdog timer
  //delay(CLIENT_ADDRESS*100);               // wait for a var msecs

} // end main()  =====================================================



// Power Usage Teensy LC with clock speed = 48Mhz:
// 1ua -  +Vbat supply power at Vin = up to 5V, quiesent divider and LDO supply
// 120ua when teensy is powered by USB, so that is dividers and 3.6V supply chip
// 6ma deep sleep w/ green led on
// 3.5ma deep sleep w/ red led on
// 623ua with no leds on no nRF24
//780ua hibernate mode with nRF24 installed but no comms
// 200ma FAN
// 207ma both shunts
// 14-16ma normal run mode with Leds, etc nRF24 installed but no comm,
// 16ma average (28ma pk) with limited TX only com, once per second
// Possible improvements = add MOSFET switches to REF and Vdividers, REF3033 = 50ua, AP2202 = 160ua, dividers = 80ua, where is the rest coming from?


//void soft_reboot() // Restarts program from beginning but does not reset the peripherals and registers
//{
//void(* soft_reboot) (void) = 0; //declare reset function @ address 0
//}

