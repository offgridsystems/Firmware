// DK Supervisor PCB production code for Teensy 3.1 3.2   7/8/15  T Economu 
// Used Arduino 1.6.6 for first Fido pilot run 5 packs or 50 pcs 
// Fido using Arduino IDE 1.8.4 with Teensyduino 1.39 (Switching to Sublime Text 3 and Stino Dev Plugin)
//
// Date       Who     Ver Date code   WAS/IS changes
//-------------------------------------------------------------------------------------------------
//
// 6/16/2016  TBE             Activate CONNECT input  to be a "LEARN BLOCKS" function
// 6/29/2016  TBE             Add 3-5VPMW for charge control on DAC2 output, 
//                            Add Limp Mode output to A4-NC testpoint (PIN19)
// 4/17/2017  TBE     1717    Added fix for 2-5V PWM DAC for charge control - not updating during charge 
//                            mode! (Sketch -> Export compiled binary and then Show Sketch folder)
// 7/07/2016  TBE     2116    Added checksum for data comms to ensure good quality data, improved
//                            bogus bytes. Also added COP watchdog timer       
// 7/11/2016  TBE     2816    Added limp mode for Cell temp >= 60C and open relay for >= 63C per Jeb     
// 7/13/2016  TBE     2816    Changed Vbalance from 4.11 to 4.10 for LG MJ1 cell, calibrated Vpack and
//                            Icharge. Also Need to order charger to test!
// 7/10/2017  TBE     2817    Keyword "Jul 12, 2017" Latched limp mode when SOC < 20%. Reset when
//                            Pack Supervisor power is cycled. Also was no SOCv averaging so fuel 
//                            gauge jumped around a lot. Averaged value so steadier .
// 7/24/2017  TBE     3017    SOCv sampled too early at startup and SOCv and TempV var not being 
//                            initialized properly. Insure no sampling of SOCv the first 10 seconds,
//                            and SOCv and TempV var now initialized to Vnominal. This will allow
//                            the rider to get a few seconds of motoring after startup no matter 
//                            the SOC condition. We may want to max of 5 restarts in limp mode.
// 9/11/2017  NHJ     3617    Switched to git for version control and collaboration
//                            Cleaned up and added comments 
//                            Added FlexCAN code with Extended support

 const uint16_t VERSION = 3617;   // 2817 = 28th week of 2017

//---------VERBOSE MODE? MAYBE YOU WANT DEBUG?-----------------------------------------------------
#define VERBOSE                   // All the org serial output
#define DEBUG                     // debug data serial output 
#define CANDEBUG                  // CAN specific data

// nRF24 2.4Mhz packet comms
// nrf24_reliable_datagram_server.pde
// -*- mode: C++ -*-
// Example sketch showing how to create a simple addressed, reliable messaging server
// with the RHReliableDatagram class, using the RH_NRF24 driver to control a NRF24 radio.
// It is designed to work with the other example nrf24_reliable_datagram_client

#include <RHReliableDatagram.h>     // comm routines
#include <RH_NRF24.h>               // Nordic nRF24L01+ drivers DOES THIS COME FROM RADIOHEAD OR ???
#include <SPI.h>                    // use SPI bus to comm with RF24 radio
#include <EEPROM.h>                 // use EEPROM to store Blocks connected to pack
#include <FlexCAN.h>                // CAN Bus connection
#include <VERBOSE.h> 

//---------SERVER ADDRESS--------------------------------------------------------------------------
// DO NOT! use 0 or 255! ---does not work!
#define SERVER_ADDRESS 5
// Singleton instance of the radio driver
//RH_NRF24 driver;
RH_NRF24 driver(9, 10); // for Teensy 3.x SS and CE lines
// Class to manage message delivery and receipt, using the driver declared above
RHReliableDatagram manager(driver, SERVER_ADDRESS);
// Dont put this on the stack:
uint8_t buf[RH_NRF24_MAX_MESSAGE_LEN];

struct DATA {
  float sCellV_Hiside;       // high voltage side cells of dkblock
  uint16_t sThottest;        // hottest NTC counts (lower is hotter)
  uint16_t sTcoldest;        // coldest NTC counts
  float sCellV_Loside ;      // Low voltage side cells
  float schecksum;           // checksum for data integrity
  //bool sCharge;
}; DATA data;

// nRF24 2.4Mhz packet comms

//---------GLOBAL VARAIBLES------------------------------------------------------------------------
//declare gMode vars
uint8_t   gMode = 0;                     // default to sleep at wakeup
const byte SLEEPMODE = 0;
const byte PAUSEMODE = 5;
const byte CHARGEMODE = 10;
const byte DRIVEMODE = 15;
const byte FAULTSHUTDOWN = 20;
const byte MODE0 = 25;                   // temporary rename when have English names
const byte MODE1 = 30;
const byte MODE2 = 35;
const byte MODE3 = 40;
const byte MODE4 = 45;

//declare gFaultMode types               // faults that may cause shutdown or current limit reduction
uint8_t  gFaultMode = 0;                 // default to no faults at wakeup
const byte NOFAULT = 0;                  // cleared fault
const byte OVERTEMP = 1;
const byte UNDERTEMP = 5;
const byte OVERVOLT_BLOCK = 10;
const byte OVERVOLT_CELL = 15;
const byte UNDERVOLT_BLOCK = 20;
const byte UNDERVOLT_CELL = 25;
const byte DEADV_BLOCK = 30;
const byte DEADV_CELL = 35;
const byte FAULTMODE0 = 40;              // temporary... rename when have English names
const byte FAULTMODE1 = 45;
const byte FAULTMODE2 = 50;
const byte FAULTMODE3 = 55;

// ---------CAN BUS VARIABLES----------------------------------------------------------------------
const uint16_t Tx_msg_interval = 1000;
elapsedMillis Tx_counter;
//static CAN_message_t rxMsg;
//static uint32_t BMS_ID = 0xF4;
const uint32_t BMS_TO_CHARGER = 0x1806E5F4;
//static uint32_t CHG_ID = 0xE5;
//const uint32_t CHG_BROADCAST = 0x18FF50E5;

//---------COMMS VARIABLES-------------------------------------------------------------------------
uint16_t CommsFaults = 0;
bool Disconnected_Block = 0;
uint16_t Disconnected_BlockNum = 0;      // BLock 0 not used - START at Block 1
float Highest_Vcell;
float Lowest_Vcell;
uint16_t Highest_Tcell;
uint16_t Lowest_Tcell;
float Temp1 ;
float Temp2 ;
uint16_t Temp3 ;
uint16_t Temp4 ;
float Hist_Highest_Vcell ;               // running average of ALL blocks highest cell voltage
float Hist_Lowest_Vcell ;
float Hist_Highest_Tcell;                // running ave of ALL blocks highest cell temperature
float Hist_Lowest_Tcell;

#define VREF (3.266)                     // ADC reference voltage (= power supply)
#define VINPUT (2.171)                   // ADC input voltage from resistive divider to VREF
#define ADCMAX (65535)                   // maximum possible reading from ADC
#define EXPECTED (ADCMAX*(VINPUT/VREF))  // expected ADC reading
#define SAMPLES (200)                    // how many samples to use for ADC read - 200 seemed to work best   
#define ADC_RESOLUTION (12)              // ADC resolution in bits, 10-13 on this chip
#define DAC_RESOLUTION (10)              // DAC resolution in bits, 0-12 on this chip (same setup as PWM outputs)

//---------OUTPUT PINS-----------------------------------------------------------------------------
// relays
const byte RELAYDR1 = 5;
const byte RELAYDR2 = 6;
//  const byte RELAYDR1 = 3;
//  const byte RELAYDR2 = 4;
//  const byte RELAYDR3 = 5;
//  const byte RELAYDR4 = 6;
uint16_t Mrelay_Cycles = 0;             // Motor relay cycle counter
uint16_t Crelay_Cycles = 0;             // Charger relay cycle counter

// Analog DAC
int DAC_OUT = A14;                      //use A14 as Analog Out (DAC)

// pwm for op amp digital dacs (note 10 bits PWM same setup as Analog DAC output)
const byte PWM1 = 20;
const byte PWM2 = 21;
const int PWMFREQ = 40000;
// const byte FULLPWMRANGE = 144;
// const byte FULLPWMRANGE = 122;
const int FULLPWMRANGE = 1000;

// application specific assignments
const byte LIMP_OUTPUT = 18;            // on off signal to motor control at 20% SOC
const byte CHARGER_CONTROL = PWM2;      // 0-2-5V output for charger control near balance
const byte CHARGER_RELAY = RELAYDR1;
const byte MTRCONTROL_RELAY = RELAYDR2;
const uint8_t No_Of_Cells = 20 ;              // default to 10 DKblock = 72V system for FIDO
const uint8_t NUMBER_OF_BLOCKS = (No_Of_Cells / 2);
const float VPACKNOMINAL = 72.0;        // Fido is 72V system

// LEARN BLOCKS
const byte LEARN_BLOCKS_IN = 2;         // input pin 2
const byte LEARN_TIMEOUT = 4;           // 4 minute learn timeout
uint8_t blockNum[NUMBER_OF_BLOCKS] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0}; // the array of 10 blocks, future user configurable
bool LearnBlockSwitch = 1;              // switch used to determine if block values should be zeroed

// block array for block 'awareness'
const uint8_t COMM_TIMEOUT = 255;       // max 4+ minute comm timeout
uint8_t Block_Comm_Timer[NUMBER_OF_BLOCKS] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

//---------CELL TYPES------------------------------------------------------------------------------
int Cell_Type = 1;                      // default to LG type
const byte  LG_MH1 = 0;                 // 3200mah
const byte  LG_MJ1 = 1;                 // 3500mah
const byte  SANYO_NCR18650B = 20;       // 3400mah
const byte  CELLTYPE0 = 40;
const byte  CELLTYPE1 = 60;
const byte  CELLTYPE2 = 80;

//---------LITHIUM ION CELL SPEC VARIABLES---------------------------------------------------------
// all variables are set in setup() based on cell type selection above
float Vcell_HVD_Spec = 4.4;	                // threshold where alarm is set and LED goes red.
float Vcell_Nominal_Spec;
float Vcell_Low_Spec;                       // threshold where fans and heaters are turned off to save power
float Vcell_LVD_Spec;  	                    // threshold where all electronics turns off......about 2VPC
float Vcell_Balance;                        // voltage above where Cells will balance
float Vcell_Trickle_Charge;                 // voltage under where trickle charge should happen
float Vcell_Bulk_Charge;                    // voltage under where the bulk of the charge should happen
float Vcell_Off_Charge;                     // voltage above where charger should turn off
const float CELL_RECONNECT_V = 4.000;       // reconnect charger at this (lowest) cell voltage

//---------CELL TEMP VARIABLES---------------------------------------------------------------------
// const int Tcell_SMOKE = 95;              // At 95C and above there may be smoke ...danger
const int NTC_SMOKE = 111;                  // ADC counts for 100C
const int NTC_63C = 390;                    // ADC counts for ~~63C
const int NTC_60C = 416;                    // ADC counts for 60C
const int NTC_WARM = 987;                   // ADC counts for 35C
const int NTC_AMBIENT = 1365;               // ADC counts for 25C
const int NTC_COLD = 3000;                  // ADC counts for -10C

//---------CHARGER VARIABLES-----------------------------------------------------------------------
const uint16_t Charge_Voltage = 830;        // max charge voltage 830 = 83.0V
const uint16_t Charge_Trickle_Current = 10; // trickle current 10 = 1.0A
const uint16_t Charge_Max_Current = 200;    // max charge current 200 = 20.0A
const uint16_t Charge_End_Current = 1;      // end charge current 1 = 0.1A
uint8_t Charge_status_flag = 0;             // flag for latching charger levels
const uint8_t TRICKLE = 0;
const uint8_t BULK = 1;
const uint8_t TOPEND = 2;
const uint8_t COMPLETE = 3;

//---------USER INPUT PINS-------------------------------------------------------------------------
const byte CHARGE_INPUT = 22;               // charger presence or absence
const byte KEYSWITCH_INPUT = 23;            // key switch on or off

//---------RELAY SETTINGS AND VARS-----------------------------------------------------------------
bool ChargeRelay = 0;
bool MtrControlRelay = 0;
// const float VPACK_HI_CHG_LIMIT = 60.0;   // do not allow charger to raise pack > 4.20V cell x 20 cells = 84VDC
// const float Vpack_HV_Run_Limit = 64.0;
// const float VPACK_LO_RUN_LIMIT = 52.0;   // do not allow to run pack below 2.80V per cell x 20 cells = 58V
const float VPACK_HI_CHG_LIMIT = 85.0;      // do not allow charger to raise pack > 4.25V cell x 20 cells = 85VDC
// const float Vpack_HV_Run_Limit = 90.0; 
// const float VPACK_LO_RUN_LIMIT = 58.0;   // do not allow to run pack below 2.80V per cell x 20 cells = 58V

/*
// pwm for op amp digital dacs (note 10 bits PWM same setup as Analog DAC output)
const byte PWM1 = 20;
const byte PWM2 = 21;
const int PWMFREQ = 40000;
// const byte FULLPWMRANGE = 144;
// const byte FULLPWMRANGE = 122;
const int FULLPWMRANGE = 1000;

// application specific assignments
const byte LIMP_OUTPUT = 18;            // on off signal to motor control at 20% SOC
const byte CHARGER_CONTROL = PWM2;      // 0-2-5V output for charger control near balance
                                        // (FIDO is 2-5V = 0-100% linear charge) 
const byte CHARGER_RELAY = RELAYDR1;
const byte MTRCONTROL_RELAY = RELAYDR2;
byte No_Of_Cells = 20 ;                 // default to 10 DKblock = 72V system for FIDO
const byte NUMBER_OF_BLOCKS = (No_Of_Cells / 2);
*/

//---------CURRENT CONTROL-------------------------------------------------------------------------
uint16_t gTempPot;                      // current potentiometer
float gAmps;                            // amps plus and minus through LEM sensor

//float gSOCv = Vnominal;               // init with a nominal value

//---------LED OUTPUT PINS-------------------------------------------------------------------------
const byte LED2red = 7;
const byte LED2green = 8;
const byte LED1red = 15;
const byte LED1green = 16;
// const int led = 13;                  //temp use of led on teensy (ALSO used as NTC4 input)

//---------SPI PORT--------------------------------------------------------------------------------
/*
  const byte SCLCK = 13;
  const byte MISO = 12;
  const byte MOSI = 11;
  const byte CE = 9;
*/

//---------INPUT PINS------------------------------------------------------------------------------
const int VSCALEDPACK = A0;             // port 14
const int NTCambient = A3;              // port 17
const int TPA4NC = A4;                  // port 18
const int IDISCHG = A5;                 // port 19
const int VBALANCE = A10;               // port 24 = POT
const int ICHG = A11;                   // port 25

//---------UNUSED PINS-----------------------------------------------------------------------------
const int UNUSEDA12 = A12;      
const int UNUSEDA13 = A13;      
const uint8_t UNUSEDA24 = 24;  
const uint8_t UNUSEDA25 = 25;  
const uint8_t UNUSEDA26 = 26;  
const uint8_t UNUSEDA27 = 27;  
const uint8_t UNUSEDA28 = 28;  
const uint8_t UNUSEDA29 = 29;  
const uint8_t UNUSEDA30 = 30;  
const uint8_t UNUSEDA31 = 31;  
const uint8_t UNUSEDA32 = 32;  
const uint8_t UNUSEDA33 = 33;  

//---------CURRRENT SENSOR VARIABLES---------------------------------------------------------------
// @ save offset from 2.50V at this address (0-4096 counts) - start with 2.500V exactly
// 3103
//const float EEPROM_CHG_SENSOR_OFFSET = 3094.8;  // see @
const float EEPROM_CHG_SENSOR_OFFSET = 3094.77;   // see @
uint16_t EEPROM_DISCH_SENSOR_OFFSET = 3103;       // see @

//---------DECLARE GLOBAL VARAIBLES----------------------------------------------------------------
const bool YES = 1;
const bool NO = 0;
const bool ON = 1;
const bool OFF = 0;
const float HYSTERESIS = 0.4;     // provide hysteresis to prevent relay chatter and oscillation
int sensorValue = 0;              // value read from the ADC input
//float Vnominal = 72.0;          // Nominal battery pack voltage at startup
float Vnominal = VPACKNOMINAL;
float Vpack = Vnominal;           // start with nominal battery voltage
float gSOCv = Vnominal;           // init with a nominal value
bool LEARNBLOCKS = OFF;           // Learn blocks is turned on by "CONNECT" switch
byte tempx = 0;

//---------SOFTWARE REAL TIME CLOCK VARIABLES------------------------------------------------------
unsigned long currentmicros = 0;
unsigned long nextmicros = 0;
unsigned long interval = 1000278UL;   // about 1000000 uS (increase constant to slow clock)
int seconds = 0;                      // at reset, set clock to 0:00:00
int minutes = 0;
int hours = 0;                        // at reset, set clock to 0:00:00
uint16_t gCharge_Timer = 0;           // default to zero for delay timer
int ModeTimer = 0;                    // use for off timer = 10 mins get set later
// byte T_MODECHECK = 10;             // update historical vars every minute for a 10 min running average
byte T_MODECHECK = 10;                // update historical vars every minute for a 10 min running average
const int T_HISTORYCHECK = 1;         // update historical vars every 1 secs
int HistoryTimer = T_HISTORYCHECK;
// int Temp1 = 0;

//=================================================================================================
// Watchdog Timer
//=================================================================================================
// Set one second (it's adjustable with WDOG_T register) watchdog enable  by redefining 
// the startup_early_hook which by default disables the COP (TURN OFF WHEN DEBUGGING). 
// Must be before void setup();
//=================================================================================================
#ifdef __cplusplus
extern "C" {
#endif
void startup_early_hook() {
  WDOG_TOVALL = 1000;               // The next 2 lines sets the time-out value. 
  WDOG_TOVALH = 0;                  // VALH=1 and VALL=1000 - should get a WDT of about 1<<16+1000 = 66536ms
  WDOG_PRESC = 0;                   // prescaler
  WDOG_STCTRLH = (WDOG_STCTRLH_ALLOWUPDATE | WDOG_STCTRLH_WDOGEN); // Enable WDG
}
#ifdef __cplusplus
}
#endif // Watchdog Timer END




//=================================================================================================
// Setup
//=================================================================================================
void setup(){
  //-SERIAL SETUP---------------------------------------------------------------------------------
  Serial.begin(115200);                       // serial output @ 115200bps
  // delay(2000);                             // delay for comm window open

  //-CAN BUS SETUP---------------------------------------------------------------------------------
  Can0.begin(250000);                         // join CAN at 250Kbps
  CAN_filter_t Filter;                        // CAN filter struct
  Filter.id = 0;                              // dont filter any Rx messages
  Filter.ext = 1;                             // set filter to Rx extended
  Filter.rtr = 0;                             // remote requests off
  for(uint8_t MBnum=0; MBnum<16; MBnum++) {   // set filter on Mailboxes
    Can0.setFilter(Filter,MBnum);
  }
  
  //-UNUSED PINS-----------------------------------------------------------------------------------
  // Unused I/O make digital output for low impedance and EMI-resistant
  pinMode(A12, INPUT_PULLUP);         // weak pullup for now
  pinMode(A13, INPUT_PULLUP);   
  pinMode(UNUSEDA24, OUTPUT);         // outputs default to low but doesn't matter
  pinMode(UNUSEDA25, OUTPUT);   
  pinMode(UNUSEDA26, OUTPUT);   
  pinMode(UNUSEDA27, OUTPUT);   
  pinMode(UNUSEDA28, OUTPUT);   
  pinMode(UNUSEDA29, OUTPUT);   
  pinMode(UNUSEDA30, OUTPUT);   
  pinMode(UNUSEDA31, OUTPUT);   
  pinMode(UNUSEDA32, OUTPUT);   
  pinMode(UNUSEDA33, OUTPUT);   
  
  //-NRF24 2.4MHZ PACKET COMMS---------------------------------------------------------------------
  if (!manager.init())
  {
    VERBOSE_PRINT("Comms init failed");
    manager.init();                   // and try again if not the first time
  }
  else  VERBOSE_PRINTLN("Comms init success");

  //-PWM OUT---------------------------------------------------------------------------------------
  analogWriteFrequency(PWM1, PWMFREQ);    // Teensy PWM runs at 23kHz
  analogWriteResolution(DAC_RESOLUTION);  // DAC0 value 0 to 1023

  //-LITHIUM CELL SPECIFICATIONS-------------------------------------------------------------------
  if (Cell_Type == LG_MH1) {          // cell parameters for LG MH1 3200mah
    Vcell_HVD_Spec = 4.21;            // high voltage disconnect, 4.25 max leave room for 0.1% acc.
    Vcell_Nominal_Spec = 3.67;
    Vcell_Low_Spec = 2.91;            // end voltage cutoff 2.5V is spec
    Vcell_LVD_Spec = 2.91;            // low voltage disconnect, all off
    Vcell_Balance = 4.11;             // voltage above where Cells will balance
    Vcell_Trickle_Charge = 2.7;       // voltage under where trickle charge should happen
    Vcell_Bulk_Charge = 4.05;         // voltage under where the bulk of the charge should happen
    Vcell_Off_Charge  = 4.15;         // voltage above where charger should turn off
  } 
  if (Cell_Type == LG_MJ1) {          // cell parameters for LG MH1 3500mah
    Vcell_HVD_Spec = 4.21;            // high voltage disconnect, charger disconnected
    Vcell_Nominal_Spec = 3.635;
    Vcell_Low_Spec = 2.91;            // end voltage cutoff 2.5V is spec
    Vcell_LVD_Spec = 2.91;            // low voltage disconnect, all off
    Vcell_Balance = 4.10;             // start balancing
    Vcell_Trickle_Charge = 2.7;       // voltage under where trickle charge should happen
    Vcell_Bulk_Charge = 4.05;         // voltage under where the bulk of the charge should happen
    Vcell_Off_Charge  = 4.15;         // voltage above where charger should turn off
  }
  if (Cell_Type == SANYO_NCR18650B) { // cell parameters for Panasonic/Sanyo NCR18650B
    Vcell_HVD_Spec = 4.3;
    Vcell_Nominal_Spec = 3.7;
    Vcell_Low_Spec = 2.9;
    Vcell_LVD_Spec = 2.9;
    Vcell_Balance = 3.9;
    Vcell_Trickle_Charge = 2.7;       // voltage under where trickle charge should happen
    Vcell_Bulk_Charge = 4.05;         // voltage under where the bulk of the charge should happen
    Vcell_Off_Charge  = 4.2;          // voltage above where charger should turn off
  }

  //-RELAYS SETUP--------------------------------------------------------------------------------
  pinMode(RELAYDR1, OUTPUT);          // enable digital output for turning on relays on board
  pinMode(RELAYDR2, OUTPUT);          // enable digital output for turning on relays board
  // pinMode(RELAYDR3,OUTPUT);        // enable digital output for turning on low shunt
  // pinMode(RELAYDR4,OUTPUT);        // enable digital output for turning on high shunt

  //-LEDS------------------------------------------------------------------------------------------
  pinMode(LED1green, OUTPUT);         // enable digital output for turning on LED indicator
  pinMode(LED1red, OUTPUT);           // enable digital output for turning on LED indicator
  pinMode(LED2green, OUTPUT);         // enable digital output for turning on LED indicator
  pinMode(LED2red, OUTPUT);           // enable digital output for turning on LED indicator

  //-PWM OUTPUTS-----------------------------------------------------------------------------------
  pinMode(PWM1, OUTPUT);              // enable PWM1
  pinMode(PWM2, OUTPUT);              // enable PWM2

  //-USER OUTPUTS----------------------------------------------------------------------------------
  pinMode(LIMP_OUTPUT, OUTPUT);       // enable Limp mode output drive

  //-USER INPUTS-----------------------------------------------------------------------------------
  pinMode(CHARGE_INPUT, INPUT);
  pinMode(KEYSWITCH_INPUT, INPUT);
  pinMode(LEARN_BLOCKS_IN, INPUT);

  //-ANALOG SETUP----------------------------------------------------------------------------------
  //analogReference(INTERNAL);        // set analog reference to internal ref (was Jun 2016)
  analogReference(EXTERNAL);          // set analog reference to ext ref
  analogReadRes(ADC_RESOLUTION);      // Teensy 3.0: set ADC resolution to this many bits
  pinMode(NTCambient, INPUT);
  analogWrite(CHARGER_CONTROL, 0);    // prog CHG current to 0

  //-STARTUP PHYSICAL INDICATION-------------------------------------------------------------------
  VERBOSE_PRINTLN("1: Green LED1 ");
  WatchdogReset();                             // reset the watchdog timer
  digitalWrite(LED1green, HIGH);  delay(100);  // LED on for .1 second
  VERBOSE_PRINTLN("1: then go RED ");
  digitalWrite(LED1green, LOW);
  WatchdogReset();                             // reset the watchdog timer
  digitalWrite(LED1red, HIGH);    delay(100);  // LED on for .1 second
  VERBOSE_PRINTLN("2: Green LED2");
  WatchdogReset();                             // reset the watchdog timer
  digitalWrite(LED2green, HIGH);  delay(100);  // LED on for .1 second
  VERBOSE_PRINTLN("3: Now go RED ");
  digitalWrite(LED2green, LOW);
  WatchdogReset();                             // reset the watchdog timer
  digitalWrite(LED2red, HIGH);    delay(100);  // LED on for .1 second
  digitalWrite(LED1green, LOW);                //leds all off
  digitalWrite(LED1red, LOW);
  digitalWrite(LED2green, LOW);
  digitalWrite(LED2red, LOW);

  //-SAVE ALL VARS UNTIL BLOCKS ARE AWAKE AND COMMS ARE ESTABLISHED--------------------------------
  Highest_Vcell = Vcell_Nominal_Spec;  // load  vars with nominal values
  Lowest_Vcell = Vcell_Nominal_Spec;
  Highest_Tcell = NTC_AMBIENT;
  Lowest_Tcell = NTC_AMBIENT;
  Hist_Highest_Vcell = Highest_Vcell;  // running average of all blocks highest cell v
  Hist_Lowest_Vcell = Lowest_Vcell;
  Hist_Highest_Tcell = NTC_AMBIENT;    // average of ALL blocks highest cell temp (seed w/ nom 25C)
  Hist_Lowest_Tcell = NTC_AMBIENT;     // seed with nominal 25C

  //-RETREIVE EEPROM CONTENTS INTO RAM FOR BLOCK COMMS---------------------------------------------
  int EE_address = 0;
  byte EE_value;
  for (EE_address = 0; EE_address < 10; EE_address++) {
    EE_value = EEPROM.read(EE_address);                     // read for print debug
    blockNum[EE_address] = EE_value;                        // READ EEprom at power up
    VERBOSE_PRINT("READ from EEprom address:  ");   VERBOSE_PRINT(blockNum[EE_address]);   
    VERBOSE_PRINT(" = Block#: ");                   VERBOSE_PRINTLN(EE_value);
    VERBOSE_PRINT("READ from EEprom address:  ");   VERBOSE_PRINT(EE_address);   
    VERBOSE_PRINT(" = Block#: ");                   VERBOSE_PRINTLN(EE_value);
    VERBOSE_PRINT("READ from EEprom address:  ");   VERBOSE_PRINT(EE_address);   
    VERBOSE_PRINT(" = Block#: ");                   VERBOSE_PRINTLN(EE_value);
    VERBOSE_PRINT("READ from EEprom address:  ");   VERBOSE_PRINT(EE_address);   
    VERBOSE_PRINT(" = Block#: ");                   VERBOSE_PRINTLN(EE_value);
    VERBOSE_PRINT("READ from EEprom address:  ");   VERBOSE_PRINT(EE_address);   
    VERBOSE_PRINT(" = Block#: ");                   VERBOSE_PRINTLN(EE_value);
    VERBOSE_PRINT("READ from EEprom address:  ");   VERBOSE_PRINT(EE_address);   
    VERBOSE_PRINT(" = Block#: ");                   VERBOSE_PRINTLN(EE_value);
    VERBOSE_PRINT("READ from EEprom address:  ");   VERBOSE_PRINT(EE_address);   
    VERBOSE_PRINT(" = Block#: ");                   VERBOSE_PRINTLN(EE_value);
    VERBOSE_PRINT("READ from EEprom address:  ");   VERBOSE_PRINT(EE_address);   
    VERBOSE_PRINT(" = Block#: ");                   VERBOSE_PRINTLN(EE_value);
    VERBOSE_PRINT("READ from EEprom address:  ");   VERBOSE_PRINT(EE_address);   
    VERBOSE_PRINT(" = Block#: ");                   VERBOSE_PRINTLN(EE_value);
  }

  //-RF24 TIME SETTINGS----------------------------------------------------------------------------
  // Default RF24 manager settings are 2000ms timeout and 3 retries.
  // Timeout can be 2x to avoid collisions. These are too long for COP, and we don't want
  // COP reset in comm routine in case the program hangs in that code... 
  // (*just* barely works at 110ms/3 retries, and 50ms/9)
  manager.setTimeout(2000);
  manager.setRetries(3);
  //manager.setPayloadSize(12);  // 2 floats (4x2 bytes) = 8bytes+2+2 = 12 bytes MUST AGREE W/ CLIENTS!!


  //-STARTUP SERIAL OUTPUT-------------------------------------------------------------------------
  VERBOSE_PRINTLN(F("Start Pack Supervisor"));
  VERBOSE_PRINT(F("Firmware Ver: "));  VERBOSE_PRINTLN(VERSION);

  // only turn LIMP mode off at power cycle - Jul 12, 2017
    digitalWrite(LIMP_OUTPUT, LOW);
    VERBOSE_PRINTLN("'Limp Mode'  is off because DK is being reset ");
} // Setup End



//=================================================================================================
// Watchdog Function
// the smallest delay needed between each refresh is 1ms. anything faster and it will also reboot
//=================================================================================================
void WatchdogReset (void) {      // reset COP watchdog timer to 1.1 sec 
  noInterrupts(); 
  WDOG_REFRESH = 0xA602;
  WDOG_REFRESH = 0xB480;
  interrupts();
  delay(1);                     
}

//=================================================================================================
// CAN Send Charger Function
// Sends data to charger, adjusts voltage, current, and on or off
//=================================================================================================
void CANSendCharger(uint32_t id, uint16_t  cVoltage, uint16_t cCurrent, bool on_off){
  CAN_message_t hold;
  hold.id = id;                                                    // set id
  hold.ext = 1;                                                    // set message as extended
  hold.rtr = 0;                                                    // set remote off
  hold.len = 5;                                                    // message length in bytes
  hold.buf[0] = (uint8_t)((cVoltage & 0xFF00) >> 8);               // split the two bytes of voltage                               
  hold.buf[1] = (uint8_t)(cVoltage & 0x00FF);
  hold.buf[2] = (uint8_t)((cCurrent & 0xFF00) >> 8);               // split the two bytes of current
  hold.buf[3] = (uint8_t)(cCurrent & 0x00FF);
  if(on_off == 1) hold.buf[4] = 0x00;                              // 00 is on
  else hold.buf[4] = 0x01;                                         // send to the bus
  Can0.write(hold);

  #ifdef CANDEBUG
  DEBUG_PRINT("Tx - ")
  DEBUG_PRINT_HEX(hold.id); DEBUG_PRINT(" ");
  DEBUG_PRINT_HEX(hold.len); DEBUG_PRINT(" ");
  for(int i = 0; i<hold.len; i++){
    DEBUG_PRINT_HEX(hold.buf[i]); DEBUG_PRINT(" ");
  }
  DEBUG_PRINTLN();
  #endif
}

//=================================================================================================
// CAN Receive Function
// Receives data from can bus, returns it in structure
//=================================================================================================
struct CAN_message_t CANReceive(){
  CAN_message_t hold;
  if(Can0.available()) Can0.read(hold);
  
  #ifdef CANDEBUG
  DEBUG_PRINT("Rx - ");
  DEBUG_PRINT_HEX(hold.id); DEBUG_PRINT(" ");
  DEBUG_PRINT_HEX(hold.len); DEBUG_PRINT(" ");
  for(int i = 0; i<hold.len; i++){
    DEBUG_PRINT_HEX(hold.buf[i]); DEBUG_PRINT(" ");
  }
  DEBUG_PRINTLN();
  #endif

  return hold;
}

//=================================================================================================
//  Main Loop
//=================================================================================================
void loop() {

  VERBOSE_PRINTLN();
  VERBOSE_PRINTLN();
  VERBOSE_PRINT(F("  Server Address: "));  VERBOSE_PRINT(SERVER_ADDRESS); 
  VERBOSE_PRINT(F("  PS Mode: "));         VERBOSE_PRINTLN(gMode);

  WatchdogReset();  // reset timer (times out in 1 sec so make sure loop is under about 500-600msec)

  VERBOSE_PRINTLN();
  VERBOSE_PRINT(F("Secs = "));                          VERBOSE_PRINTLN(seconds);
  VERBOSE_PRINT(F("Mins = "));                          VERBOSE_PRINTLN(minutes);
  VERBOSE_PRINT(F("Hrs = "));                           VERBOSE_PRINTLN(hours);
  VERBOSE_PRINT(F("Millisecond counter: "));            VERBOSE_PRINTLN(millis());
  VERBOSE_PRINT(F("Historical Avg Hottest Tcell: "));   VERBOSE_PRINT_0(Hist_Highest_Tcell);  
  VERBOSE_PRINT(" ADC counts ");

  //-MAKE CLOCK TICK TOCK--------------------------------------------------------------------------
  currentmicros = micros(); // read the time.
  bool Comm_Flag = 0;

  //-1000 MILLISECOND CLOCK TICK? RUN REAL TIME CLOCK----------------------------------------------
  if ((currentmicros - nextmicros) >= interval)   // if 1000000 microseconds have gone by...
  {
    nextmicros = nextmicros + interval;           // update for the next comparison
    seconds = seconds + 1;                        // and run real time clock
    Comm_Flag = 1;                      
    // if (DpSleep_timer > 0) DpSleep_timer --;   // run deepsleep timer every second
    if (HistoryTimer > 0) HistoryTimer --;        // run history timer every second
    if (seconds == 60)
    {
      seconds = 0;
      minutes = minutes + 1;
      // if (ModeTimer > 0) ModeTimer = ModeTimer-1;
      if (minutes == 60)
      {
        minutes = 0;
        hours = hours + 1;
        if (gCharge_Timer > 0) gCharge_Timer --;
      }
      if (hours == 24) hours = 0;
    }
  }

  //-LEARN MODE-----------------------------------------------------------------------------------
  // Learn mode switch routine...check if learn == on or off
  //tempx ++;
  tempx = tempx + digitalRead(LEARN_BLOCKS_IN);
  if ((tempx > 0) && (hours == 0) && (minutes < LEARN_TIMEOUT))   // learn blocks in first 3 minutes
  {
    if (tempx > 5)
    {
      tempx = 10;
      if (minutes < 1) LEARNBLOCKS = ON;  // turn on first minute, allow to be turned off after that
      if (LearnBlockSwitch == ON)
      {
        for(uint8_t b = 0; b < NUMBER_OF_BLOCKS; b++){
          blockNum[b] = 0;                // first time, 0 all the block locations, for learning
        }
        LearnBlockSwitch = OFF ;
        VERBOSE_PRINT(F("***************** ZERO Blocks *****************"));
      }
    }
    else if (seconds > 55)  tempx = 0;    // if switch is not pressed, 0 out var after 5 seconds
  }
  else {
    LEARNBLOCKS = OFF;
  }
  VERBOSE_PRINTLN();
  VERBOSE_PRINT("    Learn Blocks is ON/OFF: ");  VERBOSE_PRINTLN(LEARNBLOCKS);
  VERBOSE_PRINTLN("  These Block numbers populate these memory locations: ");
  VERBOSE_PRINT("EEprom location 0 =  "); VERBOSE_PRINT("Block Addr "); VERBOSE_PRINTLN(blockNum[0]);
  VERBOSE_PRINT("EEprom location 1 =  "); VERBOSE_PRINT("Block Addr "); VERBOSE_PRINTLN(blockNum[1]);
  VERBOSE_PRINT("EEprom location 2 =  "); VERBOSE_PRINT("Block Addr "); VERBOSE_PRINTLN(blockNum[2]);
  VERBOSE_PRINT("EEprom location 3 =  "); VERBOSE_PRINT("Block Addr "); VERBOSE_PRINTLN(blockNum[3]);
  VERBOSE_PRINT("EEprom location 4 =  "); VERBOSE_PRINT("Block Addr "); VERBOSE_PRINTLN(blockNum[4]);
  VERBOSE_PRINT("EEprom location 5 =  "); VERBOSE_PRINT("Block Addr "); VERBOSE_PRINTLN(blockNum[5]);
  VERBOSE_PRINT("EEprom location 6 =  "); VERBOSE_PRINT("Block Addr "); VERBOSE_PRINTLN(blockNum[6]);
  VERBOSE_PRINT("EEprom location 7 =  "); VERBOSE_PRINT("Block Addr "); VERBOSE_PRINTLN(blockNum[7]);
  VERBOSE_PRINT("EEprom location 8 =  "); VERBOSE_PRINT("Block Addr "); VERBOSE_PRINTLN(blockNum[8]);
  VERBOSE_PRINT("EEprom location 9 =  "); VERBOSE_PRINT("Block Addr "); VERBOSE_PRINTLN(blockNum[9]);
  VERBOSE_PRINTLN();

  byte tempbl = 0  ;
  //byte TWO_MINUTES = 30;      // 240 seconds (4 min)
  byte TWO_MINUTES = 120;       // 120 seconds (2 min)
  //bool Disconnected_Block;    // start with no fault
  
  VERBOSE_PRINTLN("  These Block numbers contain these timeout values: ");
  while ( tempbl < NUMBER_OF_BLOCKS ) {
    VERBOSE_PRINT("Block: ");  VERBOSE_PRINT(blockNum[tempbl]); 
    VERBOSE_PRINT(" = ");      VERBOSE_PRINTLN(Block_Comm_Timer[tempbl]);
    //tempbl++;
    // run comm disconnect timer on all blocks here - count to 4 min and stop
    if (Comm_Flag) {                                      // run timer here, gets zero'd in comm routine
      if (Block_Comm_Timer[tempbl] < COMM_TIMEOUT) Block_Comm_Timer[tempbl] ++;   
      if (Block_Comm_Timer[tempbl] >= TWO_MINUTES) {
        Disconnected_Block = YES;                        //  yes at least one block is disconnected
        Disconnected_BlockNum = blockNum[tempbl];
      }
    }
    tempbl++;
  }
  VERBOSE_PRINTLN();
  if (Disconnected_Block) {
    VERBOSE_PRINT("Block: "); VERBOSE_PRINT(Disconnected_BlockNum); 
    VERBOSE_PRINT(" = ");     VERBOSE_PRINTLN("DISCONNECTED");
  }
  VERBOSE_PRINTLN();
  bool Tempdisc = NO;
  tempbl = 0  ;
  // Now check all blocks for comms in 4 min, when timers are reset, clear Disconnect switches
  while ( tempbl < NUMBER_OF_BLOCKS ) {
    if (Block_Comm_Timer[tempbl] > TWO_MINUTES) {
      Tempdisc = YES;                              //  yes at least one block is disconnected
      VERBOSE_PRINT("Block DISCONNECT.... "); VERBOSE_PRINTLN("Block DISCONNECT");
    }
    tempbl++;
  } 
  if (Tempdisc == NO) {
    Disconnected_Block = NO;                    //  no block is disconnected
    Disconnected_BlockNum = 0;
    VERBOSE_PRINT("All Blocks connected...."); VERBOSE_PRINTLN("All Blocks connected");
  }
  VERBOSE_PRINTLN();
  // Write new learned block values to EEPROM
  // start writing and reading from the first byte (address 0) of the EEPROM
  int EE_address = 0;
  #ifdef VERBOSE
  byte EE_value;
  #endif
  //if ((LEARNBLOCKS == ON) && (blockNum[0]))   // Make sure we are in learn mode and all blocks are written
  if ((LEARNBLOCKS == ON) && (seconds == 0)) {  // Write EE once/sec when we are in learn mode
    for (EE_address = 0; EE_address < 10; EE_address++) {
      EEPROM.write(EE_address, blockNum[EE_address]);
      #ifdef VERBOSE
      EE_value = EEPROM.read(EE_address);
      #endif
      VERBOSE_PRINT("Once/min save Block#: ");    VERBOSE_PRINT(EE_value); 
      VERBOSE_PRINT("to this EEprom address:  "); VERBOSE_PRINTLN(EE_address);
      VERBOSE_PRINT("Once/min save Block#: ");    VERBOSE_PRINT(EE_value); 
      VERBOSE_PRINT("to this EEprom address:  "); VERBOSE_PRINTLN(EE_address);
    }
    if (blockNum[0]) LEARNBLOCKS = OFF;         // if last block is saved to EE, turn off Learn blocks
  }
  if (LEARNBLOCKS == ON) {                      // turn on RED LEDs
    digitalWrite(LED2red, HIGH);
    digitalWrite(LED1red, HIGH);
  }
  else {                                 // turn off red LEDs unless
    if (blockNum[0] == 0) {
      digitalWrite(LED2red, HIGH);       // turn on LED2=RED if all Blocks not programmed
      digitalWrite(LED1red, LOW);
    }
    else {
      digitalWrite(LED2red, LOW);
      digitalWrite(LED1red, LOW);
    }
  }

  //-MEASURE TEMPERATURE---------------------------------------------------------------------------
  // measure ambient NTC - start with all LEDs red. If NTC gets shorted, make them green
  // digitalWrite(LED1red,HIGH);
  // digitalWrite(LED2red,HIGH);
  float datSum = 0;                     // reset our accumulated sum of input values to zero
  long n = 0;                           // count of how many readings so far
  double x = 0;
  for (int i = 0; i < SAMPLES; i++) {
    x = analogRead(NTCambient);
    if ((i < 10 ) || (i > 150)) goto throwaway_a;  // throw away the first 10 and last 50 samples
    datSum += x;
    n++;
    throwaway_a:  ;
  }
  float datAvg = (datSum) / n;          // find the mean
  #ifdef VERBOSE
  int Tambient = datAvg;                // save ambient
  #endif
  VERBOSE_PRINT(NTCambient);                    VERBOSE_PRINT(" :");
  VERBOSE_PRINT("  NTC ambient avg value: ");   VERBOSE_PRINTLN(Tambient);
  if (datAvg < 100) {
    VERBOSE_PRINTLN("NTC possibly shorted ");
  }

  //-MEASURE CHARGE CURRENT------------------------------------------------------------------------
  datSum = 0;                                       // reset our accumulated values to zero
  n = 0;                                            // count of how many readings so far
  x = 0;
  bool polarity;
  float LEM_Offset = EEPROM_CHG_SENSOR_OFFSET;      // starting value
  //uint8_t near0 = 15;                             // near zero amps so cal sensor value
  for (int i = 0; i < SAMPLES; i++) {
    x = analogRead(ICHG);
    if ((i < 10 ) || (i > 150)) goto throwaway_i;   // throw away the first 10 and last 50 samples
    datSum += x;
    n++;
    throwaway_i:   ;
  }
  datAvg = datSum / n;                              // find the mean
  VERBOSE_PRINT(ICHG);
  if (datAvg < 50) {
    VERBOSE_PRINT(F("ICHG -> Error - Input is shorted or no signal "));
  }
  VERBOSE_PRINT("Chan: ");              VERBOSE_PRINT(ICHG);
  VERBOSE_PRINT(" ICHG avg value: ");   VERBOSE_PRINTLN_2(datAvg);

  // cal when close to 2.5V which is LEM offset at 0 amps
  // calibrate sensor when not charging or drivemode
  if ((gMode != CHARGEMODE) && (gMode != DRIVEMODE)) {
    //if ((datAvg > (LEM_Offset-near0)) && (datAvg < LEM_Offset+near0)) LEM_Offset = datAvg;    
  }
  // Sensor is offset at 2.50V from ground, so subract offset from signal to get actual current
  if (datAvg >= EEPROM_CHG_SENSOR_OFFSET) {
    datAvg = datAvg - LEM_Offset;
    polarity = 1;
  }
  else {
    datAvg = LEM_Offset - datAvg;
    polarity = 0;
  }
  // Theoretical sensitivity for the LEM HO 100S-0100 = 16mv/amp. At 12 bit resolution each 
  // bit = 3.3V/4096 = 0.8057mV. So each bit = .8057/16 = .0504 amps. Or 19.859 counts per amp.

  float TempAmps;
  TempAmps = (datAvg / 10.3);                 // scale current to amps
  //gAmps = (TempAmps + gAmps + gAmps + gAmps + gAmps) / 5.00; // average for noise reduction
  gAmps = (TempAmps + (gAmps * 9)) / 10.00;   // average for noise reduction
  VERBOSE_PRINT(" Amps avg value: ");
  if(polarity) {
    VERBOSE_PRINT("+");
  }
  else {
    VERBOSE_PRINT("-");
  }
  VERBOSE_PRINT_3(gAmps);
  VERBOSE_PRINTLN(" (POS => charge and NEG => discharge)");
  VERBOSE_PRINTLN ();

  //-MEASURE DISCHARGE CURRENT---------------------------------------------------------------------
  datSum = 0;                       // reset our accumulated sum of input values to zero
  n = 0;                            // count of how many readings so far
  x = 0;
  for (int i = 0; i < SAMPLES; i++) {
    x = analogRead(IDISCHG);
    datSum += x;
    n++;
  }
  datAvg = (1.0 * datSum) / n;      // find the mean
  VERBOSE_PRINT(IDISCHG);
  VERBOSE_PRINT(" IDISCHG avg value: ");     VERBOSE_PRINT_2(datAvg);
  if (datAvg < 50) {
    VERBOSE_PRINT("IDISCHG -> Error - Input is shorted or no signal ");
  }
  VERBOSE_PRINTLN ();
  digitalWrite(LED1green, HIGH);
  
  //-READ AND SCALE PACK VOLTAGE INPUT-------------------------------------------------------------
  delay(10);
  datSum = 0;                                   // reset our accumulated sum of input values to zero
  n = 0;                                        // count of how many readings so far
  x = 0;
  for (int i = 0; i < SAMPLES; i++) {
    x = analogRead(VSCALEDPACK);
    if ((i < 10 ) || (i > 150)) goto throwaway_c;  // throw away the first 10 and last 50 samples
    datSum += x;
    n++;
    throwaway_c:  ;
  }
  digitalWrite(LED1green, LOW);
  datAvg = (1.0 * datSum) / n;                  // find the mean
  // int temp = (3.3/4096) ;
  // float Volts = ((datAvg * temp) * 78) ;     // calibrate 7.4V high side input after low side input
  // float Volts = ((datAvg*3.3)/4096) * 83;    // calibrate 7.4V high side input after low side input
  float Volts;
  // Volts = datAvg * 0.0575;                   // low voltage calibrate
  // Volts = datAvg * 0.0565;                   // 84.1 low voltage calibrate
  // Volts = datAvg * 0.056679;                 // 84.26 low voltage calibrate
  Volts = datAvg * 0.056680;                    // calibrate at 85V for HVD accuracy
  /* if (Vpack > 70){                           // high voltage cal or ...
     Volts = ((datAvg*3.3)/4096) * 29.40;       // low voltage calibrate
     // Volts = ((datAvg*3.3)/4096) * 29.14;    // low voltage calibrate
     }
    else {
      Volts = ((datAvg*3.3)/4096) * 29.2;
      // Volts = ((datAvg*3.3)/4096) * 28.85;
      // Volts = ((datAvg*3.3)/4096) * 26.85;
    }*/
  Volts = Volts + Vpack + Vpack;                // sum and average for noise reduction
  Vpack = Volts / 3;
  // now do fine calibration on signal
  // Vpack = Vpack - 0.010;                     // add offset to signal
  VERBOSE_PRINT(VSCALEDPACK); VERBOSE_PRINT(" Vpack: ");     
  VERBOSE_PRINT_2(Vpack);     VERBOSE_PRINTLN("VDC");

  // Make logic determinations for relays = ON or OFF, and use hysteresis to prevent relay oscillation
  // eg 4.21+.02=4.23 x 20 = 84.6 (.02 is headroom to balance (remember high cellV will open relay too)
  const float Vpack_HVD = ((Vcell_HVD_Spec + 0.02) * No_Of_Cells) ;   
  int Vpack_LVD = Vcell_LVD_Spec * No_Of_Cells;           // eg 2.9 x 20 = 58
  int Vpack_HV_Run_Limit = Vpack_HVD * 1.2;               // Make high voltage run limit 10% higher than HVD
  int Vpack_Lo_Run_Limit = Vpack_LVD;

  //---------CHARGE RELAY--------------------------------------------------------------------------
  // pack check & cell check for charger relay
  // default to relay off
  // check cell and pack V before turning on relay (4.21 if LG)
  ChargeRelay = OFF;                                                          
  if ((Hist_Highest_Vcell < (Vcell_HVD_Spec)) && (Vpack < (Vpack_HVD))) {    
    if ((digitalRead(CHARGE_INPUT) == 0)) {               // check charger input for low side switch
      VERBOSE_PRINT(F(" Charge input ON / Timer = "));  
      VERBOSE_PRINT(gCharge_Timer);  VERBOSE_PRINTLN(F(" hrs"));
      if (gCharge_Timer == 0) ChargeRelay = ON;           // if timer is 0 turn on charge relay for up to a day
      if(Tx_counter >= Tx_msg_interval) {                 // send message at set speed (1 per second)
        Tx_counter = 0;
        if(Hist_Highest_Vcell < Vcell_Trickle_Charge){
          CANSendCharger(BMS_TO_CHARGER, Charge_Voltage, Charge_Trickle_Current, ON);
          Charge_status_flag = TRICKLE;
        }
        if(Hist_Highest_Vcell < Vcell_Bulk_Charge && Charge_status_flag <= BULK) {
          CANSendCharger(BMS_TO_CHARGER, Charge_Voltage, Charge_Max_Current, ON);
          Charge_status_flag = BULK;
        }
        if(Hist_Highest_Vcell > Vcell_Bulk_Charge && Charge_status_flag <= TOPEND) {
          CANSendCharger(BMS_TO_CHARGER, Charge_Voltage, Charge_End_Current, ON);
          Charge_status_flag = TOPEND;
        }
        if(Hist_Highest_Vcell < Vcell_Bulk_Charge && Charge_status_flag == TOPEND) {
          CANSendCharger(BMS_TO_CHARGER, Charge_Voltage, Charge_End_Current, ON);
          Charge_status_flag = TOPEND;
        }
        if(Hist_Highest_Vcell >= Vcell_Off_Charge) {
          CANSendCharger(BMS_TO_CHARGER, Charge_Voltage, Charge_End_Current, OFF);
          Charge_status_flag = COMPLETE;
        }
        #ifdef DEBUG
        switch (Charge_status_flag){
          case TRICKLE:
            DEBUG_PRINTLN(F("Charge Status = LOW TRICKLE"))
            break;
          case BULK:
            DEBUG_PRINTLN(F("Charge Status = HIGH CURRENT"))
            break;
          case TOPEND:
            DEBUG_PRINTLN(F("Charge Status = TOPPING UP"))
            break;
          case COMPLETE:
            DEBUG_PRINTLN(F("Charge Status = COMPLETE"))
            break;
        }
        #endif
      }
      if (Hist_Lowest_Vcell > Vcell_Balance) {            // shut down relay, start 1 day timer (4.11 if LG)
        gCharge_Timer = 24;                               // 24 hours
        ChargeRelay = OFF;
        VERBOSE_PRINTLN(F(" Charge relay IS OFF for 24 hours because cells are ALL in balance "));
      }
      else {                                              // reset charge timer if Vcell < 4V
        if (Hist_Lowest_Vcell < CELL_RECONNECT_V) {
          gCharge_Timer = 0;
          VERBOSE_PRINTLN(F(" Charge timer reset to 0 hrs because Vcell lowest discharged below 4V "));
        }
      }
    }
    else {
      VERBOSE_PRINTLN(F(" Charge relay IS OFF because charge input is OFF "));
      gCharge_Timer = 0;                                  // reset 24 hour timer because charger is disconnected
    }
  }
  else {                                                  // Open charge relay if cells or pack go over limits
    VERBOSE_PRINTLN("**** FAULT - Cell or pack voltage is too high to initiate charger now.... ");
    gCharge_Timer = 24;                                   // relay is shut and counter is set to 24 hours
    VERBOSE_PRINT(F(" Charge input OFF - Timer = "));  VERBOSE_PRINT(gCharge_Timer);  VERBOSE_PRINTLN(F(" hrs"));
  }

  //---------MOTOR RELAY---------------------------------------------------------------------------
  MtrControlRelay = OFF;                                 // default to relay off
  // Pack check for motor relay and cell check for motor relay
  if ((Vpack > Vpack_HV_Run_Limit) || (Vpack < Vpack_Lo_Run_Limit)) {
    VERBOSE_PRINTLN(" Drive relay IS OFF because Vpack is too LOW or too HIGH");
    //MtrControlRelay = OFF;                             // if pack voltage too high turn off motor control
  }
  else {
    if ((Vpack < (Vpack_HV_Run_Limit - HYSTERESIS)) && (Hist_Lowest_Vcell > Vcell_LVD_Spec)) {
      if ((digitalRead(CHARGE_INPUT) != 0) && (digitalRead(KEYSWITCH_INPUT) == 0)) {
        MtrControlRelay = ON;   // if pack ok, and charger is off (or disconnected) turn on motor control
        VERBOSE_PRINTLN(F(" Drive: ON -> Keyswitch: ON -> Charge: OFF "));
      }
      else VERBOSE_PRINTLN(F(" Drive: OFF -> Keyswitch: OFF -or- Charge: ON -> Pack and Cells: OKAY! "));
    }
    else {
      VERBOSE_PRINTLN(F(" Drive: OFF -> Pack or Cells: LOW -> SO CHARGE PACK! "));
      gFaultMode = UNDERVOLT_CELL;         
    }
  }
  // cell temperature check, make limp mode => 60C, relay open at 63C
  if (Hist_Highest_Tcell < NTC_63C) {
      MtrControlRelay = OFF;               // cell >63C so turn off motor and charger relays
      ChargeRelay = OFF;
      VERBOSE_PRINTLN(F(" Drive: OFF -> Charge: OFF -> Pack and Cells: OVERTEMP! "));
  }
  else {} // cells not hot

  VERBOSE_PRINT(" Charger HVD: ");         VERBOSE_PRINT_2(Vpack_HVD);           VERBOSE_PRINTLN("VDC");
  VERBOSE_PRINT(" Motor LVD: ");           VERBOSE_PRINT_1(Vpack_LVD);           VERBOSE_PRINTLN("VDC");
  VERBOSE_PRINT(" Vpack_HV_Run_Limit: ");  VERBOSE_PRINT_1(Vpack_HV_Run_Limit);  VERBOSE_PRINTLN("VDC");
  VERBOSE_PRINT(" Vpack_Lo_Run_Limit: ");  VERBOSE_PRINT_1(Vpack_Lo_Run_Limit);  VERBOSE_PRINTLN("VDC");
  VERBOSE_PRINTLN ();

  // if undervolt cell AND Comms timeout (= comms error), wait until no comms error to turn on Drive
  if ((Disconnected_Block == YES) && (gFaultMode == UNDERVOLT_CELL)) {
     if (gFaultMode == UNDERVOLT_CELL) MtrControlRelay = OFF; 
     VERBOSE_PRINTLN(F(" Drive: OFF -> Cells: UNDERVOLT -or- DISCONNECT "));
     //if (gFaultMode == OVERVOLT_CELL) ChargeRelay = OFF; 
  }

  //-SWITCH RELAYS---------------------------------------------------------------------------------
  //if ((ChargeRelay == ON) && (digitalRead(CHARGE_INPUT == LOW)))
  if ((ChargeRelay == ON)) {                               // Charger input goes low when charger is conn
    digitalWrite(CHARGER_RELAY, HIGH);
    VERBOSE_PRINTLN(F("Charger Relay: CLOSED"));
    gMode = CHARGEMODE;
  }
  else {
    gMode = PAUSEMODE;
    digitalWrite(CHARGER_RELAY, LOW);
    VERBOSE_PRINTLN(F("Charger Relay: OPEN"));
  }

  if ((MtrControlRelay == ON)) {                           // keyswitch input goes lo when turned on
    digitalWrite(MTRCONTROL_RELAY, HIGH);
    VERBOSE_PRINTLN(F("Motor Relay: CLOSED"));
    gMode = DRIVEMODE;
  }
  else {
    digitalWrite(MTRCONTROL_RELAY, LOW);
    VERBOSE_PRINTLN(F("Motor Relay: OPEN"));
  }
  //VERBOSE_PRINT("CRelay cycles: ");    VERBOSE_PRINT(Crelay_Cycles);  
  //VERBOSE_PRINT("  MRelay cycles: ");  VERBOSE_PRINT(Mrelay_Cycles);

  //-ANALOG CHARGER CONTROL (DAC2)-----------------------------------------------------------------
  float TargetI;                              //Amps
  //uint16_t gTempPot;                        // current potentiomenter
  const int FULL_CHARGE_RATE = 1060 / 2;      // full charge rate = 5V out for Elcon charger control
  uint16_t FullChargeCurrent = 30;            // 30 amps full chg for FIDO
  const int STARTING_CHARGE_RATE = 1023 / 5;  // zero charge rate to start (below 2V is zero charge rate for Elcon PFC charger
  //const float BEGIN_BALANCEV = 4.0;
  float BalanceChargeCurrent = 0.20;          // 200ma charge currrent for balance, cell balancers are running at 250ma
  //Vcell_Balance

  // 100% charge rate to 4VPC, 100ma from there on,  default to 50% charge rate
  // if ((Hist_Highest_Vcell < Vcell_Balance) && (Vpack < Vpack_HVD )) TargetI = FullChargeCurrent; 
  // Full charge rate to 90% or 4.000V
  if ((Hist_Highest_Vcell < Vcell_Balance) && (Vpack < (Vcell_Balance * No_Of_Cells))) TargetI = FullChargeCurrent; 
  // Full charge rate to 90% or 4.000V
  else {
    TargetI = BalanceChargeCurrent;           // 200ma charge rate from there up to keep cells balanced
  }
  VERBOSE_PRINT(" Target Charge amps: ");  VERBOSE_PRINTLN_2(TargetI);
  VERBOSE_PRINT(" Actual Charge amps: ");  VERBOSE_PRINTLN_2(gAmps);

  // init current pot
  if (gMode != CHARGEMODE) gTempPot = STARTING_CHARGE_RATE;
  //charge control fix Apr 2017
  //  if (seconds == 0) // every second check current and current control
  else {
  //charge control fix Apr 2017
    if ((gAmps > TargetI) && (gTempPot > 0)) gTempPot--;
    if ((gAmps < TargetI) && (gTempPot < FULL_CHARGE_RATE)) gTempPot++;
    // do nothing if they are equal
  //charge control fix Apr 2017
  //  analogWrite(CHARGER_CONTROL, gTempPot);     // PWM2 = DAC2 == CHG control 2-5V = 0-100%
  }
  //charge control fix Apr 2017
    analogWrite(CHARGER_CONTROL, gTempPot);       // PWM2 = DAC2 == CHG control 2-5V = 0-100%
  //charge control fix Apr 2017
  VERBOSE_PRINT(" Charger control setting: ");  VERBOSE_PRINT(gTempPot);  VERBOSE_PRINTLN(" out of 1024: ");

  //-VOLTAGE DRIVEN FUEL GAUGE---------------------------------------------------------------------
  // output SOC signal from op amp PWM - digital DAC outputs
  // 60-80V Vpack ==> 0-100% for now, VOLTAGE IS NOT A GOOD FUEL GAUAGE
  // but it's ok for now November 2016 until good SOC meter is built
  float TempV = Vnominal;    // temp pack Volts
  int SOCv;     // SOC based on voltage
  const int MAX_PWM = 1023;
  //Average SOCv over minutes (improvement) - Jul 12, 2017
  if ((MtrControlRelay == OFF) && (ChargeRelay == OFF)) gSOCv = Vpack ;           
  // both motor and charger relays are off == init SOC == Vpack
  // float gSOCv;
  VERBOSE_PRINT(" Vbat based SOC: ");  VERBOSE_PRINT(gSOCv);  VERBOSE_PRINTLN("........................... ");
  gSOCv = (Vpack + (gSOCv * 999)) / 1000.0; // average for noise reduction 
  //gSOCv = (Vpack + (gSOCv * 999)) / 1000.0; // average for noise reduction
  if (gSOCv >= 59) TempV = gSOCv - 59;    // 59V is (almost) dead at 2.95V per cell
  else TempV = 0;
  //TempV = Vpack - 59;    // 59V is (almost) dead at 2.95V per cell
  //Average SOCv over minutes (improvement) -  Jul 12, 2017
  
  TempV = TempV * 50;
  // limit check...
  if (Vpack < 59) TempV = 0;
  if (TempV > MAX_PWM) TempV = MAX_PWM;     // 10 bit resolutioin - 1024 Max

  analogWrite(PWM1, TempV);                 // Pin 20 - PWM1 ==DAC1 == SOC output
  // analogWrite(PWM2, TempV);              // PWM2 = DAC2 == CHG control

  VERBOSE_PRINT(" PWM1 Counts (SOC): ");  VERBOSE_PRINT_0(TempV);
  SOCv = (TempV / 1024) * 100;              // now write to KOSO meter var
  VERBOSE_PRINT(" SOC: ");  VERBOSE_PRINT(SOCv);  VERBOSE_PRINTLN ("%");

  //-LIMP MODE LOGIC-------------------------------------------------------------------------------
  // only turn LIMP mode off at power cycle or keyswitch cycle (improvement limp latch) Jul 12, 2017
  if ((TempV < 200) || (Hist_Highest_Tcell < NTC_60C)) {    // Check temperatures for out of safe range
    if (seconds > 10) digitalWrite(LIMP_OUTPUT, HIGH);      // Give time after startup for vpack to settle
  }
  else if (digitalRead(KEYSWITCH_INPUT) != 0) {             // Check for power cycle
    digitalWrite(LIMP_OUTPUT, LOW);                         // Set LIMP off
    VERBOSE_PRINTLN(F("Limp: OFF -> Keyswitch: ON/OFF CYCLE"));
  }
  if (digitalRead(LIMP_OUTPUT) == 0) {                      // Check for LIMP off
    VERBOSE_PRINTLN(F("Limp: OFF -> SOC > 20% and CELL TEMP ▼60C"));
  }
  else {                                                    // LIMP is on
    if (Hist_Highest_Tcell < NTC_60C){
      VERBOSE_PRINTLN(F("Limp: ON -> CELL TEMP ▲60C"));
    }
    else{
      VERBOSE_PRINTLN(F("Limp: ON -> SOC < 20% -> Keyswitch: NOT CYCLED"));
    }
  }
  VERBOSE_PRINTLN();
  digitalWrite(LED2green, HIGH);

  //-NRF24 2.4Mhz WIRELESS COMMUNICATION-----------------------------------------------------------
  // manager.init(); // init once each hour in case hardware gets flakey

  bool GoodComms = NO;
  uint8_t PAYLOAD = 16;           // bytes, use for comparing payload, comms will ignore data if wrong PL
  float floatmatch = 0.000500;    // +/- this amount to allow for matching float math on receive
  if (manager.available()) {      // Wait for a message addressed to us from the client
    uint8_t len = sizeof(buf);
    uint8_t from;
    float tempa;                  // 4 bytes or 32 bits
    float tempb;
    if (manager.recvfromAck(buf, &len, &from)) {
      VERBOSE_PRINT(F("DKBlock Client: "));
      VERBOSE_PRINT_DEC(from);
      VERBOSE_PRINT(F(" Length = "));  VERBOSE_PRINTLN(len);
      VERBOSE_PRINT(((DATA*)buf) -> sCellV_Hiside );   VERBOSE_PRINT(F(" VDC Hi Cell   "));
      VERBOSE_PRINT(((DATA*)buf) -> sThottest);        VERBOSE_PRINT(F(" Hot NTC counts   "));
      VERBOSE_PRINT(((DATA*)buf) -> sTcoldest);        VERBOSE_PRINT(F(" Cold NTC counts   "));
      VERBOSE_PRINT(((DATA*)buf) -> sCellV_Loside );   VERBOSE_PRINT(F(" VDC Lo Cell   "));
      VERBOSE_PRINT(((DATA*)buf) -> schecksum );       VERBOSE_PRINTLN(F(" Checksum from BLOCK"));
      // filter for wrong payload length
      if (len != PAYLOAD) {
        VERBOSE_PRINTLN(" WRONG data length so do not use this bad data!!!!!!");
        goto badcomm;     // check for correct payload, get out if not
      }
      // filter for bad checksum
      Temp1 = (((DATA*)buf) -> sCellV_Hiside );
      Temp2 = (((DATA*)buf) -> sCellV_Loside );
      Temp3 = (((DATA*)buf) -> sThottest );
      Temp4 = (((DATA*)buf) -> sTcoldest );
      tempb = (((DATA*)buf) -> schecksum );
      tempa = Temp1 + Temp2 + Temp3 + Temp4;
      VERBOSE_PRINT(" Calculated checksum is = ");  VERBOSE_PRINTLN_6(tempa); 
      VERBOSE_PRINT(" Received checksum is = ");    VERBOSE_PRINTLN_6(tempb);
      //if (tempa != tempb)
      Temp1= tempb - floatmatch;
      Temp2 = tempb + floatmatch;
      VERBOSE_PRINT(" Checksum window to allow var for FP math is:   ");  VERBOSE_PRINT_6(Temp1);   
      VERBOSE_PRINT(" and high   ");  VERBOSE_PRINTLN_6(Temp2);
      // allow data if checksum is within 250uV accuracy
      if ((tempa < (tempb + floatmatch)) && (tempa > (tempb - floatmatch))) {    
        VERBOSE_PRINT(" Checksum does agree with client!!!!!!!!!!");  
      }
      else{
        for(uint8_t c=0; c<2; c++) {
          VERBOSE_PRINT(" Checksum does NOT agree with client"); 
          VERBOSE_PRINT(" Bad comms so no save BAD DATA");
        } 
        goto badcomm;                           // bad data = no save
      }      
      VERBOSE_PRINT(" Calculated checksum is = ");  VERBOSE_PRINT_6(tempa); 
      VERBOSE_PRINT(" Received checksum is = ");    VERBOSE_PRINTLN_6(tempb);
      GoodComms = YES;
      // learn blocks to connect to here...if in LEARN BLOCKS mode
      uint16_t x = NUMBER_OF_BLOCKS;            // number of blocks (+ 1 for zero)
      bool This_Block_Saved = NO;
      //  Save this block if not already saved
      if ((LEARNBLOCKS == ON)) {
        digitalWrite(LED2red, HIGH);
        digitalWrite(LED1red, HIGH);

        while ( x > 0 ) {
          if (from == blockNum[x - 1]) {        // first check = has BLOCK already been saved?
            VERBOSE_PRINT("THIS BLOCK IS ALREADY SAVED IN LOCATION: "); VERBOSE_PRINTLN(x - 1);
            VERBOSE_PRINT("from: "); VERBOSE_PRINT(from); VERBOSE_PRINT("blockNum: "); VERBOSE_PRINTLN(x - 1);
            This_Block_Saved = YES;
            Block_Comm_Timer[x] = 0;            // zero comm timer for this block
          }
          x--;
        }
        x = NUMBER_OF_BLOCKS;
        while ( x > 0 ) {
          if (This_Block_Saved != YES) {        // if not saved, save BLOCK to zeroed location
            if (blockNum[x - 1] == 0) {         // not saved so store this block in an unused or zeroed location
              blockNum[x - 1] = from;
              VERBOSE_PRINT("success saving Block#: "); VERBOSE_PRINT(from);
              VERBOSE_PRINT(" into RAM location: "); VERBOSE_PRINTLN(x - 1);
              goto exitsaveblock;
            }
          }
          x--;
        }
      }
      else {                              // LEARN mode is off, so check incoming block number with saved blocks
        x = NUMBER_OF_BLOCKS;
        while ( x > 0 ) {
          if (from == blockNum[x - 1]) {  // check if BLOCK already been saved?
            VERBOSE_PRINT("This block is ALREADY SAVED: "); VERBOSE_PRINTLN(x - 1);
            This_Block_Saved = YES;
            Block_Comm_Timer[x-1] = 0;    // zero comm timer for this block
          }
          x--;
        }
      }
      exitsaveblock:
      // VERBOSE_PRINT("Blocknum Saved: "); VERBOSE_PRINTLN(This_Block_Saved);
      if (This_Block_Saved) {             // if data is sent by a DKblock in this pack, read the data
        VERBOSE_PRINTLN(" USE this GOOD data");
        VERBOSE_PRINT(((DATA*)buf) -> sCellV_Hiside ); VERBOSE_PRINT(" VDC Hi Cell   ");
        VERBOSE_PRINT(((DATA*)buf) -> sThottest);      VERBOSE_PRINT(" Hot NTC counts   ");
        VERBOSE_PRINT(((DATA*)buf) -> sTcoldest);      VERBOSE_PRINT(" Cold NTC counts   ");
        VERBOSE_PRINT(((DATA*)buf) -> sCellV_Loside ); VERBOSE_PRINT(" VDC Lo Cell   ");
        VERBOSE_PRINT(((DATA*)buf) -> schecksum );     VERBOSE_PRINTLN(" CHECKSUM");

        Temp1 = (((DATA*)buf) -> sCellV_Hiside );
        Temp2 = (((DATA*)buf) -> sCellV_Loside );
        Temp3 = (((DATA*)buf) -> sThottest );
        Temp4 = (((DATA*)buf) -> sTcoldest );

        VERBOSE_PRINTLN(" High resolution mode values: ");
        VERBOSE_PRINT_3(Temp1); VERBOSE_PRINT(" VDC Hi Cell   ");
        VERBOSE_PRINT_3(Temp2); VERBOSE_PRINTLN(" VDC Lo Cell");
      }
      else {
        VERBOSE_PRINTLN(" This is not a 'saved' block - DO NOT USE this BAD data");
        goto badcomm;
      }
      // VERBOSE_PRINTLN((char*)buf);
      // Send a reply back to the originator client
      // if (!manager.sendtoWait(data, sizeof(data), from))
      // VERBOSE_PRINTLN("sendtoWait failed");
    }
    else {
      VERBOSE_PRINTLN();
      CommsFaults ++;
    }
  }
  else {
    CommsFaults ++;               // after so many comm faults do a re-init, in case nRF24 board is locked up
    if (CommsFaults > 100) {
      CommsFaults = 0;
      //Tim find way to reset RF24 if faults gets too high. Reaches 100 every 30 secs!
      //if (!manager.init())  VERBOSE_PRINT("Comms init failed-"); // careful here this is a complete reboot
      //else  VERBOSE_PRINT("Comms init success-");
    }
    VERBOSE_PRINT("Comms faults = "); VERBOSE_PRINT(CommsFaults);
    digitalWrite(LED2red, HIGH);
  }
  // nRF24 2.4Mhz packet comms 
  digitalWrite(LED2green, LOW);
  digitalWrite(LED2red, LOW);

  // now range check data
  if ((Temp1 > 8) || (Temp1 < 0)) goto badcomm;       // cellV >8V and <0V out of range do not process the data
  if ((Temp2 > 8) || (Temp2 < 0)) goto badcomm;
  // Target out of range about -30C and +145C, after that who cares?
  if ((Temp3 > 3900) || (Temp3 < 10)) goto badcomm;   
  if ((Temp4 > 3900) || (Temp4 < 10)) goto badcomm;

  // Find the highest and lowest cell voltages and the highest and lowest cell temperatures
  // preload vars at first power up for first 10 seconds
  /*  if ((seconds < 10) && (minutes == 0) && (hours == 0) ) {
      Highest_Vcell = Vcell_Nominal_Spec;        // load  vars with nominal values
      Lowest_Vcell = Vcell_Nominal_Spec;
      Highest_Tcell = NTC_AMBIENT;
      Lowest_Tcell = NTC_AMBIENT;
      Hist_Highest_Vcell = Highest_Vcell;        // running average of all blocks highest cell v
      Hist_Lowest_Vcell = Lowest_Vcell;
      Hist_Highest_Tcell = NTC_AMBIENT;          // running ave of ALL blocks highest cell temperature (seed with nominal 25C)
      Hist_Lowest_Tcell = NTC_AMBIENT;           // seed with nom 25C
    }
  */
  //once you have good comms with known blocks ...use that good data (until then use defaults from startup)
  if (GoodComms) {
    if (Temp1 >= Temp2) {    // find highest/lowest Vcell - peak detector
      // reaction time too slow...speed up by less averaging  -data is better now...
      // if (Temp1 > Highest_Vcell) Highest_Vcell = ((Highest_Vcell + Highest_Vcell + Highest_Vcell + Temp1) / 4) ;
      // if (Temp2 < Lowest_Vcell) Lowest_Vcell = ((Lowest_Vcell + Lowest_Vcell + Lowest_Vcell + Temp2) / 4) ;
      if (Temp1 > Highest_Vcell) Highest_Vcell = ((Highest_Vcell + Temp1) / 2) ;  // average with last reading
      if (Temp2 < Lowest_Vcell) Lowest_Vcell = ((Lowest_Vcell + Temp2) / 2) ;
      // if (gMode == CHARGEMODE) { // voltages are rising
      //  if (Temp1 > Highest_Vcell) Highest_Vcell = ((Highest_Vcell + Temp1) / 2) ;  // average with last reading
      //  }
    }
    else {
      //if (Temp2 > Highest_Vcell) Highest_Vcell = ((Highest_Vcell + Highest_Vcell + Highest_Vcell + Temp2) / 4) ;
      // insure that one bad byte does not change the value
      //if (Temp1 < Lowest_Vcell) Lowest_Vcell = ((Lowest_Vcell + Lowest_Vcell + Lowest_Vcell + Temp1) / 4) ;
      if (Temp2 > Highest_Vcell) Highest_Vcell = ((Highest_Vcell + Temp2) / 2);  // insure that one bad byte does not change the value
      if (Temp1 < Lowest_Vcell) Lowest_Vcell = ((Lowest_Vcell + Temp1) / 2);
    }
    VERBOSE_PRINT(" Highest Vcell: "); VERBOSE_PRINT_3(Highest_Vcell);  VERBOSE_PRINT(" VDC");
    VERBOSE_PRINT(" Lowest Vcell: ");  VERBOSE_PRINT_3(Lowest_Vcell);  VERBOSE_PRINT(" VDC");  VERBOSE_PRINTLN();

    // find highest and lowest Tcell --- hotter is lower counts
    if ((Highest_Tcell > Temp3)||(Highest_Tcell == 0)) Highest_Tcell = ((Highest_Tcell + Highest_Tcell + Highest_Tcell + Temp3) / 4 );
    if (Lowest_Tcell < Temp4) Lowest_Tcell = ((Lowest_Tcell + Lowest_Tcell + Lowest_Tcell + Temp4) / 4);
    //    if (Lowest_Tcell < Temp4) Lowest_Tcell = Temp4;
    VERBOSE_PRINT(" Hottest Tcell: ");     VERBOSE_PRINT_1(Highest_Tcell);  VERBOSE_PRINT(" ADC counts");
    VERBOSE_PRINT(" Coolest Tcell: ");     VERBOSE_PRINT_1(Lowest_Tcell);  VERBOSE_PRINT(" ADC counts"); VERBOSE_PRINTLN ();

    // Running average of cell parameters:
    if (HistoryTimer == 0) {
      HistoryTimer = T_HISTORYCHECK;    // set timer for next check
      // take a voltage sample and make running average over last n sec
      Hist_Highest_Vcell = (Highest_Vcell + Hist_Highest_Vcell + Hist_Highest_Vcell ) / 3;
      Hist_Lowest_Vcell = (Lowest_Vcell + Hist_Lowest_Vcell + Hist_Lowest_Vcell) / 3;
      Hist_Highest_Tcell = (Highest_Tcell +  Hist_Highest_Tcell + Hist_Highest_Tcell) / 3;
      Hist_Lowest_Tcell = (Lowest_Tcell + Hist_Lowest_Tcell + Hist_Lowest_Tcell) / 3;
      // reaction time too slow...speed up by less averaging  -data is better now...
      // Hist_Highest_Vcell = (Highest_Vcell + Hist_Highest_Vcell + Hist_Highest_Vcell + Hist_Highest_Vcell) / 4;
      // Hist_Lowest_Vcell = (Lowest_Vcell + Hist_Lowest_Vcell + Hist_Lowest_Vcell + Hist_Lowest_Vcell) / 4;
      // Hist_Highest_Tcell = (Highest_Tcell + Hist_Highest_Tcell + Hist_Highest_Tcell + Hist_Highest_Tcell) / 4;
      // Hist_Lowest_Tcell = (Lowest_Tcell + Hist_Lowest_Tcell + Hist_Lowest_Tcell + Hist_Lowest_Tcell ) / 4;
 
      // reset the peak detectors (discharge the software capacitor)
      if (seconds > 10) {                                       // well after power up
        // if (Highest_Vcell > Vcell_Nominal_Spec) Highest_Vcell = (Highest_Vcell - 0.001);
        // if (Lowest_Vcell < Vcell_Nominal_Spec) Lowest_Vcell = (Lowest_Vcell + 0.001);
        Highest_Vcell = (Highest_Vcell - 0.001);                // discharge away from peak
        Lowest_Vcell = (Lowest_Vcell + 0.001);                  // discharge away from peak
        if (Highest_Tcell < NTC_AMBIENT)  Highest_Tcell ++;     // hottest goes toward colder
        if (Lowest_Tcell > NTC_AMBIENT) Lowest_Tcell --;        // coldest goes toward hotter
      }
    }
    VERBOSE_PRINT(" Historical Avg Highest Vcell: ");  VERBOSE_PRINT_3(Hist_Highest_Vcell);  VERBOSE_PRINT(" VDC ");
    VERBOSE_PRINT(" Historical Avg Lowest Vcell: ");   VERBOSE_PRINT_3(Hist_Lowest_Vcell);   VERBOSE_PRINTLN(" VDC");
    VERBOSE_PRINT(" Historical Avg Hottest Tcell: ");  VERBOSE_PRINT_3(Hist_Highest_Tcell);  VERBOSE_PRINT(" ADC counts ");
    VERBOSE_PRINT(" Historical Avg Coolest Tcell: ");  VERBOSE_PRINT_0(Hist_Lowest_Tcell);   VERBOSE_PRINTLN(" ADC counts");
  }
badcomm: ;                                            // bad comm bytes end up here...

const uint16_t NO_BARS = 100;                         // 1% PWM = (5 / 3.3) * 0.01 * 1024 = 15  ( 0 bars > 4.32)
const uint16_t ONE_BARS = 153;                        // 14% PWM = (5 / 3.3) * 0.14 * 1024 = 217 (1 bars < 4.07V)
const uint16_t TWO_BARS = 225;                        // 18% PWM = (5 / 3.3) * 0.18 * 1024 = 279 (2 bars < 3.7V)
const uint16_t THREE_BARS = 400;                      // 22% PWM = (5 / 3.3) * 0.22 * 1024 = 341 (3 bars < 3.2V)
const uint16_t FOUR_BARS = 707 ;                      // 26% PWM = (5 / 3.3) * 0.26 * 1024 = 404 (4 bars < 2.3V)

  // setup linear DAC0 output for KOSO Speedo Fuel gauge (5/3.3 = compensates for PWM testing done with Funct gen)
  if (SOCv >= 20) analogWrite(DAC_OUT, ONE_BARS);     // Need equiv of 14% PWM for 1 bars
  else analogWrite(DAC_OUT, (NO_BARS));               // No bars give *some* signal
  if (SOCv >= 40) analogWrite(DAC_OUT, TWO_BARS);     // Need equiv of 18% PWM for 2 bars
  if (SOCv >= 60) analogWrite(DAC_OUT, THREE_BARS);   // Need equiv of 22% PWM for 3 bars
  if (SOCv >= 80) analogWrite(DAC_OUT, FOUR_BARS);    // Need equiv of 26% PWM for all 4 bars

  // Block awareness: check to see which blocks are talking, over a 3 min window
} // Main Loop End


