
// ================= CHANGE LCD MODEL HERE ====================0
//#define LCD_NEWHAVEN 0       // NOT FULLY OPERATIONNAL !
#define LCD_2004A 1




#include <Wire.h>
#ifdef LCD_2004A
#include "LiquidCrystal_I2C.h"
#endif

#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdbool.h>


//---- MIN/MAX entry values and pressure limits ----//

#define MIN_BPM 8                       // in breath/minute
#define MAX_BPM 30                      // in breath/minute
#define DELTA_BPM (MAX_BPM-MIN_BPM)     // used to optimize calculus

#define MIN_VT 200                      // in ml
#define MAX_VT 800                      // in ml
#define DELTA_VT (MAX_VT-MIN_VT)/100    // In ml, used to optimize calculus

#define MIN_IE 1                        // in ratio E/I
#define MAX_IE 3                        // in ratio E/I
#define DELTA_IE (MAX_IE-MIN_IE)        // In ratio, used to optimize calculus

#define MIN_PEEP 1                      // in cmH2O
#define MAX_PEEP 20                     // in cmH2O
#define DELTA_PEEP (MAX_PEEP-MIN_PEEP)  // In cmH2O, used to optimize calculus
#define PEEP_MARGIN 3                 // in cmH2O, added pressure value to PEEP to start at initialization


//---- Fixed parameters ----//

// Mechanical

#define DISTANCE_PER_PULSE 0.0125                                       // In mm, linear motor displacement for each pulse 
#define SQUEEZED_VOLUME    1000                                         // In ml, total volume when bag squeezed (not total bag volume)
//#define TOTAL_RANGE        110                                          
#define TOTAL_RANGE        50                                           // In mm
#define VOLUME_PER_DISTANCE    SQUEEZED_VOLUME/TOTAL_RANGE              // In ml/mm (approximated total volume 1L/20cm approximated total range)
#define VOLUME_PER_PULSE       VOLUME_PER_DISTANCE*DISTANCE_PER_PULSE   // In ml/pulse
#define MOTOR_POSITION_LIMIT_INIT_MM       170                           // Max init range before error in mm 
#define MOTOR_POSITION_LIMIT_INIT_PULSE   MOTOR_POSITION_LIMIT_INIT_MM/DISTANCE_PER_PULSE  // Max init range before error in pulses
#define MOTOR_PULSE_WIDTH   4                                      // In us, note that due to the execution time, the real value will be a little bit higher (few us more ) 


// Pressure sensor
#define PRESSURE_SENSOR_POS_DYNAMIC 163 // Pressure sensor positive dynamic in cmH2O
#define MAX_PRESSURE 38                 // Max pressure at any moment in cmH2O


#define ALARM_FREQUENCY 4000            // Alarm sound frequency  
#define ALARM_SPEED 10000               // Repetitive speed of alarm

#define STATE_OK 0                      // System state ok
#define STATE_ERR_HIGH_PRESSURE 1       // Overpressure occured, need acknowledgement
#define STATE_ERR_LOW_PRESSURE 2        // Low pressure occured, need acknowledgement
#define STATE_ERR_FAIL_INIT 3           // Initialisation failure occured
#define STATE_ERR_MOTOR 4               // Motor failure detected 

#define ON 1
#define OFF 0
#define BUTTON_PUSHED 0
#define BUTTON_RELEASED 1



//---- Machine state definition ----//

#define STOP 0
#define SET_INIT 1
#define INIT 2
#define SET_INHALE 3
#define INHALE 4
#define SET_PAUSE 5
#define PAUSE 6
#define SET_EXHALE 7
#define EXHALE 8


//---- Pins definition ----//

//Analog inputs
#define PIN_BPM_IN A0
#define PIN_VT_IN A1
#define PIN_IE_IN A2
#define PIN_PRESSURE_IN A3
// A4 AND A5 RESERVED FOR I2C


//Digital inputs
#define PIN_SWITCH_ONOFF 2
#define PIN_SWITCH_ERR_ACK 3

#define PIN_WAREA 4        // Motor position between W-AREA1 and W-AREA2 (defined in driver)
#define PIN_BUSY 5         // If ON, Motor is positioning 
#define PIN_SETON 6         //  ON, if motor went back once to origin and knows his position
#define PIN_INP 7         //  ON, if motor at desired position



//Digital outputs
#define PIN_BUZZER 9
#define PIN_MOTOR_STEP_BACK 10
#define PIN_MOTOR_STEP_FORTH 11
#define PIN_MOTOR_ORIGIN 12
#define PIN_LED 13




//------- Functions definitions -------//

#ifdef LCD_NEWHAVEN

void LCDsetCursor(int Column, byte Line);
void LCDWrite(const char* text);
void LCDclear();

#endif

void SendPulse(int pin, int timeus);                         // Motor pulse function
void ActiveCount1(float timetointerruptus, long Ninterrupt); // Enable Counter 2 with interruption after timetointerrupt second
void StopCount1();

int interrtime1;                        // global variable to reload the counter
long Ninter1 = 0;                       // Number of interrupt 
int cnt1 = 0;                           // interruption counter




//----------- User Inputs variables declaration -----------//

float bpm = 0;    // In breath/minute
float IE = 0;   // Inspiration/expiration ratio 1:x
int VT = 0;     // In pulses   //MAY CHANGE
int peep = 5;     // In cmH20
bool start = 1; // ON/OFF switch


//----------- Measured variables declaration -----------//
int pressure = 15; // in cmH2O
int plateau_pressure = 0; // in cmH2O


//----------- Internal variables declaration -----------//
float T = 0;    // INHALE/EXHALE period in seconds
float Tin = 0;  // INHALE period in seconds
float Tex = 0;  // EXHALE period in seconds
float Vin = 0; // flowrate in pulses/second   //MAY CHANGE
float Vex = 0; // flowrate in pulses/second   //MAY CHANGE
float Psin = 0; // time/pulse  //MAY CHANGE
float Psex = 0; // time/pulse    //MAY CHANGE
long Npulse = 0; // Total cycle pulses    //MAY CHANGE

float Tplateau = 150; // Pause time to measure plateau pressure in millisec

int state = STOP; // Initial system state machine

byte error_state = STATE_OK; // System error state

bool restarted = 0;

bool first_stop = 1;

bool firststop = 1;

float Tinitmax = 5; // Max time to init before sending an error

bool initialized = 0;

int cnt_alarm = 0;

int cnt_positioning;


//----------- LCD management variable -----------//
int lastbpm = 0;
int lastpressure = 0;
int lastplatpressure = 0;
int lastpeep = 0;
bool laststate = 1;
int blink_error = 0;
bool printerror = 0;

#ifdef LCD_2004A
LiquidCrystal_I2C lcd(0x27, 20, 4); //Set the LCD address to 0x27 for a 20 chars and 4 lines display
#endif

#ifdef LCD_NEWHAVEN
const byte LCDa = 0x28; //LCD address on I2C bus
#endif


//----------- Motor driver variable declaration -----------//
long PEEP_relative_origine = 0;
int motorposition = 0;


//----------- Debug variable declaration -----------//
long timestart = 0;
long interuptcnt = 0;




// PCIF------ Type declaration of runtime data for GEVE Ventilator Monitoring ---------- //
struct GEVE_VeMon_Data
{
  bool on_off;
  bool error_ack;
  double breath_pm;
  double tidal_volume;
  double in_ex_ratio;
  double peep;
  double pressure;
  double pressure_plateau;
  double motor_position;
  bool alarm_p_high;
  bool alarm_p_low;
  bool alarm_init;
};


// PCIF------ Variable definition of type GEVE runtime data for communication ---------- //
GEVE_VeMon_Data geve_status =
{
  false,
  false,
  0.0,
  0.0,
  0.0,
  0.0,
  0.0,
  0.0,
  0.0,
  false,
  false,
  false
};









void setup()
{
  //----------- I/O init. -----------//

  // Input
  pinMode(PIN_SWITCH_ONOFF, INPUT_PULLUP);
  pinMode(PIN_SWITCH_ERR_ACK, INPUT_PULLUP);
  pinMode(PIN_WAREA, INPUT);
  pinMode(PIN_BUSY, INPUT);
  pinMode(PIN_SETON, INPUT);
  pinMode(PIN_INP, INPUT);

  //TIMSK0 = 0;

  //Output
  pinMode(PIN_BUZZER, OUTPUT);
  pinMode(PIN_MOTOR_STEP_BACK, OUTPUT);
  pinMode(PIN_MOTOR_STEP_FORTH, OUTPUT);
  pinMode(PIN_MOTOR_ORIGIN, OUTPUT);
  pinMode(PIN_LED, OUTPUT);

  digitalWrite(PIN_MOTOR_STEP_BACK, LOW);
  digitalWrite(PIN_MOTOR_STEP_FORTH, LOW);
  digitalWrite(PIN_MOTOR_ORIGIN, HIGH);

  //------------Serial comm. init.------------//
  Serial.begin(9600);

  // ----------- LCD init.------------//
#ifdef LCD_2004A
  lcd.begin();
  lcd.backlight();
  lcd.clear();

  lcd.setCursor(0, 1);
  lcd.print("BPM:");
  lcd.setCursor(8, 1);
  lcd.print("Tid.Vol.:");
  lcd.setCursor(0, 2);
  lcd.print("IE:");
  lcd.setCursor(12, 2);
  lcd.print("PEEP.:");
  lcd.setCursor(0, 3);
  lcd.print("Press:");
  lcd.setCursor(10, 3);
  lcd.print("PressPl:");
#endif

#ifdef LCD_NEWHAVEN

  TWBR = 100000; //sets I2C speed to 100kHz very important
  LCDclear();
  //delay(500);     // for observation only can be removed

  LCDsetCursor(0, 1);
  LCDWrite("BPM:");
  LCDsetCursor(8, 1);
  LCDWrite("Tid.Vol.:");
  LCDsetCursor(0, 2);
  LCDWrite("IE:");
  LCDsetCursor(12, 2);
  LCDWrite("PEEP.:");
  LCDsetCursor(0, 3);
  LCDWrite("Press:");
  LCDsetCursor(10, 3);
  LCDWrite("PressPl:");

#endif

}
int starttimemesure = 0;
void loop()
{
  // ================START OF Read potentiometers & display values ======================//

  bpm = analogRead(PIN_BPM_IN) * DELTA_BPM / 1024 + MIN_BPM;

  VT = analogRead(PIN_VT_IN) * DELTA_VT / 10 + MIN_VT;

  IE = (float)(analogRead(PIN_IE_IN)) * DELTA_IE / 1024 + MIN_IE;

  // read pressure
  pressure = ((analogRead(PIN_PRESSURE_IN) - 512) * PRESSURE_SENSOR_POS_DYNAMIC / 1024) * 5 / 2 ;
