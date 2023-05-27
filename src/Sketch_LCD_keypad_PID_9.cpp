/* Arduino sketch for PID temperature control
 * 
 * copyright 2017 R. A. Winters, All Rights Reserved   
 */

#define DEBUG // uncomment this if you want verbose console logging

#include <Arduino.h>
#include <DFRkeypad.h>
#include "max6675.h"
#include <PID_v1.h>  // similar to standard PID_v1, this custom library is required for full functionality
#include <avr/power.h>
#include <LiquidCrystal.h>
#define CLK_DIV 8    // 8
// define 2 exceptions (temp and pid). TEMP_ALARM is fatal - program end.
#define TEMP_ALARM_MIN -100 // exceeds min temp limit [F]
#define TEMP_ALARM_MAX 200 // exceeds max temp limit [F]

/* PID limits should reflect actual physical capability. */
#define PID_MIN -255  // exceeds min PID limit
#define PID_MAX  255  // exceeds max PID limit

#define TEMP_CORRECTION_OFFSET -1 // Thermocouple fudge factor [Fahrenheit]

//Function Definitions
//void t_PID(profile *r, double *T, unsigned long *t);
void LCD_update();
void SER_update();
void PWM_OUT(int);
void PWM_OUT(int);
void temp_alarm(double);
void pid_error();
void MainMenuDisplay();
void MainMenuBtn();
char ReadKeypad();
void WaitBtnRelease();

/*
double GetITerm();
void SetITerm(double);
double GetpTerm();
void SetpTerm(double);
double GetDTerm();
void setDTerm(double);
void SetPID(double, double, double);
*/

// MAX6675 + Thermocouple convert Arduino Analog pins to power and communication.
// K-thermocouple only, 0-1024C range, 12-bit, 0.25C resolution, ~250K conversions/sec. max.
int thermoD0 =  15; // A1
int thermoCS =  16; // A2
int thermoCLK = 17; // A3
int vccPin =    18; // A4
int gndPin =    19; // A5

// PWM on 2 pins (+/-) using Timer 2 (16 bits)
int PWM_Pin_plus = 11;
int PWM_Pin_minus = 3;

// PID variables we connect to
double Setpoint = 100, Input = 20, Output, PV; // degrees Fahrenheit, Celsius or anything else
double PIDSAMPLETIME_MS = 5000 ;  // milliseconds
double Kp = 0.1, Ki = 0.3, Kd = 0.5, K = 1; // K is fudge factor for Output Gain or Thermal Resistance

unsigned long stepN;

// menu
int keypad_pin = A0;
int keypad_value = 0;
int keypad_value_old = 0;
char btn_push;
short mainMenuPage = 1;         // byte
short mainMenuPageOld = 1;      // byte
const int mainMenuTotal = 6;    // 5 + 1

typedef struct {                     // 5 steps in temperature profile
  char *desc;                        // description 16 char max.
  double Kp, Ki, Kd;                 // PID parameters
  double T1, T2, T3, T4, T5;         // T temperature
  unsigned long t1, t2, t3, t4, t5;  // t time [integer minutes]
  } profile;
  
profile profiles[mainMenuTotal];
void t_PID(profile *r, double *T, unsigned long *t);

// create the objects
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);
MAX6675 thermocouple(thermoCLK, thermoCS, thermoD0);
LiquidCrystal lcd(8, 9, 4, 5, 6, 7);  // initialize the LCD library with the numbers of the interface pins

void setup() {
  #ifdef DEBUG
  Serial.begin(1200 * CLK_DIV);  // Change Serial Monitor tool datarate to 9600
  Serial.println("Arduino PID Temperature Control \n");
  Serial.println("********SETUP********* \n");
	#endif
  
  clock_prescale_set(clock_div_8);
  lcd.begin(16, 2);     // LCD's number of columns and rows
  lcd.setCursor(0, 0);  // Print a message to the LCD
  lcd.print("PID Temp Control");

  //keypad setup
  DFRkeypad::FastADC(true);                       // increase ADC sample frequency
  DFRkeypad::iDEFAULT_THRESHOLD=140;              // maximum threshold acceptable so bounds in DFRkeypad::iARV_VALUES are not overlapping

  // PID setup
  myPID.SetMode(AUTOMATIC);
  myPID.SetSampleTime(PIDSAMPLETIME_MS);
  myPID.SetOutputLimits(PID_MIN, PID_MAX);   // True PID heating and cooling. Should reflect actual physical limits
 
  // MAX6675 Temp. measurement setup
  pinMode(gndPin, OUTPUT); digitalWrite(gndPin, LOW);   // GND A5
  pinMode(vccPin, OUTPUT); digitalWrite(vccPin, HIGH);  // VCC A4
  delay(1000 / CLK_DIV);

  // Keypad setup
  DFRkeypad::FastADC(true);                       // increase ADC sample frequency
  DFRkeypad::iDEFAULT_THRESHOLD=140;              // maximum threshold acceptable so bounds in DFRkeypad::iARV_VALUES are not overlapping
  
  // phase-correct PWM proportional output,  Timer 2 on pins 3, 11
  TCCR2A = _BV(COM2A1) | _BV(COM2B1) | _BV(WGM20);
  //TCCR2B = _BV(CS22);
  //TCCR2B = TCCR2B & B11111000 | B00000001;    // set timer 2 divisor to     1 for PWM frequency of 31372.55 Hz
  //TCCR2B = TCCR2B & B11111000 | B00000010;    // set timer 2 divisor to     8 for PWM frequency of  3921.16 Hz
  //TCCR2B = TCCR2B & B11111000 | B00000011;    // set timer 2 divisor to    32 for PWM frequency of   980.39 Hz
  TCCR2B = TCCR2B & B11111000 | B00000100;    // set timer 2 divisor to    64 for PWM frequency of   490.20 Hz  <-- default
  //TCCR2B = TCCR2B & B11111000 | B00000101;    // set timer 2 divisor to   128 for PWM frequency of   245.10 Hz
  //TCCR2B = TCCR2B & B11111000 | B00000110;    // set timer 2 divisor to   256 for PWM frequency of   122.55 Hz
  //TCCR2B = TCCR2B & B11111000 | B00000111;    // set timer 2 divisor to  1024 for PWM frequency of    30.64 Hz
  pinMode(PWM_Pin_plus,  OUTPUT);
  pinMode(PWM_Pin_minus, OUTPUT);

  

// User Edit    "desc"    ,   Kp,  Ki,  Kd, T1,  T2,  T3,  T4,  T5,  t1,  t2,  t3,  t4,  t5 [t in integer minutes]
  profiles[0]= {"Test Profile"  ,  0.1, 0.3, 0.5,  350,   200,   150,   150,   100, 20, 20,   20,   10,   10};
  profiles[1]= {"Profile 1",  0.1, 0.3, 0.5, 100,  50, -20,  50,  30,  3,  3,  3,  3,  3};
  profiles[2]= {"Profile 2",  0.2, 0.3, 0.7, 30,   5,  40,  10,  50,   5,  60,  10,  70,   5};
  profiles[3]= {"Profile 3",  0.2, 0.4, 0.5, 20,   1,  30,  10,  40,   1,  50,  10,  60,   1};
  profiles[4]= {"Profile 4",  0.2, 0.4, 0.6, 20,   5,  30,   1,  40,   5,  50,   1,  60,   5};
  profiles[5]= {"Profile 5",  0.2, 0.4, 0.6, 20, 100,  30,  -2,  40,   2,  2,   1,  6,   5};

  #ifdef DEBUG
	Serial.println(" SETUP Done.  \n");
	#endif
}

void loop() {
    MainMenuDisplay();
    
    byte key=DFRkeypad::GetKey();                   // read a key identifier

    /*
    btn_push = ReadKeypad();
    WaitBtnRelease();
    MainMenuBtn();
    */

    #ifdef DEBUG
  	Serial.print("*  Key: ");
	  Serial.println(DFRkeypad::KeyName(key));
    #endif

    if(DFRkeypad::eNO_KEY<key && key<DFRkeypad::eINVALID_KEY) // if a valid key has been identified
    {
      int val=analogRead(KEYPAD);                   // read the analog value of the key
      DFRkeypad::KeyName(key);                      // print the key name

      #ifdef DEBUG
      Serial.print("   val: ");
      Serial.println(val);
      #endif

    }
    
    if(key == "Select")  // enter selected profile
    {
        myPID.SetTunings(profiles[mainMenuPage].Kp, profiles[mainMenuPage].Ki, profiles[mainMenuPage].Kd);
        t_PID(&profiles[mainMenuPage], &profiles[mainMenuPage].T1, &profiles[mainMenuPage].t1 );        
        t_PID(&profiles[mainMenuPage], &profiles[mainMenuPage].T2, &profiles[mainMenuPage].t2 );        
        t_PID(&profiles[mainMenuPage], &profiles[mainMenuPage].T3, &profiles[mainMenuPage].t3 );        
        t_PID(&profiles[mainMenuPage], &profiles[mainMenuPage].T4, &profiles[mainMenuPage].t4 );        
        t_PID(&profiles[mainMenuPage], &profiles[mainMenuPage].T5, &profiles[mainMenuPage].t5 );
        PWM_OUT(0);   // turn off forcing functions       
    }
      MainMenuDisplay();

} // close loop()

void t_PID(profile *r, double *T, unsigned long *t) {
  unsigned long startTime = millis();
  unsigned long t_div = 60000;  // for accelerated testing, 1000ms/s * 60s/min = 60000 ms/min
  myPID.mySetpoint = T;

  while ((millis() - startTime) * CLK_DIV / t_div < *t) {
    PV = thermocouple.readFahrenheit() + TEMP_CORRECTION_OFFSET;  // Process Value (current temperature)
    // Input = PV;  // uncomment to use for live operation
    if (Input <= TEMP_ALARM_MIN || Input >= TEMP_ALARM_MAX) { temp_alarm(Input); }
    LCD_update(); SER_update();  // update the displays
    myPID.Compute();
    if (Output <= PID_MIN || Output >= PID_MAX) { pid_error(); }
    Input = Input + K * Output;
    PWM_OUT(K * Output);   // update the control outputs
    stepN++;
    delay(PIDSAMPLETIME_MS / CLK_DIV);
  }  
}

void LCD_update() {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("t=");
  lcd.print(PIDSAMPLETIME_MS * stepN / 60000, 0);  // minutes
  lcd.print(" I=");
  lcd.print(myPID.GetKi(), 0);
  lcd.print(" D=");
  lcd.print(myPID.GetKd(), 0);
  lcd.setCursor(0, 1);
  lcd.print("PV=");
  lcd.print(Input, 1);
  lcd.print(" F");
  delay(100);
}

void SER_update() {
  Serial.print("step\t sec.\t PV\t in\t out\t ITerm\t DTerm\n");
  Serial.print(stepN);
  Serial.print("\t");
  Serial.print(PIDSAMPLETIME_MS * stepN / 1000, 0);  // seconds
  Serial.print("\t");
  Serial.print(PV);
  Serial.print("\t");
  Serial.print(Input);
  Serial.print("\t");
  Serial.print(Output);
  Serial.print("\t");
  Serial.print(myPID.GetKi());
  Serial.print("\t");
  Serial.print(myPID.GetKd());
  Serial.println("\n");
}

void PWM_OUT(int signal) {
  if (signal > 0 ) {
    analogWrite(PWM_Pin_plus, signal);
    analogWrite(PWM_Pin_minus, 0);
  } else {
    analogWrite(PWM_Pin_plus, 0);
    analogWrite(PWM_Pin_minus, 0-signal);
  }
}

void temp_alarm(double alarm_temp) {
  PWM_OUT(0);  // turn off forcing functions
  pinMode(PWM_Pin_plus, INPUT);  // disable output pins
  pinMode(PWM_Pin_minus, INPUT);
  Serial.print(alarm_temp);
  Serial.print("\nTemperature Alarm - Program End.\n");
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print(alarm_temp);
  lcd.setCursor(0, 1);
  lcd.print("TEMP ALARM - END");
  delay(100);
  exit(1);
}

void pid_error() {
  Serial.print("\nPID_Error.\n");
  lcd.setCursor(15, 1);
  lcd.print("E");
}

void MainMenuDisplay()
{
    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print("Scroll & Select");
    lcd.setCursor(0,1);
    lcd.print(profiles[mainMenuPage].desc);
    delay(1000 / CLK_DIV); 
}

void MainMenuBtn()
{
    WaitBtnRelease();
    if(btn_push == 'U')
    {
        mainMenuPage++;
        if(mainMenuPage > mainMenuTotal - 1)
          mainMenuPage = 0;
    }
    else if(btn_push == 'D')
    {
        mainMenuPage--;
        if(mainMenuPage < 0)
          mainMenuPage = mainMenuTotal - 1;    
    }
   
    if(mainMenuPage != mainMenuPageOld) //only update display when page change
    {
        MainMenuDisplay();
        mainMenuPageOld = mainMenuPage;
    }
}
 
char ReadKeypad()
{
  /* Keypad button analog Value
  no button pressed 1023
  select  741
  left    503
  down    326
  up      142
  right   0
  */
  keypad_value = analogRead(keypad_pin);
 
  if(keypad_value < 85)  // 100
    return 'R';
  else if(keypad_value < 200)
    return 'U';
  else if(keypad_value < 400)
    return 'D';
  else if(keypad_value < 600)
    return 'L';
  else if(keypad_value < 800)
    return 'S';
  else
    return 'N';
 
}
 
void WaitBtnRelease()
{
    while( analogRead(keypad_pin) < 800){}
}


