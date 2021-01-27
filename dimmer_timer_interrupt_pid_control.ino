#include <Wire.h> 
#include <LiquidCrystal_I2C.h>
#include  <TimerOne.h>          
LiquidCrystal_I2C lcd(0x27, 16, 2);

volatile int i=0;               
volatile boolean zero_cross=0;  
int AC_pin = 11;                
int dim;                    
int inc=1;                      

int freqStep = 75;    

int setpoint;
float sensor;
float error;
int P_control;
int kp = 6;
float I_control;
float ki = 0.07;
float PI_control;
int i_windup;
int saturation = 128;
int power;

float Time;
float elapsedTime;
float timePrev;

void setup() {                                      // Begin setup
  // initialize the LCD
  lcd.begin();

  // Turn on the blacklight and print a message.
  lcd.backlight();
  
  pinMode(AC_pin, OUTPUT);                          // Set the Triac pin as output
  attachInterrupt(0, zero_cross_detect, RISING);    // Attach an Interupt to Pin 2 (interupt 0) for Zero Cross Detection
  Timer1.initialize(freqStep);                      // Initialize TimerOne library for the freq we need
  Timer1.attachInterrupt(dim_check, freqStep);      
  Serial.begin(9600);                                            
}

void zero_cross_detect() {    
  zero_cross = true;               // set the boolean to true to tell our dimming function that a zero cross has occured
  i=0;
  digitalWrite(AC_pin, LOW);       // turn off TRIAC (and AC)
}                                 

// Turn on the TRIAC at the appropriate time
void dim_check() {                   
  if(zero_cross == true) {              
    if(i>=dim) {                     
      digitalWrite(AC_pin, HIGH);        
      i=0;                       
      zero_cross = false; 
    } 
    else {
      i++;                      
    }                                
  }                                  
}                                   

void loop() {                        
  /*
  dim+=inc;
  if((dim>=128) || (dim<=0))
    inc*=-1;*/

    
 setpoint = 40;
  //setpoint = map(analogRead(A1), 0, 1023, 33, 40);
   //read sensor
 sensor = ((0.1391*analogRead(A0))-46.893);
 //sensor = sensor;
   //error
 error = setpoint - sensor;
   //proportional control
 P_control = kp * error;

   //integral control
 I_control = ki*error*elapsedTime + I_control;
  
   //integral windup
  if(I_control > i_windup){
    I_control = i_windup;
  } else if (I_control < 0){
    I_control = 0; 
  }
  else {
    I_control = I_control;
  }
  //p+i control
 PI_control = P_control + I_control;
  
   //saturation
  if (PI_control > saturation){
    PI_control = saturation;
    i_windup = saturation-PI_control;
  } else if(PI_control < 0) {
     PI_control = 0;
  }
  else{
    i_windup = saturation;
  }    
  
  //dim = 100; //debug
  
  dim = saturation - PI_control;
  power = map(dim, 128, 0, 0, 100);
  //display
  lcd.setCursor (0,0);
  lcd.print("Power : ");
  if(power < 10){
  lcd.setCursor (8,0);
  lcd.print(" ");  
  lcd.setCursor (9,0);
  lcd.print(power); 
  }
    if(power >= 10 && power < 100){
    lcd.setCursor (8,0);
    lcd.print(power);  
    }
    
    lcd.setCursor (11,0);
    lcd.print("%");
    lcd.setCursor (0,1);
    lcd.print("SP: ");
    lcd.setCursor(4,1);
    lcd.print(setpoint);
    lcd.setCursor (7,1);
    lcd.print("PV: ");
    lcd.setCursor(11,1);
    lcd.print(sensor);

    //TIMER
  timePrev = Time;                            // the previous time is stored before the actual time read
  Time = millis();                            // actual time read
  elapsedTime = (Time - timePrev) / 1000; 

  //serial debug
  Serial.print("setpoint : ");
  Serial.print(setpoint);
  Serial.print(" sensor : ");
  Serial.print(sensor);
  Serial.print(" error : ");
  Serial.print(error);
  Serial.print(" P : ");
  Serial.print(P_control);
  Serial.print(" I : ");
  Serial.print(I_control);
  Serial.print(" PI : ");
  Serial.print(PI_control);
  Serial.print(" power : ");
  Serial.println(power);
  
  delay(100);  
  
}
