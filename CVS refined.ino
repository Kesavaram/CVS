
/* This program replicates a CVS machine by controlling two peltier devices one for each ear.
 *  PID is used to match the temperature of the peltier element to that of the waveform
 * Uses the AUTO_PID  library created by Ryan Downing version 1.0.3 , is present in the Arduino Library manager
 *  
 *  CVS, which stimulates the vestibular system, is self-administered via the portable TNM device, delivering thermal waveforms through 
 *  ear pieces in a headset (37° \C–42°C to one ear and 37 °C–17 °C to the other, over about 19 minutes). Ears given the slow warming and cooling 
 *  waveforms are switched every two days.
 * 
 * General information about the temperature of the two peltier is displayed on a serial LCD which recieves input
 * from pin zero of Arduino UNO
 *  
 *  Refer the site below for the orginial study done and the original waveform of the CVS device
 *  https://www.accessdata.fda.gov/cdrh_docs/reviews/DEN170023.pdf
 *  
 *  
 *  
 *  
 *  */

//https://github.com/fdebrabander/Arduino-LiquidCrystal-I2C-library



#include <AutoPID.h>

int buzzer = 12;
int seconds = 0;
int minutes = 0;
int ThermistorPin = 0;
int Vo;
float R1 = 100000;
float logR2, R2, T, Tc, Tf;
float A = 8.270926e-04, B = 2.088037e-04, C = 8.059052e-08;
double temperature_L, setPoint_L, outputVal_L, temperature_R, setPoint_R, outputVal_R;
long timer2 = 0;


long timer3 = 0;
int start = 0;
long timer4 = 0;
long timer5 = 0;


float Baseline_L = 37;
float Baseline_R = 37;
float L_peak = 42;
float R_peak = 17;
float duration_R = 74;//period of thermal waveform for the peltier responsible for the cold stimulation
float duration_L = 37;//period of thermal waveform for the peltier responsible for the hot stimulation
float counter_L = 0;
float counter_R = 0;
// rise time of the longer peak in seconds
//45 1 5


//Left peltier controller pins
#define TempDIrectionL 2
#define PwmL 3

//Right peltier controller pins
#define TempDIrectionR 4
#define PwmR 5


#define left 0
#define right 1




//pins
#define POT_PIN A0
//#define OUTPUT_PIN A1
#define TEMP_PROBE_PIN 5
//#define LED_PIN 6

#define TEMP_READ_DELAY 50 //ms delay for temperature measurement

//pid settings and gains
#define OUTPUT_MIN -255
#define OUTPUT_MAX 200
#define KP 45
#define KI  1
#define KD 800
//45 1 800
//  tu=2.4 seconds


// kp,ki and kd values for the left peltier temperature comtroller
#define KP_L 18
#define KI_L  0
#define KD_L 10000

// kp,ki and kd values for the left peltier temperature comtroller
#define KP_R 50
#define KI_R  2
#define KD_R 800

#define OUTPUT_MAX_L 150


// https://r-downing.github.io/AutoPID/
AutoPID myPID_L(&temperature_L, &setPoint_L, &outputVal_L, OUTPUT_MIN, OUTPUT_MAX_L, KP_L, KI_L, KD_L); //hot
AutoPID myPID_R(&temperature_R, &setPoint_R, &outputVal_R, OUTPUT_MIN, OUTPUT_MAX, KP_R, KI_R, KD_R);//cold

unsigned long lastTempUpdate; //tracks clock time of last temp update

//call repeatedly in loop, only updates after a certain time interval
//returns true if update happened
bool updateTemperature() {
  if ((millis() - lastTempUpdate) > TEMP_READ_DELAY) {
    Vo = analogRead(A1);
    R2 = 95000 / ((1024.0 / Vo - 1));
    logR2 = log(R2);
    T = (1.0 / (A + B * logR2 + C * logR2 * logR2 * logR2)); // We get the temperature value in Kelvin from this Stein-Hart equation
    Tc = T - 273.15;  // Convert Kelvin to Celsius
    temperature_L = Tc;

    Vo = analogRead(A0);
    R2 = 95000 / ((1024.0 / Vo - 1));
    logR2 = log(R2);
    T = (1.0 / (A + B * logR2 + C * logR2 * logR2 * logR2)); // We get the temperature value in Kelvin from this Stein-Hart equation
    Tc = T - 273.15;
    temperature_R = Tc;
    lastTempUpdate = millis();

    return true;
  }
  return false;
}


void setup() {
  pinMode(buzzer, OUTPUT);
  pinMode(13, OUTPUT);

  pinMode(TempDIrectionL, OUTPUT); // PWM
  pinMode(, OUTPUT); // direction


  pinMode(TempDIrectionR, OUTPUT);// PWM
  pinMode(PwmR, OUTPUT);// direction

  temperature_L = 37;
  temperature_R = 37;





  Serial.begin(9600);
  delay(100);
  Serial.write(17);
  delay(100); // for turning the backlight of the LCD in the ON state.
  Serial.write(12);//clear display
  Serial.print("CVS temp");
  Serial.write(13);//next line
  Serial.print("controller");
  delay(100);
  Serial.write(22);// no blink no cursor
  delay(2000);

  /* ///////lcd.begin();

    // Turn on the blacklight and print a message.
    lcd.backlight();
    lcd.setCursor(6, 0); //col and then row
    lcd.print("CVS");

    lcd.setCursor(0, 1);
    lcd.print("TEMP CONTROLLER");
    ////// delay(3000);
  */
  pinMode(POT_PIN, INPUT);
  //  pinMode(OUTPUT_PIN, OUTPUT);
  //  pinMode(LED_PIN, OUTPUT);


  ///////  Serial.print("Temperature");
  ///////  Serial.print(" PWM");
  ///////  Serial.print(" setPoint");

  /////// Serial.println("");
  while (!updateTemperature()) {} //wait until temp sensor updated

  //if temperature is more than 4 degrees below or above setpoint, OUTPUT will be set to min or max respectively
  // myPID.setBangBang(2.5);/////////////////////*******************
  //set PID update interval to 4000ms
  myPID_L.setTimeStep(200);
  myPID_R.setTimeStep(200);

  digitalWrite(TempDIrectionL, LOW);
  analogWrite(PwmL, LOW);



  digitalWrite(TempDIrectionR, LOW);
  digitalWrite(PwmR, LOW);



}//void setup


void loop() {
  updateTemperature();
  //Serial.print("temp=");


  if (temperature_L < 12 || temperature_R < 12 || temperature_L > 50 || temperature_R > 50 ) // if temperature measured by either temperature is outside of normal range 
  //then a fault in sensor function is deteced and operation of device stops 
  {
    Serial.write(12);
    temp_regulator_2(0, left);
    temp_regulator_2(0, right);

    while (1)
    {

      Serial.print("temp sensor");
      Serial.write(13);
      Serial.print("error");
      digitalWrite(buzzer, HIGH);
      delay(500);
      Serial.write(12);// clear display

      digitalWrite(buzzer, LOW);
      delay(500);
    }
  }


  if (millis() - timer5 >= 150)
  {
    myPID_L.run();
    myPID_R.run();
  }

  if (millis() - timer2 >= 1000)
  { display_serial_LCD();

    if (start == 0)
    {
      setPoint_L = 37;
      setPoint_R = 37;
    }


    temp_regulator_2(outputVal_L, left);
    temp_regulator_2(outputVal_R, right);

    //  temp_regulator_2(255, left);
    // temp_regulator_2(255, right);


    //Serial.print("temp=");
    //   Serial.print(k);
    //  Serial.print(" ");
    //////////////    Serial.print(temperature_L);
    ///////////////    Serial.print(" ");
    //////////////     Serial.print(setPoint_L);
    ///////////    Serial.print(" ");
    // Serial.print(l);
    //   Serial.print(" ");

    //////////////  Serial.print(outputVal_L);

    ////////////////////       Serial.print("           ");
    //   Serial.print(" ");
    //   Serial.print(" ");
    /*  Serial.print(temperature_R);
           Serial.print(" ");
       Serial.print(setPoint_R);
        Serial.print(" ");
        //   Serial.print(k);
        //   Serial.print(" ");
       Serial.print(outputVal_R);
        //   Serial.print(" ");
      Serial.println("");*/ // computer output

    timer2 = millis();
  }
  if (millis() - timer3 >= 55000 ) // waits for 55 seconds for both peltier to reach 37 degree celsius
  { if (start == 0)
    { start = 1;

      /* Serial.println("******************Start =1 ***************");
        Serial.print("Actual_H Setpoint_H PWM_H Actual_C Setpoint_C PWM_C");
        Serial.println("");*/
      timer3 = millis();

    }
  }

  if (start == 1 && millis() - timer3 >= 2000) // starts the waveform after two seconds
  {
    if (millis() - timer4 >= 1000) // updates the setpoint of both peltier every second
    {
      if (counter_R <= duration_R)
      {
        setPoint_R = Baseline_R + ((R_peak - Baseline_R) / duration_R) * counter_R;
        

      }
      else
      {
        setPoint_R = R_peak - ((R_peak - Baseline_R) * (counter_R - duration_R) / duration_R);
        
      }

      if ( counter_L <= duration_L)
      {
        setPoint_L = Baseline_L + ((L_peak - Baseline_L) / duration_L) *  counter_L;

      }
      else
      {
        setPoint_L = L_peak - ((L_peak - Baseline_L) * ( counter_L - duration_L)) / duration_L ;


      }




      counter_R++;
      timer4 = millis();
      if (abs(counter_L - (2 * duration_R)) < 0.3) //  resets the counter_R value to restart the thermal waveform for cold stimulation
      { counter_R=0;
      }

      counter_L++;
      if (abs( counter_L - (2 * duration_L)) < 0.3)//  resets the counter_L value to restart the thermal waveform for hot stimulation
      {  counter_L = 0;
      }
    }


  }




}
//void loop



int current_measure() // optional feature only for those motor controller which support current measurement
{
  float  mot1_ADC;
  float  mot1_voltage;
  //  mot1_ADC = analogRead(CURRENT_SENSE);

  mot1_voltage =  mot1_ADC * (5.0 / 1024);
  // Serial.print("Motor 1 Current: ");
  //Serial.print (mot1_voltage * 26*100);
  // Serial.println (" mA");
  return mot1_voltage * 26 * 100l; // temp controller current in ma
}

void  display_serial_LCD()
{
  int seconds = 0;
  int minutes = 0;

  seconds = millis() / 1000;
  minutes = seconds / 60;
  seconds = seconds % 60;

  if (minutes >= 25) // cycle time, after which the program stops
  {
    temp_regulator_2(0, left); // cuts power to the left peltier 
    temp_regulator_2(0, right);// cuts power to the right peltier 

    Serial.write(12);// clear display
    while (1) // program exectuion gets stuck in this infinite loop after the program runs for 25 minutes.
    {

      Serial.print("Cycle Over");
      digitalWrite(buzzer, HIGH); // makes a beeping noise to indicate to the user that a cycle has finished.
      delay(500);
      Serial.write(12);// clear display

      digitalWrite(buzzer, LOW);
      delay(500);
    }
  }

  Serial.write(12);// clear display


  //  lcd.clear();
  if (start == 0)
  {
    Serial.print("Warm up ");



    if (minutes <= 9)
    {
      Serial.print("0");
    }
    Serial.print(minutes);

    Serial.print(":");
    if (seconds <= 9)
      Serial.print("0");
    Serial.print(seconds);

  }
  else
  {
    Serial.print("Active ");


    if (minutes <= 9)
    {
      Serial.print("0");
    }
    Serial.print(minutes);

    Serial.print(":");
    if (seconds <= 9)
      Serial.print("0");
    Serial.print(seconds);
  }



  //col and then row
  Serial.write(13);//next line

  Serial.print("H:");
  Serial.print(temperature_L);
  Serial.print(" C:");
  Serial.print(temperature_R);



}


void temp_regulator_2(int pwm, int device)
{
  if (device == right) // the one responsible for cooling
  {
    if (pwm > 0)
    { digitalWrite(TempDIrectionL, HIGH);
      analogWrite(PwmL, pwm);
    }
    else if (pwm < 0)
    {
      digitalWrite(TempDIrectionL, LOW);
      analogWrite(PwmL, -pwm);
    }
    else
    {
      digitalWrite(TempDIrectionL, LOW);
      analogWrite(PwmL, 0);
    }
  }
  if (device == left) // the one responsible for heating
  {
    if (pwm > 0)
    { digitalWrite(TempDIrectionR, HIGH);
      analogWrite(PwmR, pwm);
    }
    else if (pwm < 0)
    {
      digitalWrite(TempDIrectionR, LOW);
      analogWrite(PwmR, -pwm);
    }
    else
    {
      digitalWrite(TempDIrectionR, LOW);
      analogWrite(PwmR, 0);
    }
  }


}
