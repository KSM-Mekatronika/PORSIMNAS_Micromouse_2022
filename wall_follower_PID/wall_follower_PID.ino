#include <NewPing.h>
#include <LiquidCrystal_I2C.h>
#define TRIGGER_PIN1    3      // Arduino pin tied to trigger pin on the ultrasonic sensor.
#define ECHO_PIN1       2      // Arduino pin tied to echo pin on the ultrasonic sensor.
#define TRIGGER_PIN2    5      // Arduino pin tied to trigger pin on the ultrasonic sensor.
#define ECHO_PIN2       6       // Arduino pin tied to echo pin on the ultrasonic sensor.
#define MAX_DISTANCE    200    // Maximum distance we want to ping for (in centimeters). Maximum sensor distance is rated at 400-500cm.
#define noOfButtons 3    //Exactly what it says; must be the same as the number of elements in buttonPins
#define bounceDelay 20    //Minimum delay before regarding a button as being pressed and debounced
#define minButtonPress 3  //Number of times the button has to be detected as pressed before the press is considered to be valid

const int buttonPins[] = {A0, A1, A2};      // Input pins to use, connect buttons between these pins and 0V
uint32_t previousMillis[noOfButtons];       // Timers to time out bounce duration for each button
uint8_t pressCount[noOfButtons];            // Counts the number of times the button is detected as pressed, when this count reaches minButtonPress button is regared as debounced 
uint8_t testCount[noOfButtons];             //Test count, incremented once per button press
uint8_t count = 0, pwmUpper = 0, menuMode = 0;

LiquidCrystal_I2C lcd = LiquidCrystal_I2C(0x27, 16, 2);

int dir1  = 12;                // inisiai pin direct (arah putaran) untuk motor kiri
int pwm1  = 10;                // inisiai pin pwm (kecepatan putar) untuk motor kiri
int pwm2  = 11;                // inisiai pin direct (arah putaran) untuk motor kanan
int dir2  = 13;                // inisiai pin pwm (kecepatan putar) untuk motor kanan

int pwmA;
int pwmB;
int ka;
int ki;

uint8_t Upper = 255;               // Pwm maksimal yang diizinkan
uint8_t Lower = 15;                // Pwm minimal yang diizinkan

NewPing sonar1(TRIGGER_PIN1, ECHO_PIN1, MAX_DISTANCE);  // NewPing setup of pins and maximum distance.
NewPing sonar2(TRIGGER_PIN2, ECHO_PIN2, MAX_DISTANCE);  // NewPing setup of pins and maximum distance.
int sp;
void setup() 
{
  uint8_t i;
  Serial.begin(115200);         // Open serial monitor at 115200 baud to see ping results.
  pinMode(dir1, OUTPUT);      // Set pin untuk motor sebagai OUTPUT
  pinMode(pwm1, OUTPUT);
  pinMode(dir2, OUTPUT);
  pinMode(pwm2, OUTPUT);
  for (i = 0; i < noOfButtons; ++i) {
    pinMode(buttonPins[i], INPUT_PULLUP);
  }
  lcd.init();
  lcd.backlight();
}

void loop() 
{
  delay(50);
  unsigned int us = sonar1.ping();     // Send ping, get ping time in microseconds (uS).
  int kanan = us / US_ROUNDTRIP_CM;
  unsigned int us2 = sonar2.ping();    // Send ping, get ping time in microseconds (uS).
  int depan = us2 / US_ROUNDTRIP_CM;

  if(kanan>20){ 
    kanan=20;
  }else {
    kanan=kanan;
  }

  if (depan<=4){
    digitalWrite(dir1, HIGH);            
    digitalWrite(dir2, LOW);
    analogWrite(pwm1, 255 - 100);
    analogWrite(pwm2, 100);
  }else{
    sp = 5;                          
    int error = sp - kanan;           
    digitalWrite(dir1, HIGH);            
    digitalWrite(dir2, HIGH);
    int pid = 9*error;                   
    pwmA = sp*10-pid;
                        
    if(pwmA > Upper){
      pwmA = Upper;
    }      
    if(pwmA < Lower){
      pwmA = Lower;
    }
    ki = pwmA;                           //pwm kiri
    pwmB = sp*10+pid;
    if(pwmB > Upper){
      pwmB = Upper;
    }
    if(pwmB < Lower){
      pwmB = Lower;
    }
    ka = pwmB;                           //pwm kanan  
    analogWrite(pwm1, ka);
    analogWrite(pwm2, ki);
  }
  Serial.println("Depan : " + String(depan) + "  | Kanan : " + String(kanan) + "  | ki : " + String(ki) + "  | ka : " + String(ka));
  uint32_t currentMillis = millis();
  if(digitalRead(buttonPins[2])) {             //Input is high, button not pressed or in the middle of bouncing and happens to be high
    previousMillis[2] = currentMillis;        //Set previousMillis to millis to reset timeout
    pressCount[2] = 0;                        //Set the number of times the button has been detected as pressed to 0
  }else {
    if (currentMillis - previousMillis[2] > bounceDelay) {
      previousMillis[2] = currentMillis;        //Set previousMillis to millis to reset timeout
      ++pressCount[2];
      if (pressCount[2] == minButtonPress) {
          count = count + 1;                             //Button has been debounced. Call function to do whatever you want done.
      }
    }
  }

  if(count == 0){
    if(digitalRead(buttonPins[0])) {             //Input is high, button not pressed or in the middle of bouncing and happens to be high
      previousMillis[0] = currentMillis;        //Set previousMillis to millis to reset timeout
      pressCount[0] = 0;                        //Set the number of times the button has been detected as pressed to 0
    }else {
      if (currentMillis - previousMillis[0] > bounceDelay) {
        previousMillis[0] = currentMillis;        //Set previousMillis to millis to reset timeout
        ++pressCount[0];
        if (pressCount[0] == minButtonPress) {
          sp = sp + 1;                             //Button has been debounced. Call function to do whatever you want done.
        }
      }
    }
    if(digitalRead(buttonPins[1])) {             //Input is high, button not pressed or in the middle of bouncing and happens to be high
      previousMillis[1] = currentMillis;        //Set previousMillis to millis to reset timeout
      pressCount[1] = 0;                        //Set the number of times the button has been detected as pressed to 0
    }else {
      if (currentMillis - previousMillis[1] > bounceDelay) {
        previousMillis[1] = currentMillis;        //Set previousMillis to millis to reset timeout
        ++pressCount[1];
        if (pressCount[1] == minButtonPress) {
          sp = sp - 1;                             //Button has been debounced. Call function to do whatever you want done.
        }
      }
    }
    lcd.clear();
    lcd.print("Sp : ");
    lcd.print(sp);
  }else if(count == 1){
    if(digitalRead(buttonPins[0])) {             //Input is high, button not pressed or in the middle of bouncing and happens to be high
      previousMillis[0] = currentMillis;        //Set previousMillis to millis to reset timeout
      pressCount[0] = 0;                        //Set the number of times the button has been detected as pressed to 0
    }else {
      if (currentMillis - previousMillis[0] > bounceDelay) {
        previousMillis[0] = currentMillis;        //Set previousMillis to millis to reset timeout
        ++pressCount[0];
        if (pressCount[0] == minButtonPress) {
          Upper = Upper + 5;                             //Button has been debounced. Call function to do whatever you want done.
        }
      }
    }
    if(digitalRead(buttonPins[1])) {             //Input is high, button not pressed or in the middle of bouncing and happens to be high
      previousMillis[1] = currentMillis;        //Set previousMillis to millis to reset timeout
      pressCount[1] = 0;                        //Set the number of times the button has been detected as pressed to 0
    }else {
      if (currentMillis - previousMillis[1] > bounceDelay) {
        previousMillis[1] = currentMillis;        //Set previousMillis to millis to reset timeout
        ++pressCount[1];
        if (pressCount[1] == minButtonPress) {
          Upper = Upper - 5;                             //Button has been debounced. Call function to do whatever you want done.
        }
      }
    }
    if(Upper > 250){
      Upper = 255;
    }
    lcd.clear();
    lcd.print("pwmUpper : ");
    lcd.print(Upper);
  }else if(count == 2){
    if(digitalRead(buttonPins[0])) {             //Input is high, button not pressed or in the middle of bouncing and happens to be high
      previousMillis[0] = currentMillis;        //Set previousMillis to millis to reset timeout
      pressCount[0] = 0;                        //Set the number of times the button has been detected as pressed to 0
    }else {
      if (currentMillis - previousMillis[0] > bounceDelay) {
        previousMillis[0] = currentMillis;        //Set previousMillis to millis to reset timeout
        ++pressCount[0];
        if (pressCount[0] == minButtonPress) {
          Lower = Lower + 5;                             //Button has been debounced. Call function to do whatever you want done.
        }
      }
    }
    if(digitalRead(buttonPins[1])) {             //Input is high, button not pressed or in the middle of bouncing and happens to be high
      previousMillis[1] = currentMillis;        //Set previousMillis to millis to reset timeout
      pressCount[1] = 0;                        //Set the number of times the button has been detected as pressed to 0
    }else {
      if (currentMillis - previousMillis[1] > bounceDelay) {
        previousMillis[1] = currentMillis;        //Set previousMillis to millis to reset timeout
        ++pressCount[1];
        if (pressCount[1] == minButtonPress) {
          Lower = Lower - 5;                             //Button has been debounced. Call function to do whatever you want done.
        }
      }
    }
    if(Lower < 5){
      Lower = 5;
    }
    lcd.clear();
    lcd.print("pwmLower : ");
    lcd.print(Lower);
  }
  
 if(count > 2){
    count = 0;
 }
  
  
  
  
 Serial.println("Count : "  + String(count) + "Sp : " + String(sp) + "pwmUpper : " + String(pwmUpper));
 
}
// if(count == 0){
//     if(digitalRead(buttonPins[0])) {             //Input is high, button not pressed or in the middle of bouncing and happens to be high
//       previousMillis[0] = currentMillis;        //Set previousMillis to millis to reset timeout
//       pressCount[0] = 0;                        //Set the number of times the button has been detected as pressed to 0
//     }else {
//       if (currentMillis - previousMillis[0] > bounceDelay) {
//         previousMillis[0] = currentMillis;        //Set previousMillis to millis to reset timeout
//         ++pressCount[0];
//         if (pressCount[0] == minButtonPress) {
//           sp = sp + 1;                             //Button has been debounced. Call function to do whatever you want done.
//         }
//       }
//     }
//     if(digitalRead(buttonPins[1])) {             //Input is high, button not pressed or in the middle of bouncing and happens to be high
//       previousMillis[1] = currentMillis;        //Set previousMillis to millis to reset timeout
//       pressCount[1] = 0;                        //Set the number of times the button has been detected as pressed to 0
//     }else {
//       if (currentMillis - previousMillis[1] > bounceDelay) {
//         previousMillis[1] = currentMillis;        //Set previousMillis to millis to reset timeout
//         ++pressCount[1];
//         if (pressCount[1] == minButtonPress) {
//           sp = sp - 1;                             //Button has been debounced. Call function to do whatever you want done.
//         }
//       }
//     }
//     lcd.clear();
//     lcd.print("Sp : ");
//     lcd.print(sp);
//   }else if(count == 1){
//     if(digitalRead(buttonPins[0])) {             //Input is high, button not pressed or in the middle of bouncing and happens to be high
//       previousMillis[0] = currentMillis;        //Set previousMillis to millis to reset timeout
//       pressCount[0] = 0;                        //Set the number of times the button has been detected as pressed to 0
//     }else {
//       if (currentMillis - previousMillis[0] > bounceDelay) {
//         previousMillis[0] = currentMillis;        //Set previousMillis to millis to reset timeout
//         ++pressCount[0];
//         if (pressCount[0] == minButtonPress) {
//           Upper = Upper + 5;                             //Button has been debounced. Call function to do whatever you want done.
//         }
//       }
//     }
//     if(digitalRead(buttonPins[1])) {             //Input is high, button not pressed or in the middle of bouncing and happens to be high
//       previousMillis[1] = currentMillis;        //Set previousMillis to millis to reset timeout
//       pressCount[1] = 0;                        //Set the number of times the button has been detected as pressed to 0
//     }else {
//       if (currentMillis - previousMillis[1] > bounceDelay) {
//         previousMillis[1] = currentMillis;        //Set previousMillis to millis to reset timeout
//         ++pressCount[1];
//         if (pressCount[1] == minButtonPress) {
//           Upper = Upper - 5;                             //Button has been debounced. Call function to do whatever you want done.
//         }
//       }
//     }
//     if(Upper > 250){
//       Upper = 255;
//     }
//     lcd.clear();
//     lcd.print("pwmUpper : ");
//     lcd.print(Upper);
//   }else if(count == 2){
//     if(digitalRead(buttonPins[0])) {             //Input is high, button not pressed or in the middle of bouncing and happens to be high
//       previousMillis[0] = currentMillis;        //Set previousMillis to millis to reset timeout
//       pressCount[0] = 0;                        //Set the number of times the button has been detected as pressed to 0
//     }else {
//       if (currentMillis - previousMillis[0] > bounceDelay) {
//         previousMillis[0] = currentMillis;        //Set previousMillis to millis to reset timeout
//         ++pressCount[0];
//         if (pressCount[0] == minButtonPress) {
//           Lower = Lower + 5;                             //Button has been debounced. Call function to do whatever you want done.
//         }
//       }
//     }
//     if(digitalRead(buttonPins[1])) {             //Input is high, button not pressed or in the middle of bouncing and happens to be high
//       previousMillis[1] = currentMillis;        //Set previousMillis to millis to reset timeout
//       pressCount[1] = 0;                        //Set the number of times the button has been detected as pressed to 0
//     }else {
//       if (currentMillis - previousMillis[1] > bounceDelay) {
//         previousMillis[1] = currentMillis;        //Set previousMillis to millis to reset timeout
//         ++pressCount[1];
//         if (pressCount[1] == minButtonPress) {
//           Lower = Lower - 5;                             //Button has been debounced. Call function to do whatever you want done.
//         }
//       }
//     }
//     if(Lower < 5){
//       Lower = 5;
//     }
//     lcd.clear();
//     lcd.print("pwmLower : ");
//     lcd.print(Lower);
//   }
  
//  if(count > 2){
//     count = 0;
//  }
  
