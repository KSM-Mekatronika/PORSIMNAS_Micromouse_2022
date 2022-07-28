#include <NewPing.h>

#define TRIGGER_PIN1    3      // Arduino pin tied to trigger pin on the ultrasonic sensor.
#define ECHO_PIN1       2      // Arduino pin tied to echo pin on the ultrasonic sensor.
#define TRIGGER_PIN2    5      // Arduino pin tied to trigger pin on the ultrasonic sensor.
#define ECHO_PIN2       6       // Arduino pin tied to echo pin on the ultrasonic sensor.
#define MAX_DISTANCE    200    // Maximum distance we want to ping for (in centimeters). Maximum sensor distance is rated at 400-500cm.
int dir1  = 12;                // inisiai pin direct (arah putaran) untuk motor kiri
int pwm1  = 10;                // inisiai pin pwm (kecepatan putar) untuk motor kiri
int pwm2  = 11;                // inisiai pin direct (arah putaran) untuk motor kanan
int dir2  = 13;                // inisiai pin pwm (kecepatan putar) untuk motor kanan

int pwmA;
int pwmB;
int ka;
int ki;

int Upper = 255;               // Pwm maksimal yang diizinkan
int Lower = 15;                // Pwm minimal yang diizinkan

NewPing sonar1(TRIGGER_PIN1, ECHO_PIN1, MAX_DISTANCE);  // NewPing setup of pins and maximum distance.
NewPing sonar2(TRIGGER_PIN2, ECHO_PIN2, MAX_DISTANCE);  // NewPing setup of pins and maximum distance.

void setup() 
{
  Serial.begin(115200);         // Open serial monitor at 115200 baud to see ping results.
  pinMode(dir1, OUTPUT);      // Set pin untuk motor sebagai OUTPUT
  pinMode(pwm1, OUTPUT);
  pinMode(dir2, OUTPUT);
  pinMode(pwm2, OUTPUT);
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

  if (depan<=4{
    digitalWrite(dir1, HIGH);            
    digitalWrite(dir2, LOW);
    analogWrite(pwm1, 255 - 100);
    analogWrite(pwm2, 100);
  }else{
    int sp = 5;                          
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
  delay(100);
}
