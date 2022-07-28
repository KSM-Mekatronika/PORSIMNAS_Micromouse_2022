//-------EDITING--------
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
  //Kodingan di bawah ini untuk mengambil nilai yang terbaca oleh sensor ultrasonic (dalam satuan CM)
  delay(50);
  unsigned int us = sonar1.ping();     // Send ping, get ping time in microseconds (uS).
  int kanan = us / US_ROUNDTRIP_CM;
  unsigned int us2 = sonar2.ping();    // Send ping, get ping time in microseconds (uS).
  int depan = us2 / US_ROUNDTRIP_CM;


  if (kanan>20) {kanan=20;}            // Pembacaan maksimal sensor kanan adalah 20, di atas 20 tetap terbaca 20.
  else {kanan=kanan;}
  //---akhir kodingan membaca nilai sensor---

  // Kodingan di bawah ini adalah untuk melakukan aksi ketika kondisi Robot menabrak dinding di depan
  if (depan<=4)                        // ketika sensor depan mendeteksi dinding kurang dari 3 cm robot harus belok kiri
  {
  digitalWrite(dir1, HIGH);            // kode untuk membuat robot berbelok ke kiri
  digitalWrite(dir2, LOW);
  analogWrite(pwm1, 255 - 100);
  analogWrite(pwm2, 100);
  }
  //---akhir kodingan kondisi menabrak dinding depan---

  // Kodingan di bawah ini adalah kondisi kebalikan dari kondisi di atas yaitu ketika sensor depan tidak menabrak dinding
  // Kodingan menggunakkan pendekatan metode PID (Kontrol Proporsional).
  else
  {
  int sp = 5;                          // sp = set point, posisi robot harus senantiasa di jarak 7 cm
  int error = sp - kanan;              // mendefinisikan nilai error, selisih antara setpoint dan pembacaan sensor kanan
    
  digitalWrite(dir1, HIGH);            // OBSTABOT bergerak Maju
  digitalWrite(dir2, HIGH);
  
  int pid = 9*error;                   // Pembobotan nilai PID, Angka 9 adalah konstanta proporsional (Kp)
    
  pwmA = sp*10-pid;                    // Pembobotan OUTPUT/pwm hasil perhitungan PID, saya masih belum yakin ini sesuai dengan teori PID
  if(pwmA > Upper){pwmA = Upper;}      // Angka 10 adalah konstanta yang membuat pwm berada dikisaran angka ratusan/motor bisa berputar
  if(pwmA < Lower){pwmA = Lower;}
  ki = pwmA;                           //pwm kiri
  
  pwmB = sp*10+pid;
  if(pwmB > Upper){pwmB = Upper;}
  if(pwmB < Lower){pwmB = Lower;}
  ka = pwmB;                           //pwm kanan
      
  analogWrite(pwm1, ka);
  analogWrite(pwm2, ki);
  }
  Serial.println("Depan : " + String(depan) + "  | Kanan : " + String(kanan) + "  | ki : " + String(ki) + "  | ka : " + String(ka));
  delay(100);
}
