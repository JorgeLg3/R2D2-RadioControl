#include <SoftwareSerial.h>
#include <Servo.h>
#include <wiring_private.h>
#include <pins_arduino.h>
#define speakerPin 8

//notas para cancion

#define  c3    7634
#define  d3    6803
#define  e3    6061
#define  f3    5714
#define  g3    5102
#define  a3    4545
#define  b3    4049
#define  c4    3816    // 261 Hz
#define  d4    3401    // 294 Hz
#define  e4    3030    // 329 Hz
#define  f4    2865    // 349 Hz
#define  g4    2551    // 392 Hz
#define  a4    2272    // 440 Hz
#define  a4s   2146
#define  b4    2028    // 493 Hz
#define  c5    1912    // 523 Hz
#define  d5    1706
#define  d5s   1608
#define  e5    1517    // 659 Hz
#define  f5    1433    // 698 Hz
#define  g5    1276
#define  a5    1136
#define  a5s   1073
#define  b5    1012
#define  c6    955

         #define  R     0  

/***   Global variables   ***/
SoftwareSerial _bt_device(2,4);
char data='e';
Servo _servo10;
Servo _servo5;
Servo _servo3;
int mode = 0;
int state_auto = 0;

// Melody 2: Star Wars Theme
int melody2[] = {  f4,  f4, f4,  a4s,   f5,  d5s,  d5,  c5, a5s, f5, d5s,  d5,  c5, a5s, f5, d5s, d5, d5s,   c5};
int beats2[]  = {  21,  21, 21,  128,  128,   21,  21,  21, 128, 64,  21,  21,  21, 128, 64,  21, 21,  21, 128 };

int MAX_COUNT = sizeof(melody2) / 2;
long tempo = 10000;
int pause = 1000;
int rest_count = 50;
int toneM = 0;
int beat = 0;
long duration  = 0;
int potVal = 0;

const uint16_t IMPERIAL_MARCH[] = {440,375,440,375,440,375,349,281,523,93,440,375,349,281,523,93,440,750,659,375,659,375,659,375,698,281,523,93,415,375,349,281,523,93,440,750,880,375,440,281,440,93,880,375,831,281,784,93,740,93,698,93,740,187,0,187,446,281,622,375,587,281,554,93,523,93,494,93,523,187,0,187,349,187,415,375,349,281,392,93,523,375,440,281,523,93,659,750,880,375,440,281,440,93,880,375,831,281,784,93,740,93,698,93,740,187,0,187,446,281,622,375,587,281,554,93,523,93,494,93,523,187,0,187,330,187,415,375,330,281,523,93,440,375,349,281,523,93,440,750};
volatile uint16_t* _current_melody_ptr;
volatile int _melody_pin=0;
volatile int _melody_counter=0;
volatile int _melody_length=0;
volatile uint8_t _melody_status=-1;
volatile unsigned long _melody_next_time=0;
unsigned long _currentTime;

/***   Function declaration   ***/
void prepareMelody(int pin,const uint16_t* melody, int length);
void playMelodyInterrupt();
//ultrasonidos
unsigned long _pulseIn(uint8_t pin, uint8_t state, unsigned long timeout);
long US_init(int trigger_pin, int echo_pin);
long distance(int trigger_pin, int echo_pin);
void playTone();

/***   Class declaration   ***/

/***   Tasks declaration   ***/
SIGNAL (TIMER0_COMPA_vect){
playMelodyInterrupt();
}
/***   ISR function declaration   ***/

/***   Additional Global variables   ***/

void setup()
{


  _bt_device.begin(9600);
  _bt_device.flush();
  
  OCR0A=0xAF;
  TIMSK0 |= _BV(OCIE0A);




  Serial.begin(9600);

  _servo10.attach(10);
  _servo5.attach(5);
  _servo3.attach(3);

  pinMode( (7) , INPUT );
  pinMode( (13) , OUTPUT );
  pinMode(speakerPin, OUTPUT);

  pinMode((11),OUTPUT);
  pinMode((9),OUTPUT);
  pinMode((6),OUTPUT);

  digitalWrite((11),HIGH);
  digitalWrite((9),LOW);
  digitalWrite((6),LOW);
  _servo10.write(((0*90)/100+90));
  _servo5.write(((0*90)/100+90));
  randomSeed(analogRead(0));
}


void loop()
{
    
    long dist = 0;
    data ='a';
    //dist = distance((13),(7));
    //Serial.println(dist);
    if (_bt_device.available()>0){
      digitalWrite((11),LOW);
      digitalWrite((9),HIGH);
      digitalWrite((6),LOW);
      data=_bt_device.read();
      Serial.print(data);
      Serial.println();
      //selector de modo
      if (data=='f' && mode==0){
        mode=1;
      }
      if (data=='g' && mode==0){
        mode=2;
      }
      if (data=='h' && mode==0){
        mode=3;
      }
      if (data=='e' && (mode==3 || mode==1  || mode==2)){
        mode=0;
      }
    }
      switch (mode)
      {
        case 0:
        {
          //modo de reposo
          digitalWrite((11),HIGH);
          digitalWrite((9),LOW);
          digitalWrite((6),LOW);
          _servo10.write((((0)*90)/100+90));
          _servo5.write((((0)*90)/100+90));
          break;
        }
        case 1:
        {
          //modo de control remoto
          switch (data)
          {
            case 'u':
            {
              //adelante
              _servo10.write((((20)*90)/100+90));
              _servo5.write((((-26)*90)/100+90));
              break;
            }
            case 'd':
            {
              //atras
              _servo10.write((((-20)*90)/100+90));
              _servo5.write((((20)*90)/100+90));
              break;
            }
            case 'r':
            {
              //derecha
              _servo10.write((((15)*90)/100+90));
              _servo5.write((((15)*90)/100+90));
              break;
            }
            case 'l':
            {
              //izq
              _servo10.write((((-15)*90)/100+90));
              _servo5.write((((-15)*90)/100+90));
              break;
            }
            case 's':
            {
              //parar
              _servo10.write((((0)*90)/100+90));
              _servo5.write((((0)*90)/100+90));
              break;
            }
            case 'x':
            {
              _servo3.write(90);
              break;
            }
            case 'y':
            {
              _servo3.write(0);
              break;
            }
            case 'm':
            {
              prepareMelody(8,IMPERIAL_MARCH,sizeof(IMPERIAL_MARCH)/(2*sizeof(uint16_t)));


              

              break;
            }
            case 'n':
            {
              // Melody2
              MAX_COUNT = sizeof(melody2) / 2;
              for (int i = 0; i < MAX_COUNT; i++) {
              toneM = melody2[i];
              beat = beats2[i];
              duration = beat * tempo;
              playTone();
              delayMicroseconds(pause);
              }
              break;
            }
            case 'o':
            {
                            int K = 2000;
              switch (random(1,7)) {
                  
                  case 1:phrase1(); break;
                  case 2:phrase2(); break;
                  case 3:phrase1(); phrase2(); break;
                  case 4:phrase1(); phrase2(); phrase1();break;
                  case 5:phrase1(); phrase2(); phrase1(); phrase2(); phrase1();break;
                  case 6:phrase2(); phrase1(); phrase2(); break;
              }
              for (int i = 0; i <= random(3, 9); i++){
                  
                  digitalWrite(6, HIGH);  
                  tone(speakerPin, K + random(-1700, 2000));          
                  delay(random(70, 170));  
                  digitalWrite(6, LOW);           
                  noTone(speakerPin);         
                  delay(random(0, 30));             
              } 
              noTone(speakerPin);         
              delay(random(1000, 2000));
              break;
            }
            case 'b':
            {
              digitalWrite((11),LOW);
              digitalWrite((9),LOW);
              digitalWrite((6),HIGH);
              break;
            }
          }
          break;
        }
        case 2:
        {
          //modo autónomo
          digitalWrite((11),HIGH);
          digitalWrite((9),LOW);
          digitalWrite((6),LOW);

          dist = distance((13),(7));

          if ((dist)<=(15) && state_auto==0){
            
            state_auto=2;
            //Serial.println(dist);
      
          }
          if (dist>15 && state_auto==0)
          {
            state_auto=1;
          }
          if (dist<=15 && state_auto==1)
          {
            state_auto=0;
          }
          if (dist>15 && state_auto==2)
          {
            state_auto=0;
          }

          Serial.println(dist);
          Serial.println(state_auto);
          switch(state_auto)
          {
            case 0:
            {
              _servo10.write((((0)*90)/100+90));
              _servo5.write((((0)*90)/100+90));
              delay(500);
              break;
            }
            case 1:
            {
              _servo10.write((((20)*90)/100+90));
              _servo5.write((((-26)*90)/100+90));
              break;
            }
            case 2:
            {
              _servo10.write((((15)*90)/100+90));
              _servo5.write((((15)*90)/100+90));
              delay(1000);
              break;
            }
          }

          


          break;
        }
        case 3:
        {
          //modo seguimiento
          digitalWrite((11),HIGH);
          digitalWrite((9),LOW);
          digitalWrite((6),LOW);
          _servo5.write((((((-20)*(distance((13),(7)))/40))*90)/100+90));
          _servo10.write((((((15)*(distance((13),(7)))/40))*90)/100+90));
          break;
        }
      }
//      switch (data)
//      {
//        case 'u':
//        {
//          _servo10.write((((20)*90)/100+90));
//          _servo5.write((((-26)*90)/100+90));
//          break;
//        }
//        case 'd':
//        {
//          _servo10.write((((-20)*90)/100+90));
//          _servo5.write((((20)*90)/100+90));
//          break;
//        }
//        case 'r':
//        {
//          _servo10.write(((20*90)/100+90));
//          _servo5.write(((20*90)/100+90));
//          break;
//        }
//        case 'l':
//        {
//          _servo10.write((((-20)*90)/100+90));
//          _servo5.write((((-20)*90)/100+90));
//          break;
//        }
//        case 's':
//        {
//          _servo10.write(((0*90)/100+90));
//          _servo5.write(((0*90)/100+90));
//          break;
//        }
//        case 'x':
//        {
//          _servo3.write(90);
//          break;
//        }
//        case 'y':
//        {
//          _servo3.write(0);
//          break;
//        }
//        case 'm':
//        {
//          prepareMelody(8,IMPERIAL_MARCH,sizeof(IMPERIAL_MARCH)/(2*sizeof(uint16_t)));
//          break;
//        }
//        case 'b':
//        {
//          digitalWrite((11),LOW);
//          digitalWrite((9),LOW);
//          digitalWrite((6),HIGH);
//          break;
//        }
//      }

//    }

}

/***   Function definition   ***/
void prepareMelody(int pin,const uint16_t* melody, int length)
{
  if (_melody_status==1)
  return;
  _melody_pin=pin;
  _current_melody_ptr=(uint16_t*)melody;
  _melody_length=length;
  _melody_counter=0;
  noTone(_melody_pin);
  _melody_next_time=millis()+2;
  _melody_status=1;
}
void playMelodyInterrupt()
{
  unsigned int _melody_note=0;
  unsigned long _melody_duration=0;
  unsigned long currentTime=millis();
   if (_melody_status==0)
 {
  noTone(_melody_pin);
  _melody_status=-1;
 }
 else if (_melody_status==1){
  if (_melody_counter>_melody_length)
  {
  _melody_status=0;
  }
  if (currentTime>=_melody_next_time)
  {
  _melody_note=*_current_melody_ptr++;
  _melody_duration=*_current_melody_ptr++;
  tone(_melody_pin,_melody_note,_melody_duration);
  _melody_next_time=currentTime+_melody_duration;
  _melody_counter++;
  }
 }
}

unsigned long _pulseIn(uint8_t pin, uint8_t state, unsigned long timeout)
{
  uint8_t bit = digitalPinToBitMask(pin);
  uint8_t port = digitalPinToPort(pin);
  uint8_t stateMask = (state ? bit : 0);
  unsigned long width = 0;
  unsigned long numloops = 0;
  unsigned long maxloops = microsecondsToClockCycles(timeout) / 16;
  while ((*portInputRegister(port) & bit) == stateMask)
  if (numloops++ == maxloops)
    return 0;
  while ((*portInputRegister(port) & bit) != stateMask)
  if (numloops++ == maxloops)
    return 0;
  while ((*portInputRegister(port) & bit) == stateMask) {
  if (numloops++ == maxloops)
    return 0;
  width++;
  }
  return clockCyclesToMicroseconds(width * 21 + 16);
}
long US_init(int trigger_pin, int echo_pin)
{
  digitalWrite(trigger_pin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigger_pin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigger_pin, LOW);
  long microseconds = _pulseIn(echo_pin ,HIGH,100000);
  return microseconds;
}
long distance(int trigger_pin, int echo_pin)
{
  long microseconds = US_init(trigger_pin, echo_pin);
  long distance;
  distance = microseconds/29/2;
  if (distance == 0){
  distance = 999;
  }
  return distance;
}

void phrase1() {
    
    int k = random(1000,2000);
    //digitalWrite(ledPin, HIGH);
    for (int i = 0; i <=  random(100,2000); i++){
        
        tone(speakerPin, k+(-i*2));          
        delay(random(.9,2));             
    } 
    //digitalWrite(ledPin, LOW);   
    for (int i = 0; i <= random(100,1000); i++){
        
        tone(speakerPin, k + (i * 10));          
        delay(random(.9,2));             
    } 
}

void phrase2() {
    
    int k = random(1000,2000);
    //digitalWrite(ledPin, HIGH);  
    for (int i = 0; i <= random(100,2000); i++){
        
        tone(speakerPin, k+(i*2));          
        delay(random(.9,2));             
    } 
    //digitalWrite(ledPin, LOW);   
    for (int i = 0; i <= random(100,1000); i++){
        
        tone(speakerPin, k + (-i * 10));          
        delay(random(.9,2));             
    } 
}

void playTone() {

long elapsed_time = 0;
if (toneM > 0) {

          //digitalWrite(led,HIGH);

while (elapsed_time < duration) {
digitalWrite(speakerPin,HIGH);
delayMicroseconds(toneM / 2);
digitalWrite(speakerPin, LOW);
delayMicroseconds(toneM / 2);
elapsed_time += (toneM);

       }

//digitalWrite(led,LOW);

}
else {
for (int j = 0; j < rest_count; j++) {
delayMicroseconds(duration);
}

}

}
