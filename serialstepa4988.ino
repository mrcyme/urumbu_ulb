//
// serialstep.ino
//
// serial step-and-direction
//
// Neil Gershenfeld 4/11/21
// Quentin Bolsee 12/7/21 : add button
//
// This work may be reproduced, modified, distributed,
// performed, and displayed for any purpose, but must
// acknowledge this project. Copyright is retained and
// must be preserved. The work is provided as is; no
// warranty is provided, and users accept all liability.
//


#define DIR 26
#define STEP 27
#define MS1 0
#define MS2 7
#define MS3 6
#define NSTEPS 1000
#define DELAYHIGH 10
#define DELAYLOW 1000
#define BLINK 100
#define BUTTON 31

void setup() {
  Serial.begin(0);
  digitalWrite(STEP,LOW);
  pinMode(STEP,OUTPUT);
  digitalWrite(DIR,LOW);
  pinMode(DIR,OUTPUT);
  pinMode(BUTTON, INPUT_PULLUP);
   /*
   // 1 step
   digitalWrite(M0,LOW);
   digitalWrite(M1,LOW);
   pinMode(M0,OUTPUT);
   pinMode(M1,OUTPUT);
   // 1/2 step
   digitalWrite(M0,HIGH);
   digitalWrite(M1,LOW);
   pinMode(M0,OUTPUT);
   pinMode(M1,OUTPUT);
   */
   // 1/8 step
  pinMode(MS1, OUTPUT);
  digitalWrite(MS1, HIGH);
  pinMode(MS2, OUTPUT);
  digitalWrite(MS2, HIGH);
  pinMode(MS3, OUTPUT);
  digitalWrite(MS3, HIGH);

   /*
   // 1/16 step
   digitalWrite(M1,HIGH);
   pinMode(M0,INPUT);
   pinMode(M1,OUTPUT);
   // 1/32 step
   digitalWrite(M0,LOW);
   pinMode(M0,OUTPUT);
   pinMode(M1,INPUT);
   */
   }

void loop() {
   if (Serial.available()) {
      char c = Serial.read();
      if (c == 'f') {
         digitalWrite(DIR,HIGH);
         digitalWrite(STEP,HIGH);
         delayMicroseconds(2);
         digitalWrite(STEP,LOW);
         }
      else if (c == 'r') {
         digitalWrite(DIR,LOW);
         digitalWrite(STEP,HIGH);
         delayMicroseconds(2);
         digitalWrite(STEP,LOW);
         }
      else if (c == '?') {
         // reply with button value
         int btn = digitalRead(BUTTON);
         Serial.write(btn ? '1' : '0');
         }
      else if (c == '@') {
         Serial.write("0000");
         }
      }
   }
