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

void setup() {
  Serial.begin(0);
  digitalWrite(STEP,LOW);
  pinMode(STEP,OUTPUT);
  digitalWrite(DIR,LOW);
  pinMode(DIR,OUTPUT);
   // 1/16 step
  pinMode(MS1, OUTPUT);
  digitalWrite(MS1, HIGH);
  pinMode(MS2, OUTPUT);
  digitalWrite(MS2, HIGH);
  pinMode(MS3, OUTPUT);
  digitalWrite(MS3, HIGH);
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
      }
   }
