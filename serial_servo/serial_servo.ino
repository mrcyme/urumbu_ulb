//
// serialservo.ino
//
// serial servo PWM
//
// Neil Gershenfeld 7/28/21
// Quentin Bolsee 12/7/21 : add button
//
// This work may be reproduced, modified, distributed,
// performed, and displayed for any purpose, but must
// acknowledge this project. Copyright is retained and
// must be preserved. The work is provided as is; no
// warranty is provided, and users accept all liability.
//


#define SERVO 0


void setup() {
   Serial.begin(0);
   digitalWrite(SERVO,LOW);
   pinMode(SERVO,OUTPUT);
   }

void loop() {
   uint16_t duration;
   if (Serial.available()) {
      digitalWrite(SERVO,HIGH);
      delayMicroseconds(duration);
      digitalWrite(SERVO,LOW);
         }
    }
   
