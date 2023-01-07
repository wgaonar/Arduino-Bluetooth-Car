#include <SoftwareSerial.h>

const int TxPin = 6;  // Go to RX in HC-05 bluetooth module
const int RxPin = 7;  // Go to TX in HC-05 bluetooth module

const int servoPin = 2;

const int echoPin = 14; // Use A0 as digital pin
const int trigPin = 15; // Use A1 as digital pin


const int in1Pin = 8;
const int in2Pin = 9;
const int enAPin = 10;
const int enBPin = 11;
const int in3Pin = 12;
const int in4Pin = 13;

int pwm;
int angle = 0;

float distance;

int movCant;
int movTipo;
int sensorValue;



SoftwareSerial SerialOne(RxPin,TxPin);

void setup()
{
  // initialize the serial communication:
  Serial.begin(9600); //baud rate
  
  // initialize my port one
  SerialOne.begin(9600);
  Serial.println("My serial por is open..."); 

  pinMode(servoPin, OUTPUT);
  
  /// Sensor HC-SR04 Pins
  pinMode(echoPin, INPUT);
  pinMode(trigPin, OUTPUT);
  
  /// Motors Pins
  pinMode(in1Pin, OUTPUT);   // sets the pin as output
  pinMode(in2Pin, OUTPUT);   // sets the pin as output
  pinMode(enAPin, OUTPUT);   // sets the pin as output
  pinMode(enBPin, OUTPUT);   // sets the pin as output
  pinMode(in3Pin, OUTPUT);   // sets the pin as output
  pinMode(in4Pin, OUTPUT);   // sets the pin as output
  
  ///motors Fast stop
  moveNone(); 
}

void loop() 
{
  
  // By default, the last intialized port is listening.
  // when you want to listen on a port, explicitly select it:
  SerialOne.listen();
   
  if(SerialOne.available() > 0) 
  {
    // look for the nexts valid integers
    movTipo = SerialOne.parseInt();
    movCant = SerialOne.parseInt();
    
    Serial.print("movTipo:\t");
    Serial.println(movTipo);
    Serial.print("movCant:\t");
    Serial.println(movCant);
    
    switch (movTipo) 
    {
      case 1: {
        Serial.println("adelante");
        moveForward(movCant);
      } 
      break;
      case 2: {
        Serial.println("atras");
        moveBackward(movCant);
      }
      break;
      case 3: {
        Serial.println("derecha");
        moveRight(movCant);
      }
      break;
      case 4:{
        Serial.println("izquierda");
        moveLeft(movCant);
      }
      break;
      case 5: {
        Serial.println("detenido");
        moveNone();  
      }
      break;
      case 6: {
        Serial.println("parada");
        while (movTipo != 7) {
          moveNone();
          movTipo = SerialOne.parseInt();
          movCant = SerialOne.parseInt();
        }  
      }
      break;   
      default:
        break;
    }
    
  }
  
  else
  {
    moveForward(128);
    delay(500);

    moveBackward(128);
    delay(500);
  }   

  /*
  int val = analogRead(analogPin);
  delay(10);
  sensorValue = map(val, 0, 1023, 0, 255);
  Serial.print("\nSensorValue: ");
  Serial.println(val);
  */
  /*
   /// Spaning the horizon with ultrasonic sonar
    for (angle = 90; angle <= 90; angle += 90)  {
        Serial.print("Angle: "); 
        Serial.print(angle);
        servoPulse(servoPin, angle);
        delay(10);
        distance = getDistanceCentimeters();
        Serial.print("\tDistance: ");
        Serial.print(distance);
        Serial.println(" cm.");
        SerialOne.println(int(distance));  
        delay(10); //make it readable    
    } 
  */
}

float getDistanceCentimeters () {
  long duration;
  float cm;
  const float airSpeed = 0.034029;
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin,HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin,LOW);
  duration = pulseIn(echoPin,HIGH);
  cm = duration*airSpeed/2;
  return cm;
}  

float getDistanceMilimeters () {
  long duration;
  float mm;
  const float airSpeed = 0.340290;
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin,HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin,LOW);
  duration = pulseIn(echoPin,HIGH);
  mm = duration*airSpeed/2;
  return mm;
}  

void servoPulse (int servoPin, int angle)
{
  pwm = (angle*11) + 440;      // Convert angle to microseconds
  for (int i = 0; i < 100; i++) {
    digitalWrite(servoPin, HIGH);
    delayMicroseconds(pwm);
    digitalWrite(servoPin, LOW);
    delayMicroseconds(2420-pwm);    // Refresh cycle of servo
  }
}

void moveForward (int movCant) {
  ///motor 1 configuration
  digitalWrite(in1Pin,HIGH);
  digitalWrite(in2Pin,LOW);
  ///motor 2 configuration
  digitalWrite(in3Pin,HIGH);
  digitalWrite(in4Pin,LOW);
  ///enable motors 1 y 2 by PWM
  analogWrite(enAPin, movCant);  
  analogWrite(enBPin, movCant);
  delay(50);
} 

void moveBackward (int movCant) {
  ///motor 1 configuration
  digitalWrite(in1Pin,LOW);
  digitalWrite(in2Pin,HIGH);
  ///motor 2 configuration
  digitalWrite(in3Pin,LOW);
  digitalWrite(in4Pin,HIGH);
  ///enable motors 1 y 2 by PWM
  analogWrite(enAPin, movCant);  
  analogWrite(enBPin, movCant);
  delay(50);
} 

void moveLeft (int movCant) {
  ///motor 1 configuration
  digitalWrite(in1Pin,LOW);
  digitalWrite(in2Pin,HIGH);
  ///motor 2 configuration
  digitalWrite(in3Pin,HIGH);
  digitalWrite(in4Pin,LOW);
  ///enable motors 1 y 2 by PWM
  analogWrite(enAPin, movCant);  
  analogWrite(enBPin, movCant);
  delay(50);
}  

void moveRight (int movCant) {
  ///motor 1 configuration
  digitalWrite(in1Pin,HIGH);
  digitalWrite(in2Pin,LOW);
  ///motor 2 configuration
  digitalWrite(in3Pin,LOW);
  digitalWrite(in4Pin,HIGH);
  ///enable motors 1 y 2 by PWM
  analogWrite(enAPin, movCant);  
  analogWrite(enBPin, movCant);
  delay(50);
}  

void moveNone () {
  ///motor 1 fast stop
  digitalWrite(in1Pin,LOW);
  digitalWrite(in2Pin,LOW);
  ///motor 2 fast stop
  digitalWrite(in3Pin,LOW);
  digitalWrite(in4Pin,LOW);
  //disable motors 1 y 2
  digitalWrite(enAPin, HIGH);  
  digitalWrite(enBPin, HIGH);
  delay(50);
} 




