#include <SoftwareSerial.h>
#include <AFMotor.h>

AF_DCMotor motorFL(1);
AF_DCMotor motorRL(3);
AF_DCMotor motorFR(2);
AF_DCMotor motorRR(4);

const int TxPin = 18;
const int RxPin = 19;

const int servoPin = 2;

const int echoPin = 3;
const int trigPin = 5;
const int analogPin = 0;

int pwm;
int angle = 0;

float distance;

int movCant;
int movTipo;
int state = 0;
int sensorValue;



SoftwareSerial SerialOne(RxPin,TxPin);

void setup()
{
  // initialize the serial communication:
  Serial.begin(9600); //baud rate
  
  // initialize my port one
  SerialOne.begin(9600);
  Serial.print("My serial port: one, is open...\n"); 

  pinMode(servoPin, OUTPUT);
  
  // Sensor HC-SR04 Pins
  pinMode(echoPin, INPUT);
  pinMode(trigPin, OUTPUT);
  

  //motors Fast stop
  moveNone(); 
}

void loop() {
  
  // By default, the last intialized port is listening.
  // when you want to listen on a port, explicitly select it:
  SerialOne.listen();
   
  if(SerialOne.available() > 0) {
    // look for the nexts valid integers
    movTipo = SerialOne.parseInt();
    movCant = SerialOne.parseInt();
    state = SerialOne.parseInt();
    
    Serial.print("movTipo: ");
    Serial.print(movTipo);
    Serial.print("\tmovCant: ");
    Serial.print(movCant);
    Serial.print("\tstate: ");
    Serial.print(state);
    Serial.print("\n");
    
    if(state==0) {
      moveNone();
    }
      
    if (state==1) {
      switch (movTipo) {
        case 1: {
          Serial.println("Moviendo hacia adelante");
          moveForward(movCant);
        } 
        break;
        case 2: {
          Serial.println("Moviendo hacia atras");
          moveBackward(movCant);
        }
        break;
        case 3: {
          Serial.println("Moviendo hacia derecha");
          moveRight(movCant);
        }
        break;
        case 4:{
          Serial.println("Moviendo hacia izquierda");
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
  // setup direction
  motorFL.run(FORWARD);
  motorRL.run(FORWARD);
  motorFR.run(FORWARD);
  motorRR.run(FORWARD);
  // setup speed
  motorFL.setSpeed(movCant);
  motorRL.setSpeed(movCant);
  motorFR.setSpeed(movCant);
  motorRR.setSpeed(movCant);
  // setup time
  delay(50);
} 

void moveBackward (int movCant) {
  // setup direction
  motorFL.run(BACKWARD);
  motorRL.run(BACKWARD);
  motorFR.run(BACKWARD);
  motorRR.run(BACKWARD);
  // setup speed
  motorFL.setSpeed(movCant);
  motorRL.setSpeed(movCant);
  motorFR.setSpeed(movCant);
  motorRR.setSpeed(movCant);
  // setup time
  delay(50);
} 

void moveLeft (int movCant) {
  // setup direction
  motorFL.run(BACKWARD);
  motorRL.run(BACKWARD);
  motorFR.run(FORWARD);
  motorRR.run(FORWARD);
  // setup speed
  motorFL.setSpeed(movCant);
  motorRL.setSpeed(movCant);
  motorFR.setSpeed(movCant);
  motorRR.setSpeed(movCant);
  // setup time
  delay(50);
}  

void moveRight (int movCant) {
  // setup direction
  motorFL.run(FORWARD);
  motorRL.run(FORWARD);
  motorFR.run(BACKWARD);
  motorRR.run(BACKWARD);
  // setup speed
  motorFL.setSpeed(movCant);
  motorRL.setSpeed(movCant);
  motorFR.setSpeed(movCant);
  motorRR.setSpeed(movCant);
  // setup time
  delay(50);
}  

void moveNone () {
  motorFL.run(RELEASE);
  motorRL.run(RELEASE);
  motorFR.run(RELEASE);
  motorRR.run(RELEASE);
  delay(50);
} 




