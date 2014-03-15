#include <Wire.h> 
#include <LiquidCrystal_I2C.h>
#include <Servo.h>
#include <Event.h>
#include <Timer.h>
#include <IRremote.h>
#include <SoftwareSerial.h>

SoftwareSerial BTSerial(10, 11); // RX | TX

LiquidCrystal_I2C lcd(0x27,16,2);  // set the LCD address to 0x27 for a 16 chars and 2 line display

int inputPin = 8;
int outputPin = 9;
int pinServo = 6;
int pinIR = 4;
int ledPin = 13;      // LED

// Motor 1
int motor1Pin1 = 3;   // Motor 2 adelante
int motor1Pin2 = 2;   // Motor 2 atras
int speedPin1 = 5;    // Motor 2 aceleracion (PWM) Pin Enable del L293
int speedMotor1 = 0;  // Almacena la velocidad del motor de 0 - 255
boolean motor1SentidoGiro = false; // FALSE = LEFT | TRUE = RIGHT

int distance = 0;

Servo myservo;
int giroServo = 90;

IRrecv irrecv(pinIR);
decode_results results;

boolean lcdBackLight = true;
int modeInfoLCD = 1; // Valor que indica que datos se van a mostrar en la LCD

Timer timer;

void readDistance() {
  digitalWrite(outputPin, LOW);
  delayMicroseconds(2);
  digitalWrite(outputPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(outputPin, LOW);
  distance = pulseIn(inputPin,HIGH);
  distance= distance/58;
}

void setup(void) {
  // initialize the lcd 
  lcd.init();                      
  lcd.backlight();
  lcd.home();
  lcd.print("Distancia: ");
 
  Serial.begin(9600);
  pinMode(inputPin, INPUT);
  pinMode(outputPin, OUTPUT);
  Serial.flush();

  BTSerial.begin(9600);  // HC-05 default speed in AT command more was 38400 but changed it to 9600

  myservo.attach(pinServo); // attaches the servo on pin 9 to the servo object
//  myservo.writeMicroseconds(1500);
  myservo.write(giroServo);

  irrecv.enableIRIn(); // Start the receiver

  timer.every(500, readDistance);
  timer.every(800, refreshLCD);

  // Control Motor 1
  pinMode(motor1Pin1, OUTPUT); 
  pinMode(motor1Pin2, OUTPUT); 
  pinMode(speedPin1, OUTPUT);

  // Establece speedPinX en modo High para poder controlar la velocidad
// digitalWrite(speedPin1, HIGH);  

  motorLeft();
}

void loop(void) {
  timer.update();
  
  // Keep reading from HC-05 and send to Arduino Serial Monitor
  if (BTSerial.available()) {
    BTSerial.write(millis());
    Serial.println(BTSerial.read());
  }

  delay(50);
  
//  motorLeft();

  if (irrecv.decode(&results)) {

    switch (results.value) {
      // Boton Mode - Mostramos los datos de la Temperatura y la Luz
      case 0xFF629D: 
        if (modeInfoLCD == 1) {
          modeInfoLCD = 0;
        } else {
          modeInfoLCD++;
        }
      break;
      
      // Boton Power - Apaga o enciende el backlight del lcd
      case 0xFFA25D:
        if (lcdBackLight) {
          lcdBackLight = false;
          lcd.noBacklight();
        } else {
          lcdBackLight = true;
          lcd.backlight();
        }
      break;
      
      // Boton >>
      case 0xFF02FD:
        zepUp();
      break;
      
      // Boton <<
      case 0xFF22DD:
        zepDown();
      break;
      
      // Boton Play
      case 0xFFC23D:
        zepCenter();
      break;
      
      // Boton +
      case 0xFFA857:
        motorSpeedUp();
      break;
      
      // Boton -
      case 0xFFE01F:
        motorSpeedDown();
      break;
      
      // Boton EQ
      case 0xFF906F:
        motorSpeedStop();
      break;
      
      // Boton <-]
      case 0xFFB04F:
        if (motor1SentidoGiro) {
          motor1SentidoGiro = false;
          motorLeft();
        } else {
          motor1SentidoGiro = true;
          motorRight();
        }
      break;
      
      default:
//        lcd.clear();
//        lcd.home();
//        lcd.print("IR: ");
//        lcd.print(results.value, HEX);
        Serial.print("IR Default: ");
        Serial.println(results.value, HEX);
//        delay(1000);
      break;
    }
    
    irrecv.resume(); // Receive the next value
  }
  
}

/**
 * Funciones de control de Motores CC
 */

void motorLeft() {
  digitalWrite(motor1Pin1, HIGH);      // Establece el sentido de giro del motor 1
  digitalWrite(motor1Pin2, LOW);
}

void motorRight() {
  digitalWrite(motor1Pin1, LOW);      // Establece el sentido de giro del motor 1
  digitalWrite(motor1Pin2, HIGH);
}

void motorSpeed(int _speed) {
  analogWrite (speedPin1, _speed);  
}

void motorSpeedUp() {
  if ((speedMotor1 <= 245) && (speedMotor1 >= 0)) {
    speedMotor1 += 10;
    motorSpeed(speedMotor1);
  }
}

void motorSpeedDown() {
  if ((speedMotor1 <= 255) && (speedMotor1 >= 10)) {
    speedMotor1 -= 10;
    motorSpeed(speedMotor1);
  }
}

void motorSpeedStop() {
  speedMotor1 = 0;
  motorSpeed(speedMotor1);
}

/**
 * Funciones de control del Zeppelin
 */
 
 void zepUp() {
  if (giroServo >= 60) {
    giroServo -= 10;
    myservo.write(giroServo);
  }
 }
 
 void zepDown() {
  if (giroServo < 120) {
    giroServo += 10;
    myservo.write(giroServo);
  }
 }
 
 void zepCenter() {
  giroServo = 90;
  myservo.write(giroServo);
 }
 
 /**
  * Funciones de Dibujar LCD
  */
  
void refreshLCD() {
  // Definimos que opcion va salir por LCD
  switch (modeInfoLCD) {
    case 0:
      lcd.clear();
      lcd.home();
      lcd.print("Dist.: ");
      lcd.setCursor(7, 0);
      lcd.print("        ");
      lcd.setCursor(7, 0);
      lcd.print(distance);
      lcd.print(" CM");
    
      Serial.print("Distancia: ");
      Serial.print(distance,DEC);
      Serial.println("CM");
      
      lcd.setCursor(0, 1);
      lcd.print("Vel. Motor: ");
      lcd.setCursor(12, 1);
      lcd.print("        ");
      lcd.setCursor(12, 1);
      lcd.print(speedMotor1);
      
      Serial.print("Vel. Motor: ");
      Serial.println(speedMotor1, DEC);

      Serial.print("Giro Servo: ");
      Serial.println(giroServo, DEC);
    break;
    
    case 1:
      lcd.clear();
      lcd.home();
      lcd.print("   -# ZEPA #-   ");
      lcd.setCursor(0, 1);
      lcd.print("  garagelab.cc  ");
    break;
    
  }
}  
