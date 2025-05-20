#include <QTRSensors.h>

QTRSensors qtr;

const uint8_t SensorCount = 1;
uint16_t sensorValues[SensorCount];

// Pines y constantes

const int pinEcho = 11;
const int standby = 12;
const int btRx = 0;
const int btTx = 1;
const int pinLed = 8;
const int motorDerAdelante = 2;
const int motorDerAtras = 5;
const int motorIzqAdelante = 3;
const int motorIzqAtras = 4;
const int pwmMotorDer = 9;
const int pwmMotorIzq = 10;

const int pinSensorSharp = A1; // Pin analógico para el sensor Sharp

double distanciaSharp;




int paso = 0;
int tiempo = 0;
int enLinea = 0;

int modo = 1;  // Modo de operación actual

// Configuración de constantes
const int umbralDistancia = 30;   // Distancia mínima en cm para encontrar al oponente
const int velocidadMaxima = 255;  // Velocidad máxima
const int velocidad80 = 220;      // Velocidad al 80%

void setup() {

  //  // configure the sensors
  // qtr.setTypeRC();
  // qtr.setSensorPins((const uint8_t[]){3}, SensorCount);
  // qtr.setEmitterPin(2);

  // delay(500);
  // pinMode(LED_BUILTIN, OUTPUT);
  // digitalWrite(LED_BUILTIN, HIGH); // turn on Arduino's LED to indicate we are in calibration mode

  // // 2.5 ms RC read timeout (default) * 10 reads per calibrate() call
  // // = ~25 ms per calibrate() call.
  // // Call calibrate() 400 times to make calibration take about 10 seconds.
  // for (uint16_t i = 0; i < 400; i++)
  // {
  //   qtr.calibrate();
  // }
  // digitalWrite(LED_BUILTIN, LOW); // turn off Arduino's LED to indicate we are through with calibration

  // // print the calibration minimum values measured when emitters were on
  // Serial.begin(9600);
  // for (uint8_t i = 0; i < SensorCount; i++)
  // {
  //   Serial.print(qtr.calibrationOn.minimum[i]);
  //   Serial.print(' ');
  // }
  // Serial.println();

  // // print the calibration maximum values measured when emitters were on
  // for (uint8_t i = 0; i < SensorCount; i++)
  // {
  //   Serial.print(qtr.calibrationOn.maximum[i]);
  //   Serial.print(' ');
  // }
  // Serial.println();
  // Serial.println();
  // delay(1000);
  // Configuración de pines
  
  pinMode(standby, OUTPUT);
  pinMode(motorDerAdelante, OUTPUT);
  pinMode(motorDerAtras, OUTPUT);
  pinMode(motorIzqAdelante, OUTPUT);
  pinMode(motorIzqAtras, OUTPUT);
  pinMode(pwmMotorDer, OUTPUT);
  pinMode(pwmMotorIzq, OUTPUT);
  pinMode(pinLed, OUTPUT);

  // Comunicación serial para Bluetooth
  Serial.begin(9600);
}

void loop() {
  
  // Leer el modo de operación desde Bluetooth
  if (Serial.available() > 0) {
    char comando = Serial.read();
    if (comando >= '0' && comando <= '3') {
      modo = comando - '0';  // Convierte char a int
      detenerMotores();      // Detiene los motores
      digitalWrite(pinLed, LOW);
    }
  }

  // Actuar según el modo
  switch (modo) {
    case 1:

      modoAutonomo1();
      break;
    case 2:

      modoAutonomo2();
      break;
    case 3:

      modoControlManual();
      break;
    default:
      detenerMotores();  // Modo 0 o invalido: detener motores
      tiempo = 0;
      digitalWrite(pinLed, LOW);

      paso = 0;
      break;
  }
}

// Función para obtener la distancia del sensor ultrasónico
// long obtenerDistancia() {
//   digitalWrite(pinTrig, LOW);
//   delayMicroseconds(2);
//   digitalWrite(pinTrig, HIGH);
//   delayMicroseconds(10);
//   digitalWrite(pinTrig, LOW);
//   long duracion = pulseIn(pinEcho, HIGH);
//   return duracion * 0.034 / 2;
// }



// double obtenerDistancia(int _t, int _e) {
//   unsigned long dur = 0;
//   digitalWrite(_t, LOW);
//   delayMicroseconds(5);
//   digitalWrite(_t, HIGH);
//   delayMicroseconds(10);
//   digitalWrite(_t, LOW);
//   dur = pulseIn(_e, HIGH, 18000);
//   if (dur == 0) return 999.0;
//   return (dur / 57);
// }

// Función para obtener la distancia del sensor Sharp
float distancia(int n) {
  long suma = 0;
  for (int i = 0; i < n; i++) {
    suma = suma + analogRead(pinSensorSharp);
  }
  float adc = suma / n;
  float distancia_cm = 17569.7 * pow(adc, -1.2062);
  return distancia_cm;
}


// Función para detener todos los motores
void detenerMotores() {
  digitalWrite(motorDerAdelante, LOW);
  digitalWrite(motorDerAtras, LOW);
  digitalWrite(motorIzqAdelante, LOW);
  digitalWrite(motorIzqAtras, LOW);
}

// Función para activar los motores para avanzar
void avanzar(int velocidad) {
  detenerMotores();
  digitalWrite(motorDerAdelante, HIGH);
  digitalWrite(motorDerAtras, LOW);
  digitalWrite(motorIzqAdelante, HIGH);
  digitalWrite(motorIzqAtras, LOW);
  analogWrite(pwmMotorDer, velocidad);
  analogWrite(pwmMotorIzq, velocidad);
}

// Función para activar los motores para retroceder
void retroceder(int velocidad) {
  detenerMotores();
  digitalWrite(motorDerAdelante, LOW);
  digitalWrite(motorDerAtras, HIGH);
  digitalWrite(motorIzqAdelante, LOW);
  digitalWrite(motorIzqAtras, HIGH);
  analogWrite(pwmMotorDer, velocidad);
  analogWrite(pwmMotorIzq, velocidad);
}

// Función para girar a la derecha
void girarDerecha(int velocidad) {
  detenerMotores();
  digitalWrite(motorDerAdelante, LOW);
  digitalWrite(motorDerAtras, HIGH);
  digitalWrite(motorIzqAdelante, HIGH);
  digitalWrite(motorIzqAtras, LOW);
  analogWrite(pwmMotorDer, velocidad);
  analogWrite(pwmMotorIzq, velocidad);
}

// Función para girar a la izquierda
void girarIzquierda(int velocidad) {
  detenerMotores();
  digitalWrite(motorDerAdelante, HIGH);
  digitalWrite(motorDerAtras, LOW);
  digitalWrite(motorIzqAdelante, LOW);
  digitalWrite(motorIzqAtras, HIGH);
  analogWrite(pwmMotorDer, velocidad);
  analogWrite(pwmMotorIzq, velocidad);
}

// Modo 1: Autónomo sin giros

void modoAutonomo1() {
  digitalWrite(standby, HIGH);

  //  // read calibrated sensor values and obtain a measure of the line position
  // // from 0 to 5000 (for a white line, use readLineWhite() instead)
  // uint16_t position = qtr.readLineBlack(sensorValues);

  // // print the sensor values as numbers from 0 to 1000, where 0 means maximum
  // // reflectance and 1000 means minimum reflectance, followed by the line
  // // position
  // for (uint8_t i = 0; i < SensorCount; i++)
  // {
  //   Serial.print(sensorValues[i]);
  //    enLinea = sensorValues[i];

  //   Serial.print('\t');
  // }


  distanciaSharp = distancia(20);
 
  if (tiempo == 0) {
    digitalWrite(pinLed, HIGH);
    delay(100);
    digitalWrite(pinLed, LOW);
    
    tiempo = 2;
  } else {
    //if (enLinea<=500) {
      // digitalWrite(pinLed, HIGH);
      // retroceder(velocidad80);
      // delay(400);
      // girarDerecha(velocidad80);
      // delay(200);

    //}else 
     if (distanciaSharp < umbralDistancia) {
      digitalWrite(pinLed, HIGH);
      avanzar(velocidadMaxima);
    } else {
      digitalWrite(pinLed, LOW);
      girarDerecha(velocidad80);
    }
  }
}

// Modo 2: Autónomo con giros
void modoAutonomo2() {
  distanciaSharp = distancia(20);
  
  if (tiempo == 0) {
    delay(5000);
    tiempo = 2;
  } else {
    if (paso == 0) {
      if (enLinea<=500) {
        retroceder(velocidad80);
        delay(500);
        girarDerecha(velocidad80);
        delay(200);
        paso = 2;
      } else {
        avanzar(velocidad80);
      }
    } else if (enLinea<=500) {
      retroceder(velocidad80);
      delay(400);
      girarDerecha(velocidad80);
      delay(200);
    } else if (distanciaSharp < umbralDistancia) {
      avanzar(velocidadMaxima);
    } else {
      girarDerecha(velocidadMaxima);
    }
  }
}

// Modo 3: Control manual por Bluetooth
void modoControlManual() {
  if (Serial.available() > 0) {
    char comando = Serial.read();
    switch (comando) {
      case 'F':  // Avanzar
        avanzar(velocidad80);
        break;
      case 'B':  // Retroceder
        retroceder(velocidad80);
        break;
      case 'L':  // Girar a la izquierda
        girarIzquierda(velocidad80);
        break;
      case 'R':  // Girar a la derecha
        girarDerecha(velocidad80);
        break;
      default:
        detenerMotores();
        break;
    }
  }
}