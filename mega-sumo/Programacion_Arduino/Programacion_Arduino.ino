

// Pines y constantes
const int sensorLineaIzq = 6;
const int sensorLineaDer = 7;


// Pines para los motores delanteros
const int motorDelanteroDerAdelante = 2;
const int motorDelanteroDerAtras = 3;
const int motorDelanteroIzqAdelante = 4;
const int motorDelanteroIzqAtras = 5;

// Pines para los motores traseros
const int motorTraseroDerAdelante = 8;
const int motorTraseroDerAtras = 9;
const int motorTraseroIzqAdelante = 10;
const int motorTraseroIzqAtras = 11;

const int y = 22;
const int b = 23;
const int a = 24;
const int x = 25;

const int up = 26;
const int down = 27;
const int left = 28;
const int right = 29;

int dir = 0;





const int pinSensorSharp1 = A0; // Pin analógico para el sensor Sharp 1

double distanciaSharp1;


int paso = 0;

bool pasoA = false;
bool pasoB = false;
bool pasoX = false;
bool pasoY = false;

int tiempo = 0;

String modo ="";

// Configuración de constantes
const int umbralDistancia = 30; // Distancia mínima en cm para encontrar al oponente


void setup() {
  // Configuración de pines
  pinMode(sensorLineaIzq, INPUT);
  pinMode(sensorLineaDer, INPUT);
  pinMode(pinSensorSharp1, INPUT);


  // Configuración para motores traseros
  pinMode(motorTraseroDerAdelante, OUTPUT);
  pinMode(motorTraseroDerAtras, OUTPUT);
  pinMode(motorTraseroIzqAdelante, OUTPUT);
  pinMode(motorTraseroIzqAtras, OUTPUT);

  // Configuración para motores delanteros
  pinMode(motorDelanteroDerAdelante, OUTPUT);
  pinMode(motorDelanteroDerAtras, OUTPUT);
  pinMode(motorDelanteroIzqAdelante, OUTPUT);
  pinMode(motorDelanteroIzqAtras, OUTPUT);

   pinMode(y, INPUT);
  pinMode(b, INPUT);
   pinMode(a, INPUT);
  pinMode(x, INPUT);


   pinMode(up, INPUT);
  pinMode(down, INPUT);
   pinMode(left, INPUT);
  pinMode(right, INPUT);

  Serial.begin(9600);
}


void loop() {

   // Actuar según el modo
  if (digitalRead(up) == true) {
   dir=1;
  }
  if (digitalRead(down) == true) {
    dir=2;
  }
  if (digitalRead(left) == true) {
    dir=3;
  }
  if (digitalRead(right) == true) {
    dir=4;
      
  }
  

  // Actuar según el modo
  if (digitalRead(x) == true) {
   pasoX = true;
   pasoB = false;
   pasoA = false;
   pasoY = false;
  }
  if (digitalRead(b) == true) {
     pasoX = false;
   pasoB = true;
   pasoA = false;
   pasoY = false;
  }
  if (digitalRead(a) == true) {
     pasoX = false;
   pasoB = false;
   pasoA = true;
   pasoY = false;
  }
  if (digitalRead(y) == true) {
    pasoX = false;
   pasoB = false;
   pasoA = false;
   pasoY = true;
      
  }


   // Actuar según el modo
  if (pasoX == true) {
    modoAutonomo1();
  }
  if (pasoB == true) {
    modoAutonomo2();
  }
  if (pasoA == true) {
    modoControlManual(dir);
  }
  if (pasoY == true) {
    detenerMotores();
      tiempo = 0;
      paso = 0;
  }
   
      
  
    // case "B":
    //   modoAutonomo2();
    //   break;
    // case "B":
    //   modoControlManual();
    //   break;
    // default:
    //   detenerMotores(); // Modo 0 o inválido: detener motores
    
    //   break;
  }


// Función para obtener la distancia del sensor Sharp 1
float distancia1(int n) {
  long suma = 0;
  for (int i = 0; i < n; i++) {
    suma = suma + analogRead(pinSensorSharp1);
  }
  float adc = suma / n;
  float distancia_cm = 17569.7 * pow(adc, -1.2062);
  return distancia_cm;
}


// Función para detener todos los motores
void detenerMotores() {
  // Apagar motores traseros
  digitalWrite(motorTraseroDerAdelante, LOW);
  digitalWrite(motorTraseroDerAtras, LOW);
  digitalWrite(motorTraseroIzqAdelante, LOW);
  digitalWrite(motorTraseroIzqAtras, LOW);

  // Apagar motores delanteros
  digitalWrite(motorDelanteroDerAdelante, LOW);
  digitalWrite(motorDelanteroDerAtras, LOW);
  digitalWrite(motorDelanteroIzqAdelante, LOW);
  digitalWrite(motorDelanteroIzqAtras, LOW);
}

// Función para activar los motores para avanzar
void avanzar() {
  detenerMotores();
  // Motores traseros
  digitalWrite(motorTraseroDerAdelante, HIGH);
  digitalWrite(motorTraseroDerAtras, LOW);
  digitalWrite(motorTraseroIzqAdelante, HIGH);
  digitalWrite(motorTraseroIzqAtras, LOW);
 

  // Motores delanteros
  digitalWrite(motorDelanteroDerAdelante, HIGH);
  digitalWrite(motorDelanteroDerAtras, LOW);
  digitalWrite(motorDelanteroIzqAdelante, HIGH);
  digitalWrite(motorDelanteroIzqAtras, LOW);

}

// Función para activar los motores para retroceder
void retroceder() {
  detenerMotores();
  // Motores traseros
  digitalWrite(motorTraseroDerAdelante, LOW);
  digitalWrite(motorTraseroDerAtras, HIGH);
  digitalWrite(motorTraseroIzqAdelante, LOW);
  digitalWrite(motorTraseroIzqAtras, HIGH);


  // Motores delanteros
  digitalWrite(motorDelanteroDerAdelante, LOW);
  digitalWrite(motorDelanteroDerAtras, HIGH);
  digitalWrite(motorDelanteroIzqAdelante, LOW);
  digitalWrite(motorDelanteroIzqAtras, HIGH);

}

// Función para girar a la derecha
void girarDerecha() {
  detenerMotores();
  // Motores traseros
  digitalWrite(motorTraseroDerAdelante, LOW);
  digitalWrite(motorTraseroDerAtras, HIGH);
  digitalWrite(motorTraseroIzqAdelante, HIGH);
  digitalWrite(motorTraseroIzqAtras, LOW);


  // Motores delanteros
  digitalWrite(motorDelanteroDerAdelante, LOW);
  digitalWrite(motorDelanteroDerAtras, HIGH);
  digitalWrite(motorDelanteroIzqAdelante, HIGH);
  digitalWrite(motorDelanteroIzqAtras, LOW);

}

// Función para girar a la izquierda
void girarIzquierda() {
  detenerMotores();
  // Motores traseros
  digitalWrite(motorTraseroDerAdelante, HIGH);
  digitalWrite(motorTraseroDerAtras, LOW);
  digitalWrite(motorTraseroIzqAdelante, LOW);
  digitalWrite(motorTraseroIzqAtras, HIGH);


  // Motores delanteros
  digitalWrite(motorDelanteroDerAdelante, HIGH);
  digitalWrite(motorDelanteroDerAtras, LOW);
  digitalWrite(motorDelanteroIzqAdelante, LOW);
  digitalWrite(motorDelanteroIzqAtras, HIGH);

}

// Modo 1: Autónomo sin giros
void modoAutonomo1() {
   Serial.println("Modo Autonomo 1");
  distanciaSharp1 = distancia1(20);
 
 
  if (tiempo == 0) {
    //Este if hace que espere un tiempo determinado antes de comenzar la funcion
    // delay(4000);
    // delay(1000);
    tiempo = 2;
  } else {

    if (distanciaSharp1 < umbralDistancia ) {
      
      avanzar();
    }
    if (distanciaSharp1 > umbralDistancia ) {
      
      girarDerecha();
    }

   
  }
}

// Modo 2: Autónomo con giros
void modoAutonomo2() {
  avanzar();
}

// Modo 3: Control manual por Bluetooth
void modoControlManual(int direccion) {


    // Actuar según el modo
  if (direccion =1) {
   
   avanzar();
  }
  if (direccion =2) {
  
     retroceder();
  }
  if (direccion =3) {

     girarIzquierda();
  }
  if (direccion =4) {
  
    girarDerecha();
      
  }
  if (direccion =0) {
    detenerMotores();
    girarDerecha();
      
  }
    }
  

