


// Pines y constantes


#define ENA  16  // PWM para motor A
#define ENB  4   // PWM para motor B
#define PWM_CHANNEL_A  0
#define PWM_CHANNEL_B  1
#define PWM_FREQ       5000
#define PWM_RESOLUTION 8

int IN1 = 33; 
int IN2 = 32; 
int IN3 = 12;
int IN4 = 13;
int JSUMO = 14;
int JUEZ_GO = 15;





int paso = 0;
int tiempo = 0;
int enLinea = 0;
// Función para detener todos los motores
void detenerMotores() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
}

// Función para activar los motores para avanzar
void avanzar(int velocidad) {
  detenerMotores();
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  analogWrite(ENA, velocidad);
  analogWrite(ENB, velocidad);
}

// Función para activar los motores para retroceder
void retroceder(int velocidad) {
  detenerMotores();
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  analogWrite(ENA, velocidad);
  analogWrite(ENB, velocidad);
}

// Función para girar a la derecha
void girarDerecha(int velocidad) {
  detenerMotores();
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  analogWrite(ENA, velocidad);
  analogWrite(ENB, velocidad);
}

// Función para girar a la izquierda
void girarIzquierda(int velocidad) {
  detenerMotores();
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  analogWrite(ENA, velocidad);
  analogWrite(ENB, velocidad);
}









void setup() {
  
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(JSUMO, INPUT);
  pinMode(JUEZ_GO, INPUT);

    // Configurar PWM con LEDC
  ledcSetup(PWM_CHANNEL_A, PWM_FREQ, PWM_RESOLUTION);
  ledcAttachPin(ENA, PWM_CHANNEL_A);

  ledcSetup(PWM_CHANNEL_B, PWM_FREQ, PWM_RESOLUTION);
  ledcAttachPin(ENB, PWM_CHANNEL_B);

}

void loop() {


  bool DIST = digitalRead(JSUMO);

  while (digitalRead(JUEZ_GO) == LOW) {
    detenerMotores();
  }
  
  if(DIST==true){
    avanzar(255);

  }else{
    girarDerecha(255);
  }

}
  
  


