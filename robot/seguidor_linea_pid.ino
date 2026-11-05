/*
 * Seguidor de línea PID
 * ESP32 + QTR-8A (analógico) + L298N + 2 motorreductores N20
 *
 * Todo se maneja con el botón BOOT del devkit (GPIO 0). No se usa ningún LED;
 * el estado se informa por el Monitor Serie (115200 baudios).
 *
 *   1) Enciende el robot -> queda esperando calibración.
 *   2) Pulsa BOOT -> 5 s de calibración: mueve el robot/sensor de lado a lado
 *      sobre la línea (negro y fondo blanco deben pasar por todos los sensores).
 *   3) Terminada la calibración (5 s) queda listo. Pulsa BOOT -> espera 1 s y arranca.
 *   4) Pulsa BOOT de nuevo para detener. Vuelve a pulsar para arrancar otra vez.
 *      (Mantén BOOT ~1 s estando detenido para recalibrar.)
 *
 * IMPORTANTE: no mantengas pulsado BOOT mientras enciendes o presionas EN/RESET,mmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmm++++++++++++++++++++++++++++++++++
 * porque el ESP32 entraría en modo de programación (bootloader).
 *
 * Alimenta el QTR-8A a 3.3 V para no exceder el límite del ADC del ESP32.
 */

// ====================== PINES ======================
const uint8_t NUM_SENSORES = 8;
const uint8_t pinesSensores[NUM_SENSORES] = { 34, 35, 32, 33, 25, 26, 27, 14 };  // S1 (izq) ... S8 (der)

// L298N - Motor izquierdo (OUT1/OUT2)
const uint8_t PIN_ENA = 18;  // PWM
const uint8_t PIN_IN1 = 16;
const uint8_t PIN_IN2 = 17;

// L298N - Motor derecho (OUT3/OUT4)
const uint8_t PIN_ENB = 22;  // PWM
const uint8_t PIN_IN3 = 19;
const uint8_t PIN_IN4 = 21;

const uint8_t PIN_BOTON = 0;  // botón BOOT del devkit (activo en LOW, ya tiene pull-up externo)

// ====================== CONFIGURACIÓN SENSORES ======================
const bool LINEA_NEGRA = true;               // true: línea negra sobre fondo blanco
const unsigned long TIEMPO_CALIBRACION_MS = 5000;
const int MUESTRAS_ADC = 4;                  // promedio por sensor (reduce ruido)
const int UMBRAL_PERDIDA = 500;              // si ningún sensor supera esto -> línea perdida
const int RUIDO_MIN = 150;                   // valores por debajo se ignoran en el cálculo
const int RANGO_MINIMO = 300;                // rango de calibración mínimo para confiar en un sensor
const int CENTRO = 3500;

// Si un motor gira al revés, cambia su valor a true
const bool INVERTIR_MOTOR_IZQ = true;
const bool INVERTIR_MOTOR_DER = true;

// ----- PID (ajústalo en pista) -----
float KP =  0.30;  
float KI = 0.0;
float KD = 1;  
const float INTEGRAL_MAX = 3000;             // anti-windup

// ----- Velocidades (0-255) -----
const int VEL_BASE    = 235;
const int VEL_MAX     = 255;
const int VEL_PERDIDA = 215;                 // giro sobre su eje cuando se pierde la línea

// ----- Lazo de control -----
const unsigned long PERIODO_PID_US = 5000;   // 5 ms = 200 Hz

// ----- PWM -----
const int PWM_FREQ = 5000;
const int PWM_RES  = 8;                      // 8 bits -> 0..255
const int CANAL_A  = 0;
const int CANAL_B  = 1;

// ====================== ESTADO ======================
int minimo[NUM_SENSORES];
int maximo[NUM_SENSORES];
int crudo[NUM_SENSORES];
int norm[NUM_SENSORES];

int ultimaPosicion = CENTRO;
float integral = 0;
int errorAnterior = 0;
unsigned long ultimoPID = 0;

enum Estado { ESPERA_CAL, ESPERA_INICIO, CORRIENDO };
Estado estado = ESPERA_CAL;

// ====================== PWM (compatible con core 2.x y 3.x) ======================
void pwmInicio() {
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcAttach(PIN_ENA, PWM_FREQ, PWM_RES);
  ledcAttach(PIN_ENB, PWM_FREQ, PWM_RES);
#else
  ledcSetup(CANAL_A, PWM_FREQ, PWM_RES);
  ledcSetup(CANAL_B, PWM_FREQ, PWM_RES);
  ledcAttachPin(PIN_ENA, CANAL_A);
  ledcAttachPin(PIN_ENB, CANAL_B);
#endif
}

void pwmEscribir(uint8_t pin, int canal, int duty) {
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcWrite(pin, duty);
#else
  ledcWrite(canal, duty);
#endif
}

// ====================== MOTORES ======================
// velocidad: -255 (reversa) ... +255 (adelante)
void setMotor(uint8_t pinA, uint8_t pinB, uint8_t pinEn, int canal, int velocidad, bool invertir) {
  velocidad = constrain(velocidad, -VEL_MAX, VEL_MAX);
  if (invertir) velocidad = -velocidad;

  if (velocidad > 0) {
    digitalWrite(pinA, HIGH);
    digitalWrite(pinB, LOW);
  } else if (velocidad < 0) {
    digitalWrite(pinA, LOW);
    digitalWrite(pinB, HIGH);
  } else {
    digitalWrite(pinA, LOW);
    digitalWrite(pinB, LOW);
  }
  pwmEscribir(pinEn, canal, abs(velocidad));
}

void motores(int izq, int der) {
  setMotor(PIN_IN1, PIN_IN2, PIN_ENA, CANAL_A, izq, INVERTIR_MOTOR_IZQ);
  setMotor(PIN_IN3, PIN_IN4, PIN_ENB, CANAL_B, der, INVERTIR_MOTOR_DER);
}

void detener() {
  motores(0, 0);
}

// ====================== SENSORES ======================
void leerCrudo() {
  for (uint8_t i = 0; i < NUM_SENSORES; i++) {
    long suma = 0;
    for (int m = 0; m < MUESTRAS_ADC; m++) suma += analogRead(pinesSensores[i]);
    crudo[i] = suma / MUESTRAS_ADC;
  }
}

void calibrar() {
  for (uint8_t i = 0; i < NUM_SENSORES; i++) {
    minimo[i] = 4095;
    maximo[i] = 0;
  }

  Serial.println("Calibrando durante 5 s: mueve el sensor de lado a lado sobre la linea...");

  unsigned long inicio = millis();
  while (millis() - inicio < TIEMPO_CALIBRACION_MS) {
    leerCrudo();
    for (uint8_t i = 0; i < NUM_SENSORES; i++) {
      if (crudo[i] < minimo[i]) minimo[i] = crudo[i];
      if (crudo[i] > maximo[i]) maximo[i] = crudo[i];
    }
    delay(5);
  }

  Serial.println("Calibracion terminada\nSensor\tMin\tMax");
  for (uint8_t i = 0; i < NUM_SENSORES; i++) {
    Serial.print("S"); Serial.print(i + 1); Serial.print('\t');
    Serial.print(minimo[i]); Serial.print('\t');
    Serial.print(maximo[i]);
    if (maximo[i] - minimo[i] < RANGO_MINIMO) Serial.print("\t<-- rango pequeno, se ignorara este sensor");
    Serial.println();
  }
}

// Devuelve la posición (0-7000) o -1 si no ve línea. Llena norm[].
int calcularPosicion() {
  long suma = 0;
  long sumaPonderada = 0;
  bool veLinea = false;

  for (uint8_t i = 0; i < NUM_SENSORES; i++) {
    int rango = maximo[i] - minimo[i];
    long v;

    if (rango < RANGO_MINIMO) {
      v = 0;                       // sensor sin calibración confiable: se ignora
    } else {
      v = (long)(crudo[i] - minimo[i]) * 1000 / rango;
      v = constrain(v, 0, 1000);
      if (!LINEA_NEGRA) v = 1000 - v;
    }
    norm[i] = v;

    if (v > UMBRAL_PERDIDA) veLinea = true;

    long w = (v > RUIDO_MIN) ? v : 0;   // filtra ruido de fondo
    suma += w;
    sumaPonderada += w * (long)i * 1000;
  }

  if (!veLinea || suma == 0) return -1;
  return sumaPonderada / suma;
}

// ====================== PID ======================
void reiniciarPID() {
  integral = 0;
  errorAnterior = 0;
  ultimaPosicion = CENTRO;
  ultimoPID = micros();
}

void controlPID() {
  leerCrudo();
  int pos = calcularPosicion();

  // --- Línea perdida: gira hacia el lado donde se vio por última vez ---
  if (pos < 0) {
    if (ultimaPosicion < CENTRO) motores(-VEL_PERDIDA, VEL_PERDIDA);   // gira a la izquierda
    else                         motores(VEL_PERDIDA, -VEL_PERDIDA);   // gira a la derecha
    integral = 0;
    return;
  }

  ultimaPosicion = pos;
  int error = pos - CENTRO;  // negativo: línea a la izquierda | positivo: línea a la derecha

  integral += error;
  integral = constrain(integral, -INTEGRAL_MAX, INTEGRAL_MAX);
  int derivada = error - errorAnterior;
  errorAnterior = error;

  float correccion = KP * error + KI * integral + KD * derivada;

  // Línea a la derecha (error > 0) -> motor izq más rápido, motor der más lento
  int velIzq = VEL_BASE + (int)correccion;
  int velDer = VEL_BASE - (int)correccion;

  motores(velIzq, velDer);
}

// ====================== BOTÓN (BOOT, GPIO 0) ======================
// Devuelve 0 = sin pulsar, 1 = pulsación corta, 2 = pulsación larga (>1 s)
int leerBoton() {
  if (digitalRead(PIN_BOTON) != LOW) return 0;
  delay(30);  // antirrebote
  if (digitalRead(PIN_BOTON) != LOW) return 0;

  unsigned long t = millis();
  while (digitalRead(PIN_BOTON) == LOW) {
    if (millis() - t > 1000) {
      while (digitalRead(PIN_BOTON) == LOW) delay(5);
      return 2;
    }
  }
  return 1;
}

// Calibra y deja el robot listo para arrancar
void procesoCalibracion() {
  delay(300);  // tiempo para apartar el dedo del botón
  calibrar();
  estado = ESPERA_INICIO;
  Serial.println("Listo. Pulsa BOOT para arrancar.");
}

// ====================== PROGRAMA PRINCIPAL ======================
void setup() {
  Serial.begin(115200);

  pinMode(PIN_IN1, OUTPUT);
  pinMode(PIN_IN2, OUTPUT);
  pinMode(PIN_IN3, OUTPUT);
  pinMode(PIN_IN4, OUTPUT);
  pinMode(PIN_BOTON, INPUT_PULLUP);
  pwmInicio();
  detener();

  analogReadResolution(12);
  analogSetAttenuation(ADC_11db);
  for (uint8_t i = 0; i < NUM_SENSORES; i++) pinMode(pinesSensores[i], INPUT);

  // Sin calibrar: usa el rango completo del ADC hasta que se calibre
  for (uint8_t i = 0; i < NUM_SENSORES; i++) {
    minimo[i] = 0;
    maximo[i] = 4095;
  }

  Serial.println("Seguidor PID listo. Pulsa BOOT para calibrar.");
}

void loop() {
  switch (estado) {
    case ESPERA_CAL:
      if (leerBoton()) procesoCalibracion();
      break;

    case ESPERA_INICIO: {
      int b = leerBoton();
      if (b == 1) {
        Serial.println("Arrancando en 1 s...");
        delay(1000);          // tiempo para soltar el robot sobre la pista
        reiniciarPID();
        estado = CORRIENDO;
        Serial.println("CORRIENDO");
      } else if (b == 2) {
        Serial.println("Recalibrando...");
        procesoCalibracion(); // pulsación larga: recalibrar directamente
      }
      break;
    }

    case CORRIENDO:
      if (micros() - ultimoPID >= PERIODO_PID_US) {
        ultimoPID += PERIODO_PID_US;
        controlPID();
      }
      if (digitalRead(PIN_BOTON) == LOW) {
        detener();
        while (digitalRead(PIN_BOTON) == LOW) delay(5);
        delay(100);
        estado = ESPERA_INICIO;
        Serial.println("DETENIDO. Pulsa BOOT para arrancar (largo = recalibrar).");
      }
      break;
  }
}
