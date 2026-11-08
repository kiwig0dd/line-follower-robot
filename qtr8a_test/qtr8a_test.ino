/*
 * Diagnóstico QTR-8A (analógico) con ESP32
 *
 *  BOOT corto  : calibrar / iniciar / detener
 *  BOOT largo  : recalibrar (estando detenido)
 *  Serial 'm'  : cambia modo de salida (BARRAS -> CRUDO -> PLOTTER)
 *  Serial 'c'  : recalibrar
 *
 * Los umbrales son los MISMOS que en seguidor_linea_pid.ino
 */

// ====================== PINES ======================
const uint8_t NUM_SENSORES = 8;
const uint8_t pinesSensores[NUM_SENSORES] = { 34, 35, 32, 33, 25, 26, 27, 14 };  // S1 (izq) ... S8 (der)
const uint8_t PIN_BOTON = 0;

// ====================== CONFIGURACIÓN (igual que el PID) ======================
const bool LINEA_NEGRA = true;
const unsigned long TIEMPO_CALIBRACION_MS = 5000;
const int MUESTRAS_ADC = 4;
const int UMBRAL_PERDIDA = 500;
const int RUIDO_MIN = 150;
const int RANGO_MINIMO = 150;
const int CENTRO = 3500;
const int MARGEN_CENTRO = 500;  // zona considerada "centrada"
const unsigned long PERIODO_PRINT_MS = 100;

// ====================== ESTADO ======================
int minimo[NUM_SENSORES];
int maximo[NUM_SENSORES];
int crudo[NUM_SENSORES];
int norm[NUM_SENSORES];
int ultimaPosicion = CENTRO;
unsigned long ultimoPrint = 0;
unsigned long tLecturaUs = 0;

enum Estado { ESPERA_CAL, ESPERA_INICIO, DIAGNOSTICO };
Estado estado = ESPERA_CAL;

enum Modo { M_BARRAS, M_CRUDO, M_PLOTTER };
Modo modo = M_BARRAS;
const char* nombreModo[] = { "BARRAS", "CRUDO", "PLOTTER" };

// ====================== SENSORES ======================
void leerCrudo() {
  unsigned long t0 = micros();
  for (uint8_t i = 0; i < NUM_SENSORES; i++) {
    long suma = 0;
    for (int m = 0; m < MUESTRAS_ADC; m++) suma += analogRead(pinesSensores[i]);
    crudo[i] = suma / MUESTRAS_ADC;
  }
  tLecturaUs = micros() - t0;
}

void calibrar() {
  for (uint8_t i = 0; i < NUM_SENSORES; i++) {
    minimo[i] = 4095;
    maximo[i] = 0;
  }

  Serial.println("Calibrando 5 s: mueve el sensor de lado a lado sobre la linea...");
  unsigned long inicio = millis();
  while (millis() - inicio < TIEMPO_CALIBRACION_MS) {
    leerCrudo();
    for (uint8_t i = 0; i < NUM_SENSORES; i++) {
      if (crudo[i] < minimo[i]) minimo[i] = crudo[i];
      if (crudo[i] > maximo[i]) maximo[i] = crudo[i];
    }
    delay(5);
  }

  Serial.println("Calibracion terminada\nSensor\tMin\tMax\tRango\tEstado");
  for (uint8_t i = 0; i < NUM_SENSORES; i++) {
    int rango = maximo[i] - minimo[i];
    Serial.printf("S%d\t%d\t%d\t%d\t", i + 1, minimo[i], maximo[i], rango);
    if (rango < RANGO_MINIMO) Serial.print("RANGO PEQUENO (se ignorara)");
    else if (maximo[i] >= 4090) Serial.print("SATURADO (revisa alimentacion 3.3V / altura)");
    else if (maximo[i] == 0) Serial.print("SIN SEÑAL (revisa cable)");
    else Serial.print("OK");
    Serial.println();
  }
}

// Devuelve posición 0-7000 o -1 si no ve línea. Llena norm[].
int calcularPosicion() {
  long suma = 0, sumaPonderada = 0;
  bool veLinea = false;

  for (uint8_t i = 0; i < NUM_SENSORES; i++) {
    int rango = maximo[i] - minimo[i];
    long v;
    if (rango < RANGO_MINIMO) {
      v = 0;
    } else {
      v = (long)(crudo[i] - minimo[i]) * 1000 / rango;
      v = constrain(v, 0, 1000);
      if (!LINEA_NEGRA) v = 1000 - v;
    }
    norm[i] = v;

    if (v > UMBRAL_PERDIDA) veLinea = true;

    long w = (v > RUIDO_MIN) ? v : 0;
    suma += w;
    sumaPonderada += w * (long)i * 1000;
  }

  if (!veLinea || suma == 0) return -1;
  return sumaPonderada / suma;
}

// ====================== SALIDA ======================
// Un carácter por sensor según intensidad (0-1000)
char nivel(int v) {
  const char* esc = " .:-=+*#%@";
  return esc[constrain(v / 101, 0, 9)];
}

void imprimir(int pos) {
  switch (modo) {
    case M_BARRAS:
      Serial.print("[");
      for (uint8_t i = 0; i < NUM_SENSORES; i++) Serial.print(nivel(norm[i]));
      Serial.print("] ");
      for (uint8_t i = 0; i < NUM_SENSORES; i++) Serial.printf("%4d ", norm[i]);
      Serial.printf("| Pos: %5d | ", pos);
      if (pos < 0) {
        Serial.println(ultimaPosicion < CENTRO ? "PERDIDA (se salio por la IZQUIERDA)"
                                               : "PERDIDA (se salio por la DERECHA)");
      } else if (pos < CENTRO - MARGEN_CENTRO) {
        Serial.println("Linea a la IZQUIERDA");
      } else if (pos > CENTRO + MARGEN_CENTRO) {
        Serial.println("Linea a la DERECHA");
      } else {
        Serial.println("CENTRADO");
      }
      Serial.printf("   lectura 8 sensores: %lu us\n", tLecturaUs);
      break;

    case M_CRUDO:
      Serial.print("ADC crudo: ");
      for (uint8_t i = 0; i < NUM_SENSORES; i++) Serial.printf("%4d ", crudo[i]);
      Serial.printf("| lectura: %lu us\n", tLecturaUs);
      break;

    case M_PLOTTER:  // formato etiqueta:valor para el Serial Plotter
      for (uint8_t i = 0; i < NUM_SENSORES; i++) Serial.printf("S%d:%d,", i + 1, norm[i]);
      Serial.printf("Pos:%d\n", pos < 0 ? -100 : pos / 7);  // pos escalada a 0-1000
      break;
  }
}

// ====================== BOTÓN ======================
// 0 = nada, 1 = corta, 2 = larga (>1 s)
int leerBoton() {
  if (digitalRead(PIN_BOTON) != LOW) return 0;
  delay(30);
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

void procesoCalibracion() {
  delay(300);
  calibrar();
  ultimaPosicion = CENTRO;
  estado = ESPERA_INICIO;
  Serial.println("Listo. Pulsa BOOT para iniciar el diagnostico.");
}

// Comandos por serial: 'm' modo, 'c' recalibrar
void procesarSerial() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == 'm') {
      modo = (Modo)((modo + 1) % 3);
      Serial.printf("Modo: %s\n", nombreModo[modo]);
    } else if (c == 'c') {
      procesoCalibracion();
    }
  }
}

// ====================== PROGRAMA ======================
void setup() {
  Serial.begin(115200);
  pinMode(PIN_BOTON, INPUT_PULLUP);

  analogReadResolution(12);
  analogSetAttenuation(ADC_11db);
  for (uint8_t i = 0; i < NUM_SENSORES; i++) pinMode(pinesSensores[i], INPUT);

  for (uint8_t i = 0; i < NUM_SENSORES; i++) {
    minimo[i] = 0;
    maximo[i] = 4095;
  }
  Serial.println("Diagnostico QTR-8A. BOOT = calibrar | 'm' = cambiar modo | 'c' = recalibrar");
}

void loop() {
  procesarSerial();

  switch (estado) {
    case ESPERA_CAL:
      if (leerBoton()) procesoCalibracion();
      break;

    case ESPERA_INICIO:
      {
        int b = leerBoton();
        if (b == 1) {
          Serial.println("Iniciando en 1 s...");
          delay(1000);
          estado = DIAGNOSTICO;
          Serial.printf("--- DIAGNOSTICO (modo %s) ---\n", nombreModo[modo]);
        } else if (b == 2) {
          Serial.println("Recalibrando...");
          procesoCalibracion();
        }
        break;
      }

    case DIAGNOSTICO:
      {
        leerCrudo();
        int pos = calcularPosicion();

        if (millis() - ultimoPrint >= PERIODO_PRINT_MS) {
          ultimoPrint = millis();
          imprimir(pos);
        }
        if (pos >= 0) ultimaPosicion = pos;  // se actualiza siempre, no solo al imprimir

        if (digitalRead(PIN_BOTON) == LOW) {
          while (digitalRead(PIN_BOTON) == LOW) delay(5);
          delay(100);
          estado = ESPERA_INICIO;
          Serial.println("DETENIDO. BOOT = reiniciar (largo = recalibrar).");
        }
        break;
      }
  }
}