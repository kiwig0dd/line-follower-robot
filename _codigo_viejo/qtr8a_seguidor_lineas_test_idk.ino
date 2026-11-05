/*
 * QTR-8A + ESP32: telemetría por Bluetooth al celular
 *
 * Comandos (enviarlos desde el celular o el monitor serie):
 *   C = calibrar (5 s, mueve el sensor de lado a lado sobre la línea)
 *   P = pausar / reanudar el envío
 *   M = cambiar formato: tabla legible <-> CSV (para graficar)
 *   H = mostrar ayuda
 *
 * Alimenta el QTR-8A a 3.3 V para no exceder el límite del ADC.
 */

#include "BluetoothSerial.h"

BluetoothSerial SerialBT;

// ---------- Configuración ----------
const uint8_t NUM_SENSORES = 8;
const uint8_t pinesSensores[NUM_SENSORES] = { 34, 35, 32, 33, 25, 26, 27, 14 };

const bool LINEA_NEGRA = true;               // true: línea negra sobre fondo blanco
const unsigned long TIEMPO_CALIBRACION_MS = 5000;
const unsigned long INTERVALO_ENVIO_MS = 50; // 50 ms = 20 envíos por segundo
const int MUESTRAS_ADC = 4;                  // promedio por sensor (reduce ruido)
const int UMBRAL_SENSOR = 500;               // normalizado: > 500 = ve la línea (solo para el dibujo # .)
const int UMBRAL_PERDIDA = 300;              // si ningún sensor supera esto -> línea perdida
const int RUIDO_MIN = 100;                   // valores por debajo se ignoran en el cálculo
const int RANGO_MINIMO = 300;                // rango de calibración mínimo para confiar en un sensor
const int CENTRO = 3500;

// ---------- Estado ----------
int minimo[NUM_SENSORES];
int maximo[NUM_SENSORES];
int crudo[NUM_SENSORES];
int norm[NUM_SENSORES];

int ultimaPosicion = CENTRO;
bool calibrado = false;
bool enviando = true;
bool modoCSV = false;
unsigned long ultimoEnvio = 0;

// ---------- Salida a monitor serie y Bluetooth ----------
void enviar(const String &texto) {
  Serial.print(texto);
  if (SerialBT.hasClient()) SerialBT.print(texto);
}

void mostrarAyuda() {
  enviar("\n=== QTR-8A Telemetria ===\n"
         "C = calibrar (5 s)\n"
         "P = pausar/reanudar\n"
         "M = cambiar formato (tabla/CSV)\n"
         "H = ayuda\n\n");
}

// ---------- Sensores ----------
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

  enviar("\n>>> CALIBRANDO: mueve el sensor de lado a lado sobre la linea <<<\n");

  unsigned long inicio = millis();
  int ultimoSegundo = -1;

  while (millis() - inicio < TIEMPO_CALIBRACION_MS) {
    leerCrudo();
    for (uint8_t i = 0; i < NUM_SENSORES; i++) {
      if (crudo[i] < minimo[i]) minimo[i] = crudo[i];
      if (crudo[i] > maximo[i]) maximo[i] = crudo[i];
    }

    int restante = (TIEMPO_CALIBRACION_MS - (millis() - inicio)) / 1000 + 1;
    if (restante != ultimoSegundo) {
      ultimoSegundo = restante;
      enviar("  " + String(restante) + " s...\n");
    }
    delay(200);
  }

  String resumen = ">>> Calibracion terminada <<<\nSensor\tMin\tMax\n";
  for (uint8_t i = 0; i < NUM_SENSORES; i++) {
    resumen += "S" + String(i + 1) + "\t" + String(minimo[i]) + "\t" + String(maximo[i]);
    if (maximo[i] - minimo[i] < 300) resumen += "\t<-- rango pequeno, revisa este sensor";
    resumen += "\n";
  }
  resumen += "\n";
  enviar(resumen);

  calibrado = true;
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

// ---------- Envío de datos ----------
void enviarDatos() {
  int pos = calcularPosicion();
  int estado = 0;  // 0 = OK, 1 = perdida por izquierda, 2 = perdida por derecha

  if (pos < 0) {
    estado = (ultimaPosicion < CENTRO) ? 1 : 2;
  } else {
    ultimaPosicion = pos;
  }

  String linea;

  if (modoCSV) {
    // S1..S8,posicion,error,estado
    for (uint8_t i = 0; i < NUM_SENSORES; i++) {
      linea += String(norm[i]);
      linea += ',';
    }
    linea += String(ultimaPosicion);
    linea += ',';
    linea += String(ultimaPosicion - CENTRO);
    linea += ',';
    linea += String(estado);
    linea += '\n';
  } else {
    for (uint8_t i = 0; i < NUM_SENSORES; i++) {
      linea += String(norm[i]);
      linea += '\t';
    }
    linea += "| ";
    if (estado == 0) {
      linea += String(pos) + "\t" + String(pos - CENTRO);
    } else {
      linea += "---\t---";
    }
    linea += "\t| ";
    for (uint8_t i = 0; i < NUM_SENSORES; i++) {
      linea += (norm[i] > UMBRAL_SENSOR) ? '#' : '.';
    }
    if (estado == 1) linea += "  PERDIDA IZQ (ult: " + String(ultimaPosicion) + ")";
    if (estado == 2) linea += "  PERDIDA DER (ult: " + String(ultimaPosicion) + ")";
    linea += '\n';
  }

  enviar(linea);
}

// ---------- Comandos ----------
void procesarComando(char c) {
  switch (toupper(c)) {
    case 'C':
      calibrar();
      if (!modoCSV) enviar("S1\tS2\tS3\tS4\tS5\tS6\tS7\tS8\t| Pos\tError\t| Visual\n");
      break;
    case 'P':
      enviando = !enviando;
      enviar(enviando ? "Envio REANUDADO\n" : "Envio PAUSADO\n");
      break;
    case 'M':
      modoCSV = !modoCSV;
      enviar(modoCSV ? "Formato: CSV (S1..S8,pos,error,estado)\n"
                     : "Formato: tabla\n");
      break;
    case 'H':
      mostrarAyuda();
      break;
    default:
      break;  // ignora \n, \r y otros
  }
}

void leerComandos() {
  while (SerialBT.available()) procesarComando(SerialBT.read());
  while (Serial.available()) procesarComando(Serial.read());
}

// ---------- Programa principal ----------
void setup() {
  Serial.begin(115200);
  SerialBT.begin("ESP32_QTR");

  analogReadResolution(12);
  analogSetAttenuation(ADC_11db);
  for (uint8_t i = 0; i < NUM_SENSORES; i++) pinMode(pinesSensores[i], INPUT);

  // Sin calibrar: usa el rango completo del ADC hasta que mandes 'C'
  for (uint8_t i = 0; i < NUM_SENSORES; i++) {
    minimo[i] = 0;
    maximo[i] = 4095;
  }

  Serial.println("Bluetooth listo: busca 'ESP32_QTR' en tu celular");
  mostrarAyuda();
}

void loop() {
  leerComandos();

  if (enviando && millis() - ultimoEnvio >= INTERVALO_ENVIO_MS) {
    ultimoEnvio = millis();
    leerCrudo();
    enviarDatos();
  }
}