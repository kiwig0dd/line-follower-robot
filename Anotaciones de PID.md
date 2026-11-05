
**VALORES DE PRIMERAS 3 VUELTAS**
// ----- PID (ajústalo en pista) -----

float KP = 0.38;  

float KI = 0.0;

float KD = 0.48;  

const float INTEGRAL_MAX = 3000;             // anti-windup

  

// ----- Velocidades (0-255) -----

const int VEL_BASE    = 185;

const int VEL_MAX     = 225;

const int VEL_PERDIDA = 180;

**VALORES BUENESITOS**
// ====================== CONFIGURACIÓN SENSORES ======================

const bool LINEA_NEGRA = true;               // true: línea negra sobre fondo blanco

const unsigned long TIEMPO_CALIBRACION_MS = 5000;

const int MUESTRAS_ADC = 4;                  // promedio por sensor (reduce ruido)

const int UMBRAL_PERDIDA = 300;              // si ningún sensor supera esto -> línea perdida

const int RUIDO_MIN = 100;                   // valores por debajo se ignoran en el cálculo

const int RANGO_MINIMO = 300;                // rango de calibración mínimo para confiar en un sensor

const int CENTRO = 3500;

  

// Si un motor gira al revés, cambia su valor a true

const bool INVERTIR_MOTOR_IZQ = false;

const bool INVERTIR_MOTOR_DER = true;

  

// ----- PID (ajústalo en pista) -----

float KP = 0.03;

float KI = 0.0;

float KD = 0.40;

const float INTEGRAL_MAX = 3000;             // anti-windup

  

// ----- Velocidades (0-255) -----

const int VEL_BASE    = 200;

const int VEL_MAX     = 220;

const int VEL_PERDIDA = 190;


valores cawai 
500 de umbral de perdida
float KP = 0.4;  

float KI = 0.0;

float KD = 0.76;


**valorcitos los mas buenecitos cafetin**

const int UMBRAL_PERDIDA = 500;              // si ningún sensor supera esto -> línea perdida

const int RUIDO_MIN = 150;                   // valores por debajo se ignoran en el cálculo

const int RANGO_MINIMO = 300;
float KP = 0.30;  

float KI = 0.0;

float KD = 1.00;
