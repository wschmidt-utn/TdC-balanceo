#include "Simple_MPU6050.h"					// incluye libreria Simple_MPU6050
#define MPU6050_ADDRESS_AD0_LOW     0x68			// direccion I2C con AD0 en LOW o sin conexion
#define MPU6050_ADDRESS_AD0_HIGH    0x69			// direccion I2C con AD0 en HIGH
#define MPU6050_DEFAULT_ADDRESS     MPU6050_ADDRESS_AD0_LOW	// por defecto AD0 en LOW

Simple_MPU6050 mpu;				// Instancia libreria mpu
int contador = 0;
float entrada = 0.0;
float A = 0.0;

//              X Accel  Y Accel  Z Accel   X Gyro   Y Gyro   Z Gyro
#define OFFSETS   -272,   -1068,    1218,     166,      51,       8

#define MPU6050_DEFAULT_ADDRESS     MPU6050_ADDRESS_AD0_LOW	// por defecto AD0 en LOW

#define PIN_MOTORA_ARRIBA 8
#define PIN_MOTORA_ABAJO 9
//#define PIN_MOTORB_ARRIBA 10
//#define PIN_MOTORB_ABAJO 11

// mostrar_valores funcion que es llamada cada vez que hay datos disponibles desde el sensor
void on_sensor_changes (int16_t *gyro, int16_t *accel, int32_t *quat, uint32_t *timestamp) {	
  Quaternion q;					// variable necesaria para calculos posteriores
  VectorFloat gravity;				// variable necesaria para calculos posteriores
  float ypr[3] = { 0, 0, 0 };			// array para almacenar valores de yaw, pitch, roll
  float xyz[3] = { 0, 0, 0 };			// array para almacenar valores convertidos a grados de yaw, pitch, roll
  mpu.GetQuaternion(&q, quat);		// funcion para obtener valor para calculo posterior
  mpu.GetGravity(&gravity, &q);		// funcion para obtener valor para calculo posterior
  mpu.GetYawPitchRoll(ypr, &q, &gravity);	// funcion obtiene valores de yaw, ptich, roll
  mpu.ConvertToDegrees(ypr, xyz);		// funcion convierte a grados sexagesimales
  if (contador == 50){
    Serial.print(xyz[2]);  // muestra en monitor serie rotacion de eje X, roll
    Serial.println();				// salto de linea
    contador = 0;
    //handle_motors(xyz[2], true);  
  }
  contador++;
  handle_motors(xyz[2], false);
  
}

void handle_motors(float roll, bool debug){
  float error = entrada - roll; // el error minimo es -65 y el máximo 65;
  float G = 2;
  //Si error = 0 => ambos deben ser 0.
  //Si error > 0 => encender motor A. 
  //Si error < 0 => encender motor B. 

  int angulo_cero = -roll;

  int velocidad_cero = G * angulo_cero;

  int velocidad_objetivo = A * error;

  int velocidad = 0;
  int velocidad_minima = 10;
  int velocidad_maxima = 255;
  velocidad = velocidad_objetivo + velocidad_cero;
  int velocidad_derecha = 0;
  int velocidad_izquierda = 0;
  if (velocidad < -1) {

    velocidad_izquierda = min(velocidad_maxima, max(0,20 + velocidad_minima - velocidad));

    analogWrite(PIN_MOTORA_ARRIBA, 255);
    //analogWrite(PIN_MOTORB_ABAJO, 255);

    //analogWrite(PIN_MOTORB_ARRIBA, 255 - velocidad_izquierda);
    analogWrite(PIN_MOTORA_ABAJO, 255 - velocidad_izquierda);

  } else if (velocidad > 1) {
s
    velocidad_derecha = min(velocidad_maxima, max(0,velocidad_minima + velocidad));

    //analogWrite(PIN_MOTORB_ARRIBA, 255);
    analogWrite(PIN_MOTORA_ABAJO, 255);

    analogWrite(PIN_MOTORA_ARRIBA, 255 - velocidad_derecha);
    //analogWrite(PIN_MOTORB_ABAJO, 255 - velocidad_derecha);

  } else {    
    analogWrite(PIN_MOTORA_ARRIBA, 255);
    //analogWrite(PIN_MOTORB_ABAJO, 255);

    //analogWrite(PIN_MOTORB_ARRIBA, 255);
    analogWrite(PIN_MOTORA_ABAJO, 255);
  }

  if (debug){
    Serial.print("Error (Angulo respecto objetivo): ");
    Serial.print(error);
    Serial.println();

    Serial.print("Angulo respecto al Cero: ");
    Serial.print(angulo_cero);
    Serial.println();

    Serial.print("Velocidad hacia el Cero: ");
    Serial.print(velocidad_cero);
    Serial.println();

    Serial.print("Velocidad Objetivo: ");
    Serial.print(velocidad_objetivo);
    Serial.println();

    if (velocidad_izquierda > 0){
      Serial.print("Velocidad Izquierda: ");
      Serial.print(velocidad_izquierda);
      Serial.println();
    } else if (velocidad_derecha > 0){
      Serial.print("Velocidad Derecha: ");
      Serial.print(velocidad_derecha);
      Serial.println();
    } else {
      Serial.print("Equilibrio");
      Serial.println();
    }

  }
  
}

void setup() {
  uint8_t val;
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE	// activacion de bus I2C a 400 Khz
  Wire.begin();
  Wire.setClock(400000);
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
  Fastwire::setup(400, true);
#endif
  
  Serial.begin(19200);			// inicializacion de monitor serie a 19200 bps
  pinMode(PIN_MOTORA_ARRIBA,OUTPUT); 
  analogWrite(PIN_MOTORA_ARRIBA, 255);
  //pinMode(PIN_MOTORB_ARRIBA,OUTPUT);
  //analogWrite(PIN_MOTORB_ARRIBA, 255);
  pinMode(PIN_MOTORA_ABAJO,OUTPUT); 
  analogWrite(PIN_MOTORA_ABAJO, 255);
  //pinMode(PIN_MOTORB_ABAJO,OUTPUT);
  //analogWrite(PIN_MOTORA_ABAJO, 255);
  while (!Serial); 			// espera a enumeracion en caso de modelos con USB nativo
  Serial.println(F("Inicio:"));		// muestra texto estatico
#ifdef OFFSETS								// si existen OFFSETS
  Serial.println(F("Usando Offsets predefinidos"));			// texto estatico
  mpu.SetAddress(MPU6050_ADDRESS_AD0_LOW).load_DMP_Image(OFFSETS);	// inicializacion de sensor

#else										// sin no existen OFFSETS
  Serial.println(F(" No se establecieron Offsets, haremos unos nuevos.\n"
                   " Colocar el sensor en un superficie plana y esperar unos segundos\n"
                   " Colocar los nuevos Offsets en #define OFFSETS\n"
                   " para saltar la calibracion inicial \n"
                   " \t\tPresionar cualquier tecla y ENTER"));
  while (Serial.available() && Serial.read());		// lectura de monitor serie
  while (!Serial.available());   			// si no hay espera              
  while (Serial.available() && Serial.read()); 		// lecyura de monitor serie
  mpu.SetAddress(MPU6050_ADDRESS_AD0_LOW).CalibrateMPU().load_DMP_Image();	// inicializacion de sensor
#endif
  mpu.on_FIFO(on_sensor_changes);		// llamado a funcion on_sensor_changes si memoria FIFO tiene valores
  Serial.println();
  entrada = obtenerFloatDeConsola("entrada", -30.0, 30.0); 
  A = obtenerFloatDeConsola("constante", 0.1, 5.0);
}

float obtenerFloatDeConsola(char* nombre_de_la_variable, float valor_minimo, float valor_maximo){
  float retorno = -100.0;
  while (retorno == -100.0){
    Serial.print("Ingrese el valor de ");
    Serial.print(nombre_de_la_variable);
    Serial.print(" (entre ");
    Serial.print(valor_minimo);
    Serial.print(" y ");
    Serial.print(valor_maximo);
    Serial.println("): ");
    while (Serial.available() && Serial.read());		// lectura de monitor serie
    while (!Serial.available());   			// si no hay espera      
    float lectura = Serial.parseFloat();
    
    Serial.println(lectura);
    if (lectura < valor_minimo || lectura > valor_maximo){
      Serial.println("Valor incorrecto");
    } else {
      retorno = lectura;
    }   
    
  }
  
  return retorno;
}

void loop() {
  mpu.dmp_read_fifo();		// funcion que evalua si existen datos nuevos en el sensor y llama
}				// a funcion mostrar_valores si es el caso
