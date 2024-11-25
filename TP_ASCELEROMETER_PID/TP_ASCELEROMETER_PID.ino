#include "Simple_MPU6050.h"					// incluye libreria Simple_MPU6050
#include <PID_v1.h>					// incluye libreria de PID
#define MPU6050_ADDRESS_AD0_LOW     0x68			// direccion I2C con AD0 en LOW o sin conexion
#define MPU6050_ADDRESS_AD0_HIGH    0x69			// direccion I2C con AD0 en HIGH
#define MPU6050_DEFAULT_ADDRESS     MPU6050_ADDRESS_AD0_LOW	// por defecto AD0 en LOW

Simple_MPU6050 mpu;				// Instancia libreria mpu
int contador = 0;
float entrada = 5.0;
float velocidad_minima = 48.0;

double consKp=10.5, consKi=0.05, consKd=0.1;
double Setpoint, Input, Output;
PID myPID(&Input, &Output, &Setpoint, consKp, consKi, consKd, DIRECT);

//              X Accel  Y Accel  Z Accel   X Gyro   Y Gyro   Z Gyro
#define OFFSETS   -332,   -1118,    1196,     166,      53,       5

#define MPU6050_DEFAULT_ADDRESS     MPU6050_ADDRESS_AD0_LOW	// por defecto AD0 en LOW

#define PIN_MOTORA_ARRIBA 8
#define PIN_MOTORA_ABAJO 9
#define PIN_MOTORB_ARRIBA 10
#define PIN_MOTORB_ABAJO 11

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
  if (contador == 10){
    Serial.print(xyz[2]);  // muestra en monitor serie rotacion de eje X, roll
    Serial.println();				// salto de linea
    contador = 0;
    handle_motors(xyz[2], true);  
  }
  contador++;
  //handle_motors(xyz[2], false);
  
}

void handle_motors(float roll, bool debug){
  Input = roll;
  double gap = abs(Setpoint-Input);
  myPID.Compute();

  float tolerancia = 3;
  float velocidad_izquierda = 0;
  float velocidad_derecha = 0;
  float velocidad_maxima = 255;

  if (Output < -tolerancia) {

    velocidad_izquierda = min(velocidad_maxima, max(0, velocidad_minima - Output));

    analogWrite(PIN_MOTORA_ARRIBA, 255);
    analogWrite(PIN_MOTORB_ABAJO, 255);

    analogWrite(PIN_MOTORB_ARRIBA, 255 - velocidad_izquierda);
    analogWrite(PIN_MOTORA_ABAJO, 255 - velocidad_izquierda);

  } else if (Output > tolerancia) {

    velocidad_derecha = min(velocidad_maxima, max(0,velocidad_minima + 20 + Output));

    analogWrite(PIN_MOTORB_ARRIBA, 255);
    analogWrite(PIN_MOTORA_ABAJO, 255);

    analogWrite(PIN_MOTORA_ARRIBA, 255 - velocidad_derecha);
    analogWrite(PIN_MOTORB_ABAJO, 255 - velocidad_derecha);

  } else {    
    analogWrite(PIN_MOTORA_ARRIBA, 255);
    analogWrite(PIN_MOTORB_ABAJO, 255-velocidad_minima);

    analogWrite(PIN_MOTORB_ARRIBA, 255);
    analogWrite(PIN_MOTORA_ABAJO, 255-velocidad_minima);
  }

  if (debug){

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
  Wire.setWireTimeout(3000, true); //timeout value in uSec
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
  Fastwire::setup(400, true);
#endif
  
  Serial.begin(115200);			// inicializacion de monitor serie a 19200 bps
  pinMode(PIN_MOTORA_ARRIBA,OUTPUT); 
  analogWrite(PIN_MOTORA_ARRIBA, 255);
  pinMode(PIN_MOTORB_ARRIBA,OUTPUT);
  analogWrite(PIN_MOTORB_ARRIBA, 255);
  pinMode(PIN_MOTORA_ABAJO,OUTPUT); 
  analogWrite(PIN_MOTORA_ABAJO, 255);
  pinMode(PIN_MOTORB_ABAJO,OUTPUT);
  analogWrite(PIN_MOTORB_ABAJO, 255);
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
  Setpoint = entrada;
  myPID.SetMode(AUTOMATIC);
  myPID.SetOutputLimits(-255, 255);
  myPID.SetSampleTime(1);
  Serial.println();
  //entrada = obtenerFloatDeConsola("entrada", -30.0, 30.0); 
  //A = obtenerFloatDeConsola("constante", 0, 5.0);
  mpu.on_FIFO(on_sensor_changes);		// llamado a funcion on_sensor_changes si memoria FIFO tiene valores

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
    retorno = RecibirFloatPorSerial(valor_minimo, valor_maximo);
    
  }
  
  return retorno;
}

void loop() {
  mpu.dmp_read_fifo();
  RevisarCambiosConfig();
}

void RevisarCambiosConfig(){
  if(Serial.available() > 0){
        char receivedByte = Serial.read();
        if (receivedByte == 'P'){
          consKp = RecibirFloatPorSerial(0.0, 20.0);
          Serial.print("Nuevo valor de constante proporcional: ");
          Serial.println(consKp);
          myPID.SetTunings(consKp, consKi, consKd);
        } else if (receivedByte == 'M'){
          velocidad_minima = RecibirFloatPorSerial(0.0, 100.0);
          Serial.print("Nuevo valor de velocidad minima: ");
          Serial.println(velocidad_minima);
        } else if (receivedByte == 'D'){
          consKd = RecibirFloatPorSerial(0.0, 100.0);
          Serial.print("Nuevo valor contante derivativa: ");
          Serial.println(consKd);
          myPID.SetTunings(consKp, consKi, consKd);
        } else if (receivedByte == 'I'){
          consKi = RecibirFloatPorSerial(0.0, 100.0);
          Serial.print("Nuevo valor constante integrativa: ");
          Serial.println(consKi);
          myPID.SetTunings(consKp, consKi, consKd);
        } else if (receivedByte == 'E'){
          entrada = RecibirFloatPorSerial(0.0, 100.0);
          Serial.print("Nuevo valor Entrada: ");
          Serial.println(entrada);
          Setpoint = entrada;
        } else {
          Serial.println("Valor incorrecto");
        }
  }
}

float RecibirFloatPorSerial(float valor_minimo, float valor_maximo){
  float lectura = Serial.parseFloat();
  if (lectura < valor_minimo || lectura > valor_maximo){
    Serial.println("Valor incorrecto");
  }
  return lectura;
}