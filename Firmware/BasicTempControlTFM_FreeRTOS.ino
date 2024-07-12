/*
   AutoPID BasicTempControl Example Sketch

   This program reads a dallas temperature probe as input, potentiometer as setpoint, drives an analog output.
   It lights an LED when the temperature has reached the setpoint.
*/

/*
This is a modified example for SHT3X-DIS Digital Humidity & Temperature Sensors Arduino Library
ClosedCube SHT31-D [Digital I2C] Humidity and Temperature Sensor Breakout

Initial Date: 06-Oct-2015
Last Updated: 13-Mar-2024

Hardware connections for Arduino Uno:
VDD to 3.3V DC
SDA to A4
SCL to A5
GND to common ground */

//included libraries
//Arduino main library
#include <Arduino.h>

//PID Control library
#include <AutoPID.h>

//I2C and peripheral sensor libraries
#include <OneWire.h>
#include <Wire.h>
#include <ArtronShop_SHT3x.h>

//FreeRTOS libraries
#include <FreeRTOS_SAMD21.h> //SAMD21 FreeRTOS library
#include <semphr.h> //semaphore & mutex library

//pins' definitions
#define SET_PIN A0
#define OUTPUT_PIN A9
#define OUTPUT_PIN2 A10
#define LED_PIN 6
#define OPTICAL_SWITCH_PIN 8

#define TEMP_READ_DELAY 750 //can only read digital temp sensor every ~750ms

//pid settings and gains
#define OUTPUT_MIN 0
#define OUTPUT_MAX 255
#define KP 0.5
#define KI 0.05
#define KD 0.75

//structure definition
//this type-defined structure works globally 
typedef struct sht3x_variables_t {
    double humidity;
    double temperature1;
    double temperature2;
    double PIDTemp;
    double setpoint;
    double output;
    double error_accum;
} sht3x_variables_t;

//PID control variables
//AutoPID class makes mandatory to use double data type for all its global variables
double PIDtemperatureRead, setPoint, outputTemp, error, error_acum;
volatile unsigned long lastTempUpdate; //tracks the last time a PIDtemperatureRead sample was taken

//variable to test assertions
const boolean valueToAssert = true; //use it when the baseline is finished

//AtronShop_SHT3x class: class for SHT3x I2C temperature sensor
ArtronShop_SHT3x sht3x1(0x44, &Wire), sht3x2(0x45, &Wire); // ADDR: 0 => 0x44, ADDR: 1 => 0x45

//AutoPID object class: input/output variables passed by reference, so they are updated automatically
AutoPID Temp_Peltier_PID(&PIDtemperatureRead, &setPoint, &outputTemp, OUTPUT_MIN, OUTPUT_MAX, KP, KI, KD);

//FreeRTOS Handlers
SemaphoreHandle_t xSerialSemaphore; //Semaphores to control the access and update of the temperature and humidity variables; //Semaphore to control the access and update of the temperature variable

//Global variables for task handlers
TaskHandle_t xTask_sht3x1_handle, xTask_PID_temp_update_handle, xTask_optical_switch_handle;

//Handler as a global variable to the Queue that holds the temperature and humidity variables
QueueHandle_t xQueue_SHT31x_queue;

//Function prototypes both for FreeRTOS task definitions and for external function variables
void sht3x1_task(void *pvParameters);
void PID_temp_update_task(void *pvParameters);
void optical_switch_task(void *pvParameters);

//Function prototypes for global functions
bool updateTemperature(bool);
void myDelayUs(int);

void myDelayMs(int);
void myDelayMsUntil(TickType_t, int);


  // Assert value is true, execution doesn't stop.
  //configASSERT(valueToAssert == true);

  // Assert value is false, FreeRTOS execution stops and start to blink main led two times with 4 second cycle.
  //configASSERT(valueToAssert == false);

void setup() {

  pinMode(SET_PIN, INPUT);
  pinMode(OUTPUT_PIN, OUTPUT);
  pinMode(LED_PIN, OUTPUT);
  pinMode(OPTICAL_SWITCH_PIN, OUTPUT);

   //PID Control Global Functions
   //if temperature is more than 5 degrees below or above setpoint, OUTPUT will be set to min or max respectively
   //Temp_Peltier_PID.setBangBang(5);
   //set PID update interval to 750ms
   Temp_Peltier_PID.setTimeStep(750);

  Serial.begin(115200);
  Serial.println("**********Inicialiando interfaz Serial...*************");

  while(!Serial){
    Serial.println("Puerto Serial no disponible... Reintentando...");
    delay(500);
  }

  /*Structured queue creation. The protoype is like this:
   *
   * xQueue_SHT31x_queue = xQueueCreate(Queue_Length, sizeof(struct type_of_data_t));
   */

  //Queue service initialization
    if ( xQueue_SHT31x_queue == NULL )
    {
     xQueue_SHT31x_queue = xQueueCreate(25, sizeof(sht3x_variables_t));
     if ( ( xQueue_SHT31x_queue ) != NULL ){
      Serial.println("**********Servicio de gestion de Cola creado correctamente*************");
      delay(100);
      }
      else
      {
      Serial.println("La creacion de la cola ha fallado. Reintentando...");
      delay(100);
      while(1);
      }
    }

  //serial semaphore service initialization
  if ( xSerialSemaphore == NULL )
    {
    xSerialSemaphore = xSemaphoreCreateMutex(); 
    if ( ( xSerialSemaphore ) != NULL ){
      xSemaphoreGive( ( xSerialSemaphore ) );
      Serial.println("Semaforo para la interfaz Serial creado correctamente");
      delay(100);
      }
      else
      {
      Serial.println("Creacion del semaforo serial fallida. Reintentando...");
      delay(100);
      while(1);
      }
    }

      /***************************************INITIAL SETUP MENU********************************************/
      Serial.println("\n***********AJUSTE DEL PUNTO DE CONSIGNA INICIAL**************");
      Serial.println("\nPor favor teclee el punto de consigna inicial de temperatura en ºC seguido de [enter]:");
      while(Serial.available() == 0){}
      setPoint = Serial.parseFloat();
      Serial.println("El valor del punto de consigna inicial de temperatura insertado es:\t");
      Serial.print(setPoint);
      Serial.print("\tºC\n");
      Serial.println("***********FIN AJUSTE DEL PUNTO DE CONSIGNA INICIAL**************");
      /***************************************INITIAL SETUP MENU********************************************/ 

    //FreeRTOS task creation definitions
    //Task function, "name",,parameter,priority,handle
    xTaskCreate(PID_temp_update_task, "PID_temp_update_task", 512, NULL, tskIDLE_PRIORITY + 1, &xTask_PID_temp_update_handle);
    xTaskCreate(sht3x1_task, "sht3x1_task", 512, NULL, tskIDLE_PRIORITY + 1, &xTask_sht3x1_handle); 
    xTaskCreate(optical_switch_task, "optical_switch_task", 512, NULL, tskIDLE_PRIORITY + 1, &xTask_optical_switch_handle);
  
    //Starts the FreeRTOS kernel scheduler 
    vTaskStartScheduler();

  }//void setup()


  void PID_temp_update_task(void *pvParameters __attribute__((unused))) {

   pinMode(LED_BUILTIN, OUTPUT);


   for(;;){

          sht3x_variables_t sensor_read;
          sensor_read.temperature1 = sht3x1.temperature();
          sensor_read.temperature2 = sht3x2.temperature();
          sensor_read.setpoint = setPoint;
          sensor_read.error_accum = error_acum;
          sensor_read.output = outputTemp;
          sensor_read.PIDTemp = PIDtemperatureRead;

          Serial.flush();
          if ( xSemaphoreTake( xSerialSemaphore, ( TickType_t ) (1500 / portTICK_PERIOD_MS) ) == pdTRUE )
          {
            if (xQueueReceive(xQueue_SHT31x_queue, &sensor_read, portMAX_DELAY) == pdPASS)
            {
             /***************************************PID CONTROLLER DATA********************************************/ 
             Serial.println("\n***********RECIBIENDO MUESTRA DE DATOS DE LA COLA**************");
             Serial.println("***********INICIO PID**************");
             Serial.println("El valor de temperatura del sensor # 1 tomado por el PID es: ");
             Serial.println(sensor_read.temperature1);
             Serial.println("El valor de temperatura del sensor # 2 tomado por el PID es: ");
             Serial.println(sensor_read.temperature2);
             Serial.println("El valor de salida del PID es: ");
             Serial.println(sensor_read.output);
             Serial.println("El valor del punto de consigna es:\t");
             Serial.print(sensor_read.setpoint);
             Serial.print("\tºC\n");
             Serial.println("El error absoluto de temperatura es:");
             Serial.print(error);
             Serial.print("\tºC\n");
             Serial.println("El error acum. del PID en relacion a la temperatura es:");
             Serial.print(sensor_read.error_accum);
             Serial.print("\tºC\n");
             Serial.println("***********FIN PID**************");
             myDelayMs(1000);
             /***************************************PID CONTROLLER DATA********************************************/

            /****************************************PID-SUBROUTINE********************************************/
             Temp_Peltier_PID.run(); //call the PID controller sub-routine, updates automatically at certain time interval
             sensor_read.PIDTemp = sensor_read.temperature1;
             Serial.println("\nLa muestra de temperatura tomada por el controlador PID, es:");
             Serial.print(sensor_read.PIDTemp);
             if(sensor_read.PIDTemp > sensor_read.setpoint){
              analogWrite(OUTPUT_PIN2, sensor_read.output);
              analogWrite(OUTPUT_PIN, HIGH);
             }
             else if(sensor_read.PIDTemp <= sensor_read.setpoint){
              analogWrite(OUTPUT_PIN, sensor_read.output);
              analogWrite(OUTPUT_PIN2, HIGH);
             }
             digitalWrite(LED_PIN, Temp_Peltier_PID.atSetPoint(3)); //light up LED when we're at setpoint +-10 degrees
             /****************************************PID-SUBROUTINE********************************************/ 
 
          //error = sht3x1.temperature() - sht3x2.temperature();
           }
          xSemaphoreGive( ( xSerialSemaphore ) );
          }
      }
  }//void PID_task()

  void sht3x1_task(void *pvParameters __attribute__((unused))) {

    sht3x1.begin();
    Serial.println("SHT3x # 1 comenzando...");
  
    sht3x2.begin();
    Serial.println("SHT3x # 2 comenzando...");

    Wire.begin();
    while (!sht3x1.begin()) {
    Serial.println("¡ SHT3x # 1 no encontrado !");
    myDelayMs(250);
    }

    while (!sht3x2.begin()) {
    Serial.println("¡ SHT3x # 2 no encontrado !");
    myDelayMs(250); //vTaskDelay((250 * 1000) / portTICK_PERIOD_US);
    }

    for(;;){

          error_acum += (KP*error)+(KI*error)-(KD*error);
          
          sht3x_variables_t sensor_read;
          sensor_read.temperature1 = sht3x1.temperature();
          sensor_read.temperature2 = sht3x2.temperature();
          sensor_read.humidity = sht3x1.humidity();
          sensor_read.output = outputTemp;
          sensor_read.setpoint = setPoint;
          sensor_read.PIDTemp = PIDtemperatureRead;
          sensor_read.error_accum = error_acum;
          
          Serial.flush();
          //setPoint = analogRead(SET_PIN);
          Serial.println("\n\n***********INICIO MEDIDA**************");
          if (sht3x1.measure()) {
          Serial.print("Temperatura del Sensor # 1: ");
          Serial.print(sensor_read.temperature1, 1);
          Serial.print("ºC \tHumedad del Sensor # 1: ");
          Serial.print(sensor_read.humidity, 1);
          Serial.print(" %RH");
          Serial.println();
          Serial.println("\n\nEl paso de la medicion se cumple[1=Si/0=No]:\t");
          Serial.print(updateTemperature());
          } else {
            Serial.println("\nError de lectura del SHT3x1");
          }
          myDelayMs(500);
          if (sht3x2.measure()) {
          Serial.println("Enviando muestra de sensor 2 por la cola\n");
          Serial.print("Temperatura del Sensor # 2: ");
          Serial.print(sensor_read.temperature2, 2);
          Serial.print("ºC \tHumedad del Sensor # 2: ");
          Serial.print(sensor_read.humidity, 1);
          Serial.print(" %RH");
          Serial.println();
          Serial.println("\n\nEl paso de la medicion se cumple[1=Si/0=No]:\t");
          Serial.print(updateTemperature());
          } else {
            Serial.println("\nError de lectura del SHT3x2");
          }
          myDelayMs(500);
          Serial.println("\n***********FIN MEDIDA**************");
          Serial.println("***********ENVIANDO MUESTRA DE DATOS A LA COLA**************");
          xQueueSend(xQueue_SHT31x_queue, &sensor_read, portMAX_DELAY);
      }
  }//void sht3x1_task()>

  void optical_switch_task(void *pvParameters __attribute__((unused))) {

      pinMode(LED_BUILTIN, OUTPUT);

      for(;;){

          Serial.println("\n\n***********INICIO SWTICH**************");
          digitalWrite(OPTICAL_SWITCH_PIN, LOW);
          myDelayMs(15000);
          Serial.println("SWITCH ON");
          digitalWrite(OPTICAL_SWITCH_PIN, HIGH);
          myDelayMs(15000);
          Serial.println("SWITCH OFF");
          Serial.println("***********FIN SWTICH**************");
      }
  }//void optical_switch_task()

void loop()
  {
  //Serial.println("SCHEDULER VIVO Y LOOP ALCANZADO ESO ES UN MILAGRO");

  }//Try to use this void loop() for a specific routine.


bool updateTemperature() {
  sht3x_variables_t sensor_read;
  if ((millis() - lastTempUpdate) > TEMP_READ_DELAY) {
    sensor_read.temperature1 = sht3x1.temperature(); //get temp reading
    lastTempUpdate = millis();
    return true;
  }
  return false;
}//void updateTemperature

void myDelayUs(int us)
{
  vTaskDelay( us / portTICK_PERIOD_US );  
}

void myDelayMs(int ms)
{
  vTaskDelay( (ms * 1000) / portTICK_PERIOD_US );  
}//void myDelayMs

void myDelayMsUntil(TickType_t *previousWakeTime, int ms)
{
  vTaskDelayUntil( previousWakeTime, (ms * 1000) / portTICK_PERIOD_US );  
}//void myDelayMsUntil