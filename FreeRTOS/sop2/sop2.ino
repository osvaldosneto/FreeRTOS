// Code based on Examples of Arduino and examples 
// from https://github.com/feilipu/Arduino_FreeRTOS_Library
 
#include <Arduino_FreeRTOS.h>
#include <FreeRTOSVariant.h>
#include <task.h>
#include <semphr.h>
 
const byte interruptPin = 2;   // colocar fio no pino 2
 
// 4 tarefas: pisca led, le dados serial, escreve dados na serial, e conta interrupções zero
 
void TaskBlink( void *pvParameters );
void TaskReadFromSerial( void *pvParameters );
void TaskPrintSerial( void *pvParameters );
void TaskINT0( void *pvParameters );
 
SemaphoreHandle_t xSemaphoreSerial = NULL;
SemaphoreHandle_t xSemaphoreINT0 = NULL;
SemaphoreHandle_t semaforoPrint = NULL;
int dadoRecebido = 0; // variável para o dado recebido
int tamanhoFilaTotal = 3;
int tamanho = 0;
QueueHandle_t fila = NULL;



// função de setup
 
void setup() {
 
  Serial.begin(9600);
 
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB, on LEONARDO, MICRO, YUN, and other 32u4 based boards.
  }
 
  //criar dois semáforos binários
  xSemaphoreSerial = xSemaphoreCreateBinary();
  xSemaphoreINT0 = xSemaphoreCreateBinary();
  semaforoPrint = xSemaphoreCreateMutex(); // Semáforo para o objeto print.
  fila = xQueueCreate(tamanhoFilaTotal, sizeof(int)); // Fila com tamanho 3 que armazenará dados inteiros
 
  // criar as 4 tarefas
 
  xTaskCreate(
    TaskBlink
    ,  (const portCHAR *)"Blink"   // A name just for humans
    ,  128  // This stack size can be checked & adjusted by reading the Stack Highwater
    ,  NULL
    ,  2  // Priority, with 3 (configMAX_PRIORITIES - 1) being the highest, and 0 being the lowest.
    ,  NULL );
 
   xTaskCreate(
    TaskReadFromSerial
    ,  (const portCHAR *) "ReadFromSerial"
    ,  128  // Stack size
    ,  NULL
    ,  1  // Priority
    ,  NULL );
 
  /* xTaskCreate(
    TaskPrintSerial
    ,  (const portCHAR *) "PrintSerial"
    ,  128  // Stack size
    ,  NULL
    ,  1  // Priority
    ,  NULL );*/
 
  /* xTaskCreate(
    TaskINT0
    ,  (const portCHAR *) "Task da INT0"
    ,  128  // Stack size
    ,  NULL
    ,  1  // Priority
    ,  NULL ); */
 
  //vPortSetInterruptHandler( mainINTERRUPT_NUMBER, ulExampleInterruptHandler );
  attachInterrupt(digitalPinToInterrupt(interruptPin), ulMinhaInterruptHandler, FALLING);
 
  // escalonador toma conta a partir daqui
 
}
 
void loop()
{
  // nada a fazer aqui
}
 
/*--------------------------------------------------*/
/*---------------------- Tasks ---------------------*/
/*--------------------------------------------------*/
 
void TaskBlink(void *pvParameters)  // This is a task.
{
  (void) pvParameters;
 
/*
  Blink
  Turns on an LED on for one second, then off for one second, repeatedly.
 
  Most Arduinos have an on-board LED you can control. On the UNO, LEONARDO, MEGA, and ZERO 
  it is attached to digital pin 13, on MKR1000 on pin 6. LED_BUILTIN takes care 
  of use the correct LED pin whatever is the board used.
 
  The MICRO does not have a LED_BUILTIN available. For the MICRO board please substitute
  the LED_BUILTIN definition with either LED_BUILTIN_RX or LED_BUILTIN_TX.
  e.g. pinMode(LED_BUILTIN_RX, OUTPUT); etc.
 
  If you want to know what pin the on-board LED is connected to on your Arduino model, check
  the Technical Specs of your board  at https://www.arduino.cc/en/Main/Products
 
  This example code is in the public domain.
 
  modified 8 May 2014
  by Scott Fitzgerald
 
  modified 2 Sep 2016
  by Arturo Guadalupi
*/
 
  // initialize digital LED_BUILTIN on pin 13 as an output.
  pinMode(LED_BUILTIN, OUTPUT);
 
  for (;;) // A Task shall never return or exit.
  {
    digitalWrite(LED_BUILTIN, HIGH);   // turn the LED on (HIGH is the voltage level)
    vTaskDelay( 1000 / portTICK_PERIOD_MS ); // wait for one second
    digitalWrite(LED_BUILTIN, LOW);    // turn the LED off by making the voltage LOW
    vTaskDelay( 1000 / portTICK_PERIOD_MS ); // wait for one second
  }
}
 
 
 
void TaskReadFromSerial(void *pvParameters)  // This is a task.
{
  (void) pvParameters;

  
/*
  AnalogReadSerial
  Reads an analog input on pin 0, prints the result to the serial monitor.
  Graphical representation is available using serial plotter (Tools > Serial Plotter menu)
  Attach the center pin of a potentiometer to pin A0, and the outside pins to +5V and ground.
 
  This example code is in the public domain.
  It was modified by Eraldo S.Silva just to read from serial and to signal in a Binary Semaphore 
 
*/
 
  for (;;)
  {
    int dadoRecebido = 0;
    int tamanho      = 0;

   
    xSemaphoreTake( semaforoPrint, portMAX_DELAY );
    Serial.println("\nEntrar com dados: \n");
    dadoRecebido = Serial.read();
    xSemaphoreGive(semaforoPrint);
   
    while (Serial.available() == 0); // espera ocupada lendo a serial - não é uma boa ideia...
   
   
    tamanho = uxQueueMessagesWaiting(fila);
    if (tamanho < tamanhoFilaTotal)
    {
      // Adiciona dado na fila
      xSemaphoreTake( semaforoPrint, portMAX_DELAY );
      xQueueSendToBack(fila, &dadoRecebido, 0);
      tamanho = uxQueueMessagesWaiting(fila);
      Serial.println("Dado inserido com sucesso!");
      Serial.print("Tamanho atual da fila: ");
      Serial.println(tamanho);
      xSemaphoreGive(semaforoPrint);
     
    }
    else {
      xSemaphoreTake( semaforoPrint, portMAX_DELAY );
      Serial.println("A fila está cheia!");
      xSemaphoreGive(semaforoPrint);
      //Acenderá o LED.
    }
  }  
}
 
void TaskPrintSerial(void *pvParameters)  // Task que imprime na serial - must be improved...
{
  (void) pvParameters;
/*
 
*/
 
  for (;;)
  {
    int valorA = 5;
    int valorB = 6;
    int valorRetirado = 0;
    int tamanho = 0;
    
    xQueueSendToBack(fila, &valorA, 0);
    xQueueSendToBack(fila, &valorB, 0);

    
    xSemaphoreTake( semaforoPrint, portMAX_DELAY );
    //tamanho = uxQueueMessagesWaiting(fila);
    //xQueueReceive(fila, &valorRetirado, portMAX_DELAY);
    //xQueuePeek(fila, &valorRetirado, portMAX_DELAY);
    Serial.println("Valor retirado: ");
   // Serial.println(valorRetirado);
    //tamanho = uxQueueMessagesWaiting(fila);
    //Serial.print("Tamanho atual da fila: ");
    //Serial.println(tamanho);
    xSemaphoreGive(semaforoPrint);
   
    vTaskDelay(1);  // one tick delay (15ms) in between reads for stability - b
  }
}
 
//task to count INT0 occurrences
void TaskINT0(void *pvParameters)  // Task que processa a INT0
{
  (void) pvParameters;
  int contINT0=0;
/*
  created by Eraldo S. e Silva
*/
 
  for (;;)
  {
    xSemaphoreTake( xSemaphoreINT0, portMAX_DELAY );
    Serial.print("Cont INT0 : ");
    Serial.println(contINT0++, DEC);
  }
}
 
// Handler de Interrupção 0 - acorda a tarefa TaskINT0 que espera no semáforo
// Based on FreeRTOS Reference Manual 
 
void ulMinhaInterruptHandler( void )
{
BaseType_t xHigherPriorityTaskWoken;
 
xHigherPriorityTaskWoken = pdFALSE;
 
xSemaphoreGiveFromISR( xSemaphoreINT0, &xHigherPriorityTaskWoken );
 
//portYIELD_FROM_ISR( xHigherPriorityTaskWoken ); parece não ter ...
}
