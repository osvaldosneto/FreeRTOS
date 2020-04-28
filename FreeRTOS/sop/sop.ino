// Code based on Examples of Arduino and examples 
// from https://github.com/feilipu/Arduino_FreeRTOS_Library
 
#include <Arduino_FreeRTOS.h>
#include <FreeRTOSVariant.h>
#include <task.h>
#include <semphr.h>
 
const byte interruptPin = 2;   // colocar fio no pino 2
 
// 4 tarefas: pisca led, le dados serial, escreve dados na serial, e conta interrupções zero
void TaskReadFromSerial( void *pvParameters );
void TaskPrintSerial( void *pvParameters );

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
 
  //criar um semáforos binários
  semaforoPrint = xSemaphoreCreateMutex(); // Semáforo para o objeto print.
  fila = xQueueCreate(tamanhoFilaTotal, sizeof(int)); // Fila com tamanho 3 que armazenará dados inteiros
 
  // criar as 4 tarefas
   xTaskCreate(
    TaskReadFromSerial
    ,  (const portCHAR *) "ReadFromSerial"
    ,  128  // Stack size
    ,  NULL
    ,  2  // Priority
    ,  NULL );
 
   xTaskCreate(
    TaskPrintSerial
    ,  (const portCHAR *) "PrintSerial"
    ,  128  // Stack size
    ,  NULL
    ,  1  // Priority
    ,  NULL );

}
 
void loop()
{
  // nada a fazer aqui
}
 
/*--------------------------------------------------*/
/*---------------------- Tasks ---------------------*/
/*--------------------------------------------------*/
  
void TaskReadFromSerial(void *pvParameters)  // This is a task.
{
  (void) pvParameters;

  for (;;)
  {
    int dadoRecebido = 0;
    int tamanho      = 0;

    
    xSemaphoreTake( semaforoPrint, portMAX_DELAY );
    Serial.println("Entrar com dados\n");
    dadoRecebido = Serial.read();
    xSemaphoreGive(semaforoPrint);
    
    while (Serial.available() == 0); // espera ocupada lendo a serial - não é uma boa ideia...
   
    
    tamanho = uxQueueMessagesWaiting(fila);
    if (tamanho < tamanhoFilaTotal)
    {
      // Adiciona dado na fila
      xQueueSendToBack(fila, &dadoRecebido, 0);
      tamanho = uxQueueMessagesWaiting(fila);
      xSemaphoreTake( semaforoPrint, portMAX_DELAY );
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
 
  for (;;)
  {
    int valorRetirado = 0;
    //int tamanho = 0;
    //xSemaphoreTake( xSemaphoreSerial, portMAX_DELAY );
    

    xSemaphoreTake( semaforoPrint, portMAX_DELAY );
    tamanho = uxQueueMessagesWaiting(fila);
    xQueueReceive(fila, &valorRetirado,  portMAX_DELAY);
    Serial.println("Valor retirado: ");
    //Serial.println(valorRetirado);
    //tamanho = uxQueueMessagesWaiting(fila);
    Serial.print("Tamanho atual da fila: ");
    Serial.println(tamanho);
    xSemaphoreGive(semaforoPrint);
    
    vTaskDelay(1);  // one tick delay (15ms) in between reads for stability - b
  }
}
  
