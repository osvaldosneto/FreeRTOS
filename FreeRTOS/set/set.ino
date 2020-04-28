#include <Arduino_FreeRTOS.h>
#include <FreeRTOSVariant.h>
#include <queue.h>
#include <task.h>
#include <semphr.h>

void TaskSender1(void *pvParameters);
void TaskSender2(void *pvParameters);
void TaskReceiver1(void *pvParameters);

SemaphoreHandle_t semaforoPrint = NULL;
QueueHandle_t queue1 = NULL;
QueueHandle_t queue2 = NULL;
QueueHandle_t queueSet = NULL;

void setup() {
  
  Serial.begin(9600);
 
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB, on LEONARDO, MICRO, YUN, and other 32u4 based boards.
  }

  // Semáforo print.
  semaforoPrint = xSemaphoreCreateMutex(); 
  
  //Criando as filas e o conjunto delas.
  queue1   = xQueueCreate(1, sizeof(int));
  queue2   = xQueueCreate(1, sizeof(int));
  queueSet = xQueueCreateSet(1*2);

  //Adicionando as filas no conjunto.
  xQueueAddToSet(queue1, queueSet);
  xQueueAddToSet(queue2, queueSet);

  //Criando 2 tarefas de escrita e 1 de leitura.
  xTaskCreate(TaskSender1,(const portCHAR *)"Sender1", 128, NULL, 1, NULL );
  xTaskCreate(TaskSender2,(const portCHAR *)"Sender2", 128, NULL, 1, NULL );
  xTaskCreate(TaskReceiver1,(const portCHAR *)"Receiver1", 128, NULL, 2, NULL );

  vTaskStartScheduler();
}

void loop() { /* Do nothing */ }

void TaskSender1(void *pvParameters)
{
  int dadoEnvio = 1;
  for(;;){
    vTaskDelay(50);
    xQueueSend(queue1, &dadoEnvio, 0);
    dadoEnvio = dadoEnvio + 2;
  }

}

void TaskSender2(void *pvParameters)
{
  
  int dadoEnvio = 2;
  for(;;){
    vTaskDelay(100);
    xQueueSend(queue2, &dadoEnvio, 0);
    dadoEnvio = dadoEnvio + 2;
  }

}

void TaskReceiver1(void *pvParameters){

  QueueHandle_t xQueueThatContainsData;
  int dadoRecebido = 0;
  int dadoEnvio = 1;
  
  for(;;){
    xQueueThatContainsData = ( QueueHandle_t ) xQueueSelectFromSet(queueSet, portMAX_DELAY );
    xQueueReceive(xQueueThatContainsData, &dadoRecebido, 0);
    
    xSemaphoreTake(semaforoPrint, portMAX_DELAY);
    if(xQueueThatContainsData == queue1){
      Serial.println("Recebi dados da fila 1:");
      Serial.print("Dado recebido:");
      Serial.println(dadoRecebido);
    }else{
      Serial.println("Recebi dados da fila 2");
      Serial.print("Dado recebido:");
      Serial.println(dadoRecebido);
    }
    xSemaphoreGive(semaforoPrint);
  }

}
