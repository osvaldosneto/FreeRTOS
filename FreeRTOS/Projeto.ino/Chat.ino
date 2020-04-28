#include <Arduino_FreeRTOS.h>
#include <FreeRTOSVariant.h>
#include <task.h>
#include <semphr.h>
#include <timers.h>

#define BIT_PERIODO pdMS_TO_TICKS(25)
#define HALF_BIT_PERIODO pdMS_TO_TICKS(10)
#define STOP_PERIODO 3*BIT_PERIODO

const byte interruptPin = 2;
const byte outputSignalPIN = 13;

//tarefas
void TaskSender(void *pvParameters);
void TaskChatSender(void *pvParameters);
void TaskChatReceiver(void *pvParameters);

//Timer utilizado para controlar tempo de leitura
static TimerHandle_t xTimer;
//Timer utilizado para verifica start bit
static TimerHandle_t xTimerVerifica;

//Fila de armazenamento byte
QueueHandle_t queueSender = NULL;
QueueHandle_t queueReceived = NULL;
QueueHandle_t filaSinalizacao = NULL;
QueueHandle_t queueSenderStruct = NULL;
QueueHandle_t queueSet = NULL;

//variável controle byte
bool processaByte = false;

//conatador para leitura dos bits
int contador = 0;

//contador para leitura de bytes
int contadorByte = 0;

//dado recebido pelo rx
byte dadoRecebido = 0;

//Semáforo utilizado para sinalizar a Tarefa de Interrupção
SemaphoreHandle_t semaforo = NULL;

SemaphoreHandle_t semaforoPrint = NULL;

//estado da máquina de leitura
volatile static enum t_estadoLeitura {START, lendo, STOP} estadoLeitura;

volatile byte state = HIGH;

//MAQUINA DE TRANSMISSÂO
#define STOP_BIT_PERIODO 4*BIT_PERIODO

byte frame[15];

byte sender[15];
byte received[13];

struct quadro{
  byte dado[10];
  byte portDest;
  byte portFonte;
  byte dest_mac;
}quadro;


class maquina_TX {
  private:
    volatile static enum t_estado {AGUARDA_STOP, TX, FIM_TX, FIM} estado;
    static byte dado;
    static byte cont;
    static TimerHandle_t xTimerSerial;
  public:
    maquina_TX() {
      digitalWrite(outputSignalPIN, HIGH);
      xTimerSerial = xTimerCreate("Signal", STOP_BIT_PERIODO, pdTRUE, 0, timerSerialHandler);
    };

    void waitFim(){
      int aux = 0;
      xQueueReceive(filaSinalizacao, &aux, portMAX_DELAY);
    }

    static void timerSerialHandler (TimerHandle_t meuTimer) {
      switch (estado) {
        case AGUARDA_STOP:
          estado = TX;
          cont = 0;
          digitalWrite(outputSignalPIN, LOW);
          if (digitalRead(outputSignalPIN) == LOW) {
          }
          xTimerChangePeriod(meuTimer, BIT_PERIODO, 0);
          break;
        case TX:
          if (dado & B00000001) {
            digitalWrite(outputSignalPIN, HIGH);
          } else {
            digitalWrite(outputSignalPIN, LOW);
          }
          cont++;
          if (cont != 8) {
            dado = dado >> 1;
          } else {
            estado = FIM_TX;
          }
          break;
        case FIM_TX:
          xTimerStop(meuTimer, 0);
          estado = FIM;
          digitalWrite(outputSignalPIN, HIGH);
          xSemaphoreGive(semaforo);
          int i = 1;
          xQueueOverwrite(filaSinalizacao, &i);
          break;
      }
    };

    void montarQuadro(struct quadro *quadro){
       frame[0] = B00000010;
       frame[1] = B00001010 | quadro->dest_mac;
       frame[2] = B01000000;
       frame[3] = quadro->dado[0];
       frame[4] = quadro->dado[1];
       frame[5] = quadro->dado[2];
       frame[6] = quadro->dado[3];
       frame[7] = quadro->dado[4];
       frame[8] = quadro->dado[5];
       frame[9] = quadro->dado[6];
       frame[10] = quadro->dado[7];
       frame[11] = quadro->dado[8];
       frame[12] = quadro->dado[9];
       frame[13] = B00100110;
       frame[14] = B00000100;
    }
    
    void enviar_byte(byte dado) {
      estado = AGUARDA_STOP;
      this->dado = dado;
      digitalWrite(outputSignalPIN, HIGH);
      xTimerChangePeriod(xTimerSerial, STOP_BIT_PERIODO, 0);
      xTimerStart(xTimerSerial, 0);
      waitFim();
    };
} MTX;

volatile enum maquina_TX::t_estado maquina_TX::estado;
byte maquina_TX::dado;
byte maquina_TX::cont;
TimerHandle_t maquina_TX::xTimerSerial;


void setup() {

  Serial.begin(9600);
  pinMode(interruptPin, INPUT_PULLUP);
  pinMode(outputSignalPIN, OUTPUT);
  
  //Criando Semáforo Mutex
  semaforo = xSemaphoreCreateMutex();
  semaforoPrint = xSemaphoreCreateMutex(); 

  //Iniciando o estado da máquina de leitura
  estadoLeitura = STOP;

  //Criando fila de dados
  queueSender = xQueueCreate(50, sizeof(sender));
  queueReceived = xQueueCreate(50, sizeof(received));
  filaSinalizacao = xQueueCreate(1, sizeof(int));
  queueSenderStruct = xQueueCreate(50, sizeof(quadro));
  queueSet = xQueueCreateSet(50*2);

  //conjunto de filas
  xQueueAddToSet(queueSenderStruct, queueSet);
  xQueueAddToSet(queueSender, queueSet);

  //Timer com METADE do período, chamando função para verificação do start bit
  xTimerVerifica = xTimerCreate("Verifica", HALF_BIT_PERIODO, pdFALSE, 0, timerHandlerVerifica);
  //Timer que chama função de leitura
  xTimer = xTimerCreate("Timer_1", BIT_PERIODO, pdTRUE, 0, timerHandler);
  //interrupção ativada em nível baixo
  attachInterrupt(digitalPinToInterrupt(interruptPin), interrup, LOW);

  //Tarefa de envio de BYTES
  xTaskCreate(TaskSender, (const portCHAR*)"TaskSender", 128, NULL, 2, NULL);

  //Tarefa de chat - envio bytes
  xTaskCreate(TaskChatSender, (const portCHAR*)"TaskChatSender", 128, NULL, 1, NULL);

  //Tarefa de chat - recebimento bytes
  xTaskCreate(TaskChatReceiver, (const portCHAR*)"TaskChatReceiver", 128, NULL, 1, NULL);

}

void TaskSender(void *pvParameters) {
  
  (void) pvParameters;

  uint32_t receiveData;

  QueueHandle_t xQueueThatContainsData;
  byte envio[15];
   
  for(;;){
    
    xQueueThatContainsData = ( QueueHandle_t ) xQueueSelectFromSet(queueSet, portMAX_DELAY );
    if(xQueueThatContainsData == queueSender){
      
      xQueueReceive(queueSender, &envio, 0);
      for (int i=0; i<15; i++){      
        MTX.enviar_byte(envio[i]);
        envio[i] = NULL;     
      }
    }else{
      struct quadro aux;
      xQueueReceive(queueSenderStruct, &aux, 0);
      MTX.montarQuadro(&aux);
      for (int i=0; i<15; i++){
        MTX.enviar_byte(frame[i]);
        frame[i] = NULL; 
      }
      
    }
  }
}

//Tarefa de envio byte
void TaskChatSender(void *pvParameters) {
  struct quadro enviar;
  byte aux;
  int i=0;
  
  Serial.print("\nEntre com mensagem:");
  
  for (;;) {
    while (Serial.available() == 0){} // espera ocupada lendo a serial  
    aux = Serial.read(); // Lê mensagem da serial
    enviar.dado[i]=aux;  // Adiciona dados na struct
    i++;  
    if (aux=='\n') {
      i=0;
      leitura_Destino(enviar);
      Serial.print("\nEntre com mensagem:");
    }
  }
}

//Tarefa de Recebimento byte
void TaskChatReceiver(void *pvParameters) {
  for(;;){
    byte recebido[13];
    if(uxQueueMessagesWaiting(queueReceived)){     
      xQueueReceive(queueReceived, &recebido, 0);
      Serial.print("\n Mensagem Recebida: ");
      for(int i=3; i<13; i++){
        Serial.print((char)recebido[i]);
      }
    }
  }
}


void leitura_Destino(struct quadro &enviar){
  byte aux;
  Serial.print("\nEntre com destino:"); 
  while (Serial.available() == 0){} // espera ocupada lendo a serial  
  aux = Serial.parseInt(); // Lê mensagem da serial
  while(Serial.read() != '\n');
  enviar.dest_mac = aux;
  xQueueSendToBack(queueSenderStruct, &enviar, 0);
}

//Função timer inicia leitura bits
void timerHandler(TimerHandle_t Timer) {

  bool recebendo = true;
  //Análise do estado da máquina
  switch (estadoLeitura) {
    case lendo:
    
      if (contador != 8) {
        dadoRecebido = dadoRecebido >> 1;
        if (digitalRead(interruptPin) == HIGH) {
          dadoRecebido = dadoRecebido | B10000000;
        }
        contador++;      
      } else {
        byte dest = 0;
        xTimerChangePeriod(xTimer, STOP_PERIODO, 0); //mudança do timer para garantir stop bit
        estadoLeitura = STOP; //mudança do estado da máquina
        contador = 0;

        if(dadoRecebido == 2){
          recebendo = true;
        }

        if(recebendo){
          if(contadorByte==1){
            dest = B11110000 & dadoRecebido;
            verificaDestino(dest);
            if(processaByte){        
            } else {
              byte a = B00000010;
              sender[0] = B00000010;
              sender[contadorByte] = dadoRecebido;
            }         
          }
          if(contadorByte>1){
              if(processaByte){
              received[contadorByte] = dadoRecebido;
            } else {
              sender[contadorByte] = dadoRecebido;
            }     
          }
        }
        contadorByte ++;
        if(contadorByte == 15){
          if(processaByte){
            //adicionar na fila de processanento
            xQueueSendToBack(queueReceived, &received, portMAX_DELAY);
          } else {
             xQueueSendToBack(queueSender, &sender, portMAX_DELAY);
          }
          contadorByte = 0;
          processaByte = false;
        }
      }
      break;

    case STOP:
      estadoLeitura = START;
      attachInterrupt(digitalPinToInterrupt(interruptPin), interrup, LOW);
      break;
  }
}

void verificaDestino(int aux){
  if(aux == 80){
    processaByte = true;
  } else {
    processaByte = false;
  }
}

//Timer que verifica se o start bit está correto
void timerHandlerVerifica (TimerHandle_t timer) {

  if (digitalRead(interruptPin) == LOW) {
    
    xTimerChangePeriod(xTimer, BIT_PERIODO, 0); //Chama timer que fará a leitura, pois o start bit está correto!
    xTimerStart(xTimer, 0); //Start bit confirmado, alterando estado da máquina de leitura!
    estadoLeitura = lendo;
  } else {
    //ativa interrupção pois start bit está incorreto
    attachInterrupt(digitalPinToInterrupt(interruptPin), interrup, LOW);
  }
}

//interrupção em sinal baixo
void interrup(void) {

  BaseType_t xHigherPriorityTaskWoken;
  xHigherPriorityTaskWoken = pdFALSE;
  xSemaphoreGiveFromISR( semaforo, &xHigherPriorityTaskWoken );

  //timer iniciado com T/2 para verificar stop bit.
  xTimerStart(xTimerVerifica, 0);
  detachInterrupt(digitalPinToInterrupt(interruptPin));
}

void loop() {
}
