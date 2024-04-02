#include "LibCMSIS.h"
#include "LibFreeRTOS.h"
#include "definicoesProjeto.h"
#include "LibNetwork.h"
#include <string.h>
#include <stdio.h>


static TimerHandle_t blinkTmr;


//================================================================================

static uint8_t ip[4] = {192,168,53,216};
static uint8_t mask[4] = {255,255,255,0};
static uint8_t gateway[4] = {192,168,53,254};
static uint8_t dns[4] = {8,8,8,8};
uint8_t ucMACAddress[6] = {0x00,0x18,0x80,0x01,0x02,0x03};

//================================================================================
//Protótipos de funções

static void blinkCallBack( TimerHandle_t timer );
static void vTask1( void * pvParameters );
static void vTaskbotao( void * pvParameters );
static void vTaskbotao1( void * pvParameters );
static void vTaskMandaUART(void * pvParameters);
static void vTaskMandaUARTnotif(void * pvParameters);
uint16_t leCanalAD( unsigned char canal );
static void vUDPServer( void *pvParameters );

static SemaphoreHandle_t mutexADC;//pra ler

SemaphoreHandle_t xBinarySemaphore;

SemaphoreHandle_t semaforoUART;

TaskHandle_t xHandleTask1 = NULL; // inicializa handle task 1

TaskHandle_t xTaskReceiver;// Criação do objeto de notificação



typedef struct {
		char message[50]; // Tamanho máximo da mensagem
}Message;

//================================================================================

int main( void ){
    UART_CFG_Type
        uartCfg;
    UART_FIFO_CFG_Type
        uartFifo;
    PINSEL_CFG_Type
        pinsel;       
    
    //
    if( LPC_SC->PLL0CFG == 0 )
        SystemInit();
    SystemCoreClockUpdate();
    NVIC_SetPriorityGrouping( 2 );
    
    //=============================================================================
    
    //RUN LED como saída
    GPIO_SetDir( RUNLED_PORT, 1 << RUNLED_BIT, 1 );
    GPIO_SetDir(led2_PORT, 1 << led2_BIT, 1);
   
    //==============================================
    //UART0 (USB_Serial)
    
    UART_FIFOConfigStructInit( &uartFifo );
    
    UART_ConfigStructInit( &uartCfg );
    
    UART_Init( LPC_UART0, &uartCfg );
    
    UART_TxCmd( LPC_UART0, ENABLE );

    //TXD0
    pinsel.Funcnum= 1;
    pinsel.Portnum= 0;
    pinsel.Pinnum= 2;
    PINSEL_ConfigPin( &pinsel );
    
    //RXD0
    pinsel.Funcnum= 1;
    pinsel.Portnum= 0;
    pinsel.Pinnum= 3;
    PINSEL_ConfigPin( &pinsel );    
  
    //==============================================
		//configura P0.26 como AD0.3 (medição de tensão da bateria)
			pinsel.Portnum= PINSEL_PORT_0;		    
			pinsel.Pinmode= PINSEL_PINMODE_TRISTATE;	    
			pinsel.Pinnum= PINSEL_PIN_26;
			pinsel.Funcnum= 1;
			PINSEL_ConfigPin( &pinsel );  

			//inicializa o conversor AD do LPC1768
			ADC_Init( LPC_ADC, 100000 );
		
		//==============================================
		
		FreeRTOS_IPInit( ip, mask, gateway, dns, ucMACAddress );
		
		xBinarySemaphore = xSemaphoreCreateBinary();
		mutexADC = xSemaphoreCreateMutex();
		semaforoUART = xSemaphoreCreateMutex();//semaforo mutex para mandar uart
		
		xTaskCreate(vTask1, "NomeTask1", configMINIMAL_STACK_SIZE, NULL, 1, &xHandleTask1); //cria task 1
		xTaskCreate(vTaskbotao, "NomeTaskbotao", configMINIMAL_STACK_SIZE, NULL, 1, NULL);
		xTaskCreate(vTaskbotao1, "NomeTaskbotao1", configMINIMAL_STACK_SIZE, NULL, 1, NULL);
		xTaskCreate(vTaskMandaUART, "MandaUART", configMINIMAL_STACK_SIZE, NULL, 1, NULL);
		xTaskCreate(vTaskMandaUARTnotif, "NomeTask1notif", configMINIMAL_STACK_SIZE, NULL, 1, &xTaskReceiver);//recebe notificação
		
		xTaskCreate(vUDPServer, "recebe_pacote", configMINIMAL_STACK_SIZE, NULL, 1, NULL);
    //
  //  blinkTmr= xTimerCreate( "blinkTmr", pdMS_TO_TICKS(100), pdTRUE, 0, blinkCallBack );		
   // xTimerStart( blinkTmr, 0 );               
    
    vTaskStartScheduler();
    
}


//================================================================================
void vApplicationIPNetworkEventHook( eIPCallbackEvent_t eNetworkEvent ){}

//================================================================================
	
static void vUDPServer( void *pvParameters ){    // recebe um pacote e devolve o que recebeu
    static const TickType_t 
        xReceiveTimeOut = portMAX_DELAY;
    struct freertos_sockaddr         
        xBindAddress, xSourceAddress;
    static uint8_t
        udpServerRxBuffer[1024],
        aesBuf[1024];
    BaseType_t 
        lBytesReceived;         
    uint16_t
        length;    
		Socket_t xListeningSocket;
   
    /* Cria o socket. */
		xListeningSocket = FreeRTOS_socket( FREERTOS_AF_INET, FREERTOS_SOCK_DGRAM, FREERTOS_IPPROTO_UDP );
	  
    
    //
    FreeRTOS_setsockopt( xListeningSocket, 0, FREERTOS_SO_RCVTIMEO, &xReceiveTimeOut, sizeof(xReceiveTimeOut) );
    
    //    
    xBindAddress.sin_port= FreeRTOS_htons( 5000 );
    
    /* Bind the socket to the port that the client RTOS task will send to. */
    FreeRTOS_bind( xListeningSocket, &xBindAddress, sizeof( xBindAddress ) );   
    
    
    for(;;){        
        
        //====================================================================        
        
        memset( aesBuf, 0, sizeof(aesBuf) );          
        
        lBytesReceived= FreeRTOS_recvfrom( xListeningSocket, aesBuf, sizeof(aesBuf), 0, &xSourceAddress, (socklen_t *)sizeof(xSourceAddress) );        
                                
            
				//====================================================================
				if( strlen((char *)aesBuf) ){
						//====================================================================
						//                 
						/* Num teste alternando os comandos de abrir e fechar pista a cada 100ms,
						não retorna mais desta função após processar alguns comandos. Provavelmente,
						por não conseguir mais alocar memória para o envio, pois o timeout default do envio
						é portMAX_DELAY. */
						FreeRTOS_sendto( xListeningSocket, aesBuf, lBytesReceived, 0, &xSourceAddress, sizeof(xSourceAddress) );  

				}               
        
    }
    
}

//=======================================================================
static void funcaoUART(const char *msgToSend) {
    if (xSemaphoreTake(semaforoUART, portMAX_DELAY) == pdTRUE) // Tenta obter o semáforo mutex
		{ 
        UART_Send(LPC_UART0, (uint8_t *)msgToSend, strlen(msgToSend), BLOCKING);
        xSemaphoreGive(semaforoUART); // Libera o semáforo após o envio da mensagem
    }
}


//================================================================================
uint16_t leCanalAD( unsigned char canal ){    
    uint16_t
        valorAD;

    //obtém o mutex para acesso ao ADC
    xSemaphoreTake( mutexADC, portMAX_DELAY );   

    //habilita o canal desejado 
    ADC_ChannelCmd( LPC_ADC, canal, ENABLE );   

    //Inicia uma conversão
    ADC_StartCmd( LPC_ADC, ADC_START_NOW );
    
    //Espera a conversão terminar
    while( ADC_ChannelGetStatus(LPC_ADC, canal, ADC_DATA_DONE) == RESET );

    //lê o resultado da conversão
    valorAD= ADC_ChannelGetData ( LPC_ADC, canal );    

    //desabilita o canal que foi habilitado para realizar a conversão
    ADC_ChannelCmd( LPC_ADC, canal, DISABLE );
   
    //
    //delay_us( 10 );

    //devolve o mutex para acesso ao ADC
    xSemaphoreGive( mutexADC );
    
    return valorAD;    

}
//================================================================================
static void vTask1( void * pvParameters )// task numero 1 vai aqui
		{
			char buffer[20];
			
					while(1)
					{
						if( GPIO_ReadValue(led2_PORT) & (1 << led2_BIT) )
								GPIO_ClearValue( led2_PORT, 1 << led2_BIT );
						else
								GPIO_SetValue( led2_PORT, 1 << led2_BIT ); 
						
						// Variável para armazenar o valor do canal AD
						uint16_t valorAD = leCanalAD(3);

						// Buffer para armazenar a conversão do valor AD em uma string
						
						sprintf(buffer, "%d\r\n", valorAD);

						// Envio do valor convertido pela porta UART
						funcaoUART(buffer);
						
					vTaskDelay(pdMS_TO_TICKS(200));	
					}
		}

//================================================================================
		
static void vTaskMandaUART(void * pvParameters) {
    Message msgToSend;
    
    while(1) {
        if(xSemaphoreTake(xBinarySemaphore, portMAX_DELAY) == pdTRUE) {
            strcpy(msgToSend.message, "Hello, world!\r\n");
            funcaoUART(msgToSend.message); 
        }
    }
}
//================================================================================
		
static void vTaskMandaUARTnotif(void * pvParameters)//vai receber da fila
		{
			Message msgToSend;
			
			while(1)
			{
				if( ulTaskNotifyTake(pdTRUE, 0))//semaforo
				{
					strcpy(msgToSend.message, "funcionando com notificaçao !\r\n");
					funcaoUART(msgToSend.message);
				}
			}
		}

//====================================================================================================
		
static void vTaskbotao( void * pvParameters )// task numero 1 vai aqui
		{	
					uint8_t filtroBotao = 0;
			
					while(1)
					{
						if(!(GPIO_ReadValue(botao_PORT) & (1 << botao_BIT)) )
						{
							if(filtroBotao < 10)
							{
								filtroBotao++;
								if(filtroBotao == 10)
								{
									//semaforo binario
									xSemaphoreGive(xBinarySemaphore);
								}

							}
							
							vTaskSuspend(xHandleTask1);// para de piscar o led enquanto presiona
						}else
						{
							filtroBotao = 0;
							vTaskResume(xHandleTask1);
						}
					}
		}
		
static void vTaskbotao1( void * pvParameters )// task numero 1 vai aqui
		{	
					uint8_t filtroBotao = 0;
			
					while(1)
					{
						if(!(GPIO_ReadValue(botao_1_PORT) & (1 << botao_1_BIT)) )
						{
							if(filtroBotao < 10)
							{
								filtroBotao++;
								if(filtroBotao == 10)
								{
									//semaforo binario
									xTaskNotifyGive(xTaskReceiver);
								}

							}
							
							vTaskSuspend(xHandleTask1);
						}else
						{
							filtroBotao = 0;
							vTaskResume(xHandleTask1);
						}
					}
		}
		
//==========================================================================================================	
		
static void blinkCallBack( TimerHandle_t timer ){
    
    if( GPIO_ReadValue(led2_PORT) & (1 << led2_BIT) )
        GPIO_ClearValue( led2_PORT, 1 << led2_BIT );
    else
        GPIO_SetValue( led2_PORT, 1 << led2_BIT );   
	
}


//================================================================================
/*No momento não está sendo usada, pois o kernel está sendo compilado
sem a opção configASSERT */


void vAssertCalled( const char *pcFile, uint32_t ulLine ){
    char strAux[32];
    
    sprintf( strAux, "F:%s, L:%u\r\n", (char *)*pcFile, ulLine );
    UART_Send( LPC_UART0, (uint8_t *)strAux, strlen(strAux), BLOCKING );
}

//================================================================================

void vApplicationStackOverflowHook( xTaskHandle *pxTask, signed portCHAR *pcTaskName ){ 

    taskDISABLE_INTERRUPTS();
    
    while( 1 ){
        UART_Send( LPC_UART0, (uint8_t *)pcTaskName, strlen((char *)pcTaskName), BLOCKING );
        UART_Send( LPC_UART0, "\r\n", 2, BLOCKING );//quebra de linha p/ facilitar a leitura no terminal serial
        
        if( GPIO_ReadValue(RUNLED_PORT) & (1 << RUNLED_BIT) )
            GPIO_ClearValue( RUNLED_PORT, 1 << RUNLED_BIT );
        else
            GPIO_SetValue( RUNLED_PORT, 1 << RUNLED_BIT );
        
        //delay_ms( 100 );
    }
    
}
