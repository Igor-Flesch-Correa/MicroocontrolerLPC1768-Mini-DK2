#include "LibCMSIS.h"
#include "LibFreeRTOS.h"
#include "definicoesProjeto.h"
#include <string.h>
#include <stdio.h>


static TimerHandle_t blinkTmr;

//================================================================================
//Protótipos de funções

static void blinkCallBack( TimerHandle_t timer );
static void vTask1( void * pvParameters );
static void vTaskbotao( void * pvParameters );
static void vTaskMandaUART(void * pvParameters);

TaskHandle_t xHandleTask1 = NULL; // inicializa handle task 1

SemaphoreHandle_t xBinarySemaphore;

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
		
		
		xBinarySemaphore = xSemaphoreCreateBinary();
		
		xTaskCreate(vTask1, "NomeTask1", configMINIMAL_STACK_SIZE, NULL, 1, &xHandleTask1); //cria task 1
		xTaskCreate(vTaskbotao, "NomeTaskbotao", configMINIMAL_STACK_SIZE, NULL, 1, NULL);
		xTaskCreate(vTaskMandaUART, "MandaUART", configMINIMAL_STACK_SIZE, NULL, 1, NULL);
    //
  //  blinkTmr= xTimerCreate( "blinkTmr", pdMS_TO_TICKS(100), pdTRUE, 0, blinkCallBack );		
   // xTimerStart( blinkTmr, 0 );               
    
		              
    
    //
    vTaskStartScheduler();
    
}


//================================================================================
static void vTask1( void * pvParameters )// task numero 1 vai aqui
		{
					while(1)
					{
						if( GPIO_ReadValue(led2_PORT) & (1 << led2_BIT) )
								GPIO_ClearValue( led2_PORT, 1 << led2_BIT );
						else
								GPIO_SetValue( led2_PORT, 1 << led2_BIT ); 
						
					vTaskDelay(pdMS_TO_TICKS(400));	
					}
		}

//================================================================================
		
static void vTaskMandaUART(void * pvParameters)//vai receber da fila
		{
			Message msgToSend;
			
			while(1)
			{
				if(xSemaphoreTake(xBinarySemaphore, portMAX_DELAY) == pdTRUE)//semaforo
				{
					strcpy(msgToSend.message, "Hello, world!\r\n");
					UART_Send(LPC_UART0, (uint8_t *)msgToSend.message, strlen(msgToSend.message), BLOCKING);
				}
			}
		}

//====================================================================================================
		
static void vTaskbotao( void * pvParameters )// task numero 1 vai aqui
		{	
					Message msgToSend; //declara variavel do tipo struct perssonalizado
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
