#include <stdint.h>

#define FW_VERSION  1
#define FW_DATE_STR "EME91/EME96"

#define MRP_MODEL   "022024"

extern uint8_t ucMACAddress[6];
extern uint8_t contactCycle;

//====================================================================
//chaves de criptografia AES

#define UDP_LOCAL_CRYPTO_KEY    "Le8*RgD+4Gb11L9c"
#define UDP_CLOUD_CRYPTO_KEY    "H27z8Bg*Lv14O7p="   

//====================================================================

#define RUNLED_PORT 3
#define RUNLED_BIT  25

#define led2_PORT 3
#define led2_BIT 26

#define botao_PORT 2
#define botao_BIT 10






#define SDCARD_DETECT_PORT  1
#define SDCARD_DETECT_BIT   25

#define SERIAL_ID_PORT  1
#define SERIAL_ID_BIT   22

#define MOTOR_TOP_SNS_PORT  0
#define MOTOR_TOP_SNS_BIT   25

#define MOTOR_BOT_SNS_PORT  1
#define MOTOR_BOT_SNS_BIT   30

#define PIN1_SENS_PORT  3
#define PIN1_SENS_BIT   26

#define PIN2_SENS_PORT  0
#define PIN2_SENS_BIT   29

#define PIN3_SENS_PORT  0
#define PIN3_SENS_BIT   28

#define PIN4_SENS_PORT  1
#define PIN4_SENS_BIT   18

#define PIN5_SENS_PORT  0
#define PIN5_SENS_BIT   27

#define PIN6_SENS_PORT  0
#define PIN6_SENS_BIT   23

#define PIN7_SENS_PORT  2
#define PIN7_SENS_BIT   11

#define PIN8_SENS_PORT  0
#define PIN8_SENS_BIT   30

#define PIN9_SENS_PORT  1
#define PIN9_SENS_BIT   31

#define PIN10_SENS_PORT  0
#define PIN10_SENS_BIT   24

#define MOTOR_UP_PORT   0
#define MOTOR_UP_BIT    5

#define MOTOR_DOWN_PORT 0
#define MOTOR_DOWN_BIT  1

#define BUMPER_UP_PORT  0
#define BUMPER_UP_BIT   0

#define BUMPER_DOWN_PORT  2
#define BUMPER_DOWN_BIT   9

#define SR_STB_PORT 4
#define SR_STB_BIT  29

#define SR_DATA_PORT    4
#define SR_DATA_BIT     28

#define SR_CLK_PORT 0
#define SR_CLK_BIT  4

#define BALL_SNS_PORT	2
#define BALL_SNS_BIT	12

#define KEY_ISP_PORT    2
#define KEY_ISP_BIT     10

#define INV_RELAY_PORT  1
#define INV_RELAY_BIT   19

#define MEM_CS_PORT 0
#define MEM_CS_BIT  11

//====================================================================
//Mapeamento da EEPROM externa

//0-31: não utilizados

#define INDEX_TEST_INIT             32 //1 byte
#define INDEX_CFG_IP                33 //4 bytes
#define INDEX_CFG_MASK              37 //4 bytes
#define INDEX_CFG_GATEWAY           41 //4 bytes
#define INDEX_CFG_DNS               45 //4 bytes
#define INDEX_POWERUP               49 //2 bytes
#define INDEX_CPU_ODOMETER          51 //4 bytes
#define INDEX_TOTAL_BUMPER          55 //4 bytes
#define INDEX_BALL_SENSOR_ERRORS    59 //2 bytes
#define INDEX_TOTAL_REARMS		    61 //4 bytes
#define INDEX_REARM_ERRORS          65 //2 bytes
#define INDEX_MOTOR_ODOMETER	    67 //4 bytes 
#define INDEX_PIN_STUCK             71 //4 bytes
#define INDEX_STUCK_AUTO_CLR        75 //4 bytes
#define INDEX_UNSTUCK_ATTEMPTS      79 //4 bytes
#define INDEX_LANGUAGE              83 //1 byte
#define INDEX_PSM_BLOCKED           84 //1 byte
#define INDEX_CTRL_OFFLINE          85 //1 byte
#define INDEX_PTR_STORED            86 //2 bytes
#define INDEX_PTR_SENT              88 //2 bytes
#define INDEX_CFG_GAME_HOST         90 //4 bytes
#define INDEX_LANE_PAUSED           94 //1 byte
#define INDEX_LANE_REGISTERED       95 //1 byte
#define INDEX_REED_MOTOR_TIME_ON    96 //2 bytes
#define INDEX_REED_MOTOR_TIME_OFF   98 //2 bytes
#define INDEX_MOTOR_REF             100//2 bytes
#define INDEX_MACHINE_TYPE          102//1 byte
#define INDEX_SNS_TYPE              103//1 byte
#define INDEX_BOARD_TYPE            104//1 byte

#define INDEX_INI_LOGS              4096
