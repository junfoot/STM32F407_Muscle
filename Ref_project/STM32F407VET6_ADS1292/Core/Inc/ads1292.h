#ifndef __ADS1292_H
#define __ADS1292_H

#include "main.h"

#define ADS1292_CS_PORT                ADS129X_CS_GPIO_Port
#define ADS1292_CS_PIN                 ADS129X_CS_Pin
#define ADS1292_START_PORT             ADS129X_START_GPIO_Port
#define ADS1292_START_PIN              ADS129X_START_Pin
#define ADS1292_PWDN_PORT              ADS129X_PWDN_GPIO_Port
#define ADS1292_PWDN_PIN               ADS129X_PWDN_Pin
#define ADS1292_DRDY_PORT              ADS129X_DRDY_GPIO_Port
#define ADS1292_DRDY_PIN               ADS129X_DRDY_Pin

#define ADS1292_CS_H			HAL_GPIO_WritePin(ADS1292_CS_PORT, ADS1292_CS_PIN,GPIO_PIN_SET)
#define ADS1292_CS_L			HAL_GPIO_WritePin(ADS1292_CS_PORT, ADS1292_CS_PIN,GPIO_PIN_RESET)
#define ADS1292_PWDN_H		HAL_GPIO_WritePin(ADS1292_PWDN_PORT, ADS1292_PWDN_PIN,GPIO_PIN_SET)
#define ADS1292_PWDN_L		HAL_GPIO_WritePin(ADS1292_PWDN_PORT, ADS1292_PWDN_PIN,GPIO_PIN_RESET)
#define ADS1292_START_H		HAL_GPIO_WritePin(ADS1292_START_PORT, ADS1292_START_PIN,GPIO_PIN_SET)
#define ADS1292_START_L		HAL_GPIO_WritePin(ADS1292_START_PORT, ADS1292_START_PIN,GPIO_PIN_RESET)

/*ADS1292命令定义*/
/*系统命令*/
#define ADS1292_WAKEUP	        0X02	//从待机模式唤醒
#define ADS1292_STANDBY	        0X04	//进入待机模式
#define ADS1292_ADSRESET        0X06	//复位
#define ADS1292_START	        	0X08	//启动或转换
#define ADS1292_STOP	        	0X0A	//停止转换
#define ADS1292_OFFSETCAL				0X1A	//通道偏移校准
/*数据读取命令*/
#define ADS1292_RDATAC	        0X10	//启用连续的数据读取模式,默认使用此模式
#define ADS1292_SDATAC	        0X11	//停止连续的数据读取模式
#define ADS1292_RDATA	        	0X12	//通过命令读取数据;支持多种读回。
/*寄存器读取命令*/
#define	ADS1292_RREG	        	0X20	//读取001r rrrr 000n nnnn  这里定义的只有高八位，低8位在发送命令时设置  r rrrr=要读、写的寄存器地址
#define ADS1292_WREG	        	0X40	//写入010r rrrr 000n nnnn   n nnnn=要读、写的数据

/* ADS1292内部寄存器地址定义	*/
#define ADS1292R_ID								0X00	//ID控制寄存器
#define ADS1292R_CONFIG1					0X01	//配置寄存器1
#define ADS1292R_CONFIG2					0X02	//配置寄存器2
#define ADS1292R_LOFF							0X03	//导联脱落控制寄存器
#define ADS1292R_CH1SET						0X04	//通道1设置寄存器
#define ADS1292R_CH2SET						0X05	//通道2设置寄存器
#define ADS1292R_RLD_SENS					0X06	//右腿驱动选择寄存器
#define ADS1292R_LOFF_SENS				0X07	//导联脱落检测选择寄存器
#define ADS1292R_LOFF_STAT				0X08	//导联脱落检测状态寄存器
#define	ADS1292R_RESP1						0X09	//呼吸检测控制寄存器1
#define	ADS1292R_RESP2						0X0A	//呼吸检测控制寄存器2
#define	ADS1292R_GPIO							0X0B	//GPIO控制寄存器


typedef struct {
	
	// 采样率
	uint16_t rate; 
	
	// 模拟前端放大倍数
	uint8_t pga;
	
	// 通道使能
	uint8_t ch_en;
	
	// 通道输入
	uint8_t ch_sw;
	
	// RLD（右腿驱动）使能：0=关闭，1=开启
	uint8_t rld_en;
	
}ads1292_info_t;

extern ads1292_info_t ads1292_info;
extern volatile uint8_t ads1292_loff_stat;   // 导联脱落状态字

uint8_t ads1292_init(void); //初始化ADS1292R，成功返回1
void ads1292_start(void); //开始采集
void ads1292_stop(void);  //停止采集
void ads1292_read_data(float *chx_val);//读ADS1292数据

void ads1292_update_sample_parameter(void);


#endif
