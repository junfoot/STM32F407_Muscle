#include "ads1292.h"
#include "spi.h"
#include "string.h"

// SPI 发送缓存
static uint8_t spi_tx_buf[32];

// SPI 接收缓存
static uint8_t spi_rx_buf[32];

// ADS1292寄存器
uint8_t ads1292_regs[12]; 

// ADS1292 配置信息
ads1292_info_t ads1292_info;

// ADS1292 导联脱落状态（从数据输出状态字提取，中断中更新、主循环读取）
volatile uint8_t ads1292_loff_stat = 0;

/**
 * @brief  SPI读写
 * @param  tx: 发送数据缓冲区指针 
 * @param  rx: 接收数据缓冲区指针
 * @param  len: 发送和接收的数据长度
 * @retval 无
 */
void ads1292_spi_transfer(uint8_t *tx, uint8_t *rx, uint16_t len)
{
    HAL_SPI_TransmitReceive(&hspi1, tx, rx, len, 100);
}

/**
 * @brief  设置SPI速率
 * @param  baud_rate_prescaler : 分频系数  
 * @retval 无
 */
void ads1292_set_spi_rate(uint32_t baud_rate_prescaler) 
{
    hspi1.Init.BaudRatePrescaler = baud_rate_prescaler;
    if (HAL_SPI_Init(&hspi1) != HAL_OK)
    {
        Error_Handler();
    }
}

/**
 * @brief  发送命令
 * @param  cmd: 命令码 
 * @retval 无
 */
void ads1292_write_command(uint8_t cmd)
{
    ADS1292_CS_L;
    spi_tx_buf[0] = cmd;
    ads1292_spi_transfer(spi_tx_buf, spi_rx_buf, 1);
    ADS1292_CS_H;
}

/**
 * @brief  连续写入寄存器
 * @param  addr: 寄存器起始地址 
 * @param  regs: 寄存器数组指针
 * @param  len:  写入寄存器数量
 * @retval 无
 */
void ads1292_write_regs(uint8_t addr, uint8_t *regs, uint8_t len)
{
    spi_tx_buf[0] = 0x40 + addr;
    spi_tx_buf[1] = 0x00 + len - 1;
    ADS1292_CS_L;
    ads1292_spi_transfer(spi_tx_buf, spi_rx_buf, 2);
    ads1292_spi_transfer(regs, spi_rx_buf, len);
    ADS1292_CS_H;
}

/**
 * @brief  连续读取寄存器
 * @param  addr: 寄存器起始地址 
 * @param  regs: 寄存器数组指针
 * @param  len:  读取寄存器数量
 * @retval 无
 */
void ads1292_read_regs(uint8_t addr, uint8_t *regs, uint8_t len)
{
    spi_tx_buf[0] = 0x20 + addr;
    spi_tx_buf[1] = 0x00 + len - 1;
    ADS1292_CS_L;
    ads1292_spi_transfer(spi_tx_buf, spi_rx_buf, 2);
    memset(spi_tx_buf, 0x00, sizeof(spi_tx_buf));
    ads1292_spi_transfer(spi_tx_buf, regs, len);
    ADS1292_CS_H;
}

/**
 * @brief  ADS1292上电复位
 * @retval 无
 */
void ads1292_reset(void)
{
    ADS1292_CS_H;
    ADS1292_START_L;
    ADS1292_PWDN_H; 
    HAL_Delay(200); // 将PWDN引脚拉高后，等待稳定   
    
    // 复位设备
    ADS1292_PWDN_L;
    HAL_Delay(10);
    ADS1292_PWDN_H;
    HAL_Delay(100); 
    
    // 发送停止连续读取指令
    ads1292_write_command(ADS1292_SDATAC); 
    
    // 发送停止采集指令
    ads1292_write_command(ADS1292_STOP);   
}

/**
 * @brief  ADS1292 寄存器初始化
 * @retval 无
 */
static void ads1292_regs_init(void)
{
	  /********************适用2EMG or 3ECG 版本*********************************/	
		ads1292_regs[0x00]  = 0x73;   //ID
		ads1292_regs[0x01]  = 0x04;   //CONFIG1   默认2000 SPS
		ads1292_regs[0x02]  = 0xE0;   //CONFIG2   使用内部参考电压2.42V, 关闭测试信号, 使能 LOFF 比较器
		ads1292_regs[0x03]  = 0xF0;   //LOFF
		ads1292_regs[0x04]  = 0x60;   //CH1SET    Channel 1 开启, PGA=12, 正常电极输入 (MUX=0000)
		ads1292_regs[0x05]  = 0x60;   //CH2SET    Channel 2 开启, PGA=12, 正常电极输入 (MUX=0000)
		ads1292_regs[0x06]  = 0x3F;   //RLD_SENS  CHOP=fMOD/16, PDB_RLD=1, RLD_LOFF_SENS=1, 两通道 RLD 正负端全部接入
		ads1292_regs[0x07]  = 0x0F;   //LOFF_SENS 两通道正负输入端均启用导联脱落检测
		ads1292_regs[0x08]  = 0x40;   //LOFF_STAT bit6=CLK_DIV=1, 配合外部 2.048MHz 时钟 (fMOD=fCLK/16=128kHz)
		ads1292_regs[0x09]  = 0x02;   //RESP1     关闭呼吸检测, bit1=1(必须)
		ads1292_regs[0x0A]  = 0x03;   //RESP2     RLDREF_INT=1 使用内部 RLD 参考, bit0=1(必须)
		ads1292_regs[0x0B]  = 0x03;   //GPIO      GPIO1/2 配置为输入
	
		ads1292_info.ch_en = 0x03;
		ads1292_info.ch_sw = 0x00;
		ads1292_info.pga = 12;
		ads1292_info.rate = 2000;
		ads1292_info.rld_en = 1;	// 默认开启 RLD
}

/**
 * @brief  ADS1292 初始化
 * @retval 无
 */
uint8_t ads1292_init(void)
{    
	  // 降低SPI速率传输速率
	  ads1292_set_spi_rate(SPI_BAUDRATEPRESCALER_64);
	
    // 上电复位
    ads1292_reset();

    // 寄存器初始化
	  ads1292_regs_init();
    
    // 写入寄存器 
    ads1292_write_regs(0x01, ads1292_regs + 1, 11);
    
    // 延时1毫秒
    HAL_Delay(1);
    
    // 读取寄存器
    ads1292_read_regs(0x00, ads1292_regs, 12);
      
    // 提高SPI速率传输速率
    ads1292_set_spi_rate(SPI_BAUDRATEPRESCALER_8);
	
	  /* ADS1292/ADS1292R ID寄存器高5位固定为0x70。 */
    return ((ads1292_regs[ADS1292R_ID] & 0xF0U) == 0x70U) ? 1U : 0U;
}

// 设置采样率、PGA放大倍数、通道输入
// 采样率: 125、250、500、1000、2000、4000、8000
// 量程: ±2.4V(PGA=1)、±1.2V(PGA=2)、±800mV(PGA=3)、±600mV(PGA=4)、±400mV(PGA=6)、±300mV(PGA=8)、±200mV(PGA=12)
// 通道选择:
// 0x00: 电极输入
// 0x01: 正负极内部短路
// 0x02: 测试信号
/**
 * @brief  更新采样参数
 * @retval 无
 */
void ads1292_update_sample_parameter(void)
{ 
    // 采样率
	  uint8_t rate_reg = 0x05;
	  switch(ads1292_info.rate)
		{
			case 125: rate_reg = 0x00;break;
		  case 250: rate_reg = 0x01;break;
			case 500: rate_reg = 0x02;break;
			case 1000: rate_reg = 0x03;break;
			case 2000: rate_reg = 0x04;break;	
			case 4000: rate_reg = 0x05;break;	
			case 8000: rate_reg = 0x06;break;	
		}
		ads1292_regs[0x01] &= 0xF8;
    ads1292_regs[0x01] |= rate_reg;  
		
    // PGA
		uint8_t pga_reg = 0x00;
	  switch(ads1292_info.pga)
		{
		  case 1: pga_reg = 0x01;break;
			case 2: pga_reg = 0x02;break;
			case 3: pga_reg = 0x03;break;
			case 4: pga_reg = 0x04;break;	
			case 6: pga_reg = 0x00;break;	
			case 8: pga_reg = 0x05;break;	
			case 12: pga_reg = 0x06;break;	
		}
		
		for (uint8_t ch = 0; ch < 2; ch++)
		{
				uint8_t reg = ads1292_regs[0x04 + ch];
				reg &= 0x8F;
				reg |= pga_reg << 4;
				ads1292_regs[0x04 + ch] = reg;
		} 
		
		// 通道使能
		for(uint8_t ch=0;ch<2;ch++)
		{				
				uint8_t reg = ads1292_regs[0x04+ch];
			
			  if((ads1292_info.ch_en >> ch) & 0x01)
				{
					reg &= 0x7F; // 开启通道
				}
				else
				{
				  reg |= 0x80; // 关闭通道
					reg = (reg&0xF0)|0x01;  // 将输入端短路
				}
				ads1292_regs[0x04+ch] = reg;
		}
    
    // 通道开关
    if (ads1292_info.ch_sw <= 0x02)
    { 
        uint8_t ch_regs[3] = {0x00, 0x01, 0x05};     
        for (uint8_t ch = 0; ch < 2; ch++)
        {                    
            uint8_t reg = ads1292_regs[0x04 + ch];
            reg &= 0xF0;
            reg |= ch_regs[ads1292_info.ch_sw];     
            ads1292_regs[0x04 + ch] = reg;
        }
    }
    
    // RLD（右腿驱动）开关
    if (ads1292_info.rld_en)
    {
        // 使能 RLD：CHOP=00(fMOD/16), PDB_RLD=1, RLD_LOFF_SENS=1, 两通道 RLD 正负端全部接入
        // 注：若 CH2 未接电极或用于呼吸，可改为 0x33（仅 CH1± 参与 RLD）
        ads1292_regs[0x06] = 0x3F;
    }
    else
    {
        // DC Voltage Buffer Mode：PDB_RLD=1 保持 RLD 缓冲器电源开启，
        // 断开 RLD 输入反馈（RLDxP/RLDxN=0），同时使能 RLD 脱落检测位
        // (RLD_LOFF_SENSE=1)，让 ERL 引脚稳定输出 VCM（约 1.65V）直流中间电平。
        ads1292_regs[0x06] = 0x30;
    }
    
    // 降低SPI速率，配置寄存器
    ads1292_set_spi_rate(SPI_BAUDRATEPRESCALER_64);
    HAL_Delay(1);
    
    ads1292_write_command(ADS1292_SDATAC); // 停止连续读取
    ads1292_write_command(ADS1292_STOP);   // 停止采集
    HAL_Delay(1);
    
    // 写入寄存器 
    ads1292_write_regs(0x01, ads1292_regs + 1, 11);
    
    // 加入延时，确保能读取寄存器成功
    HAL_Delay(1);
    
    // 读取寄存器
    ads1292_read_regs(0x00, ads1292_regs, 12);
    
    // 提高SPI速率，读取数据
    ads1292_set_spi_rate(SPI_BAUDRATEPRESCALER_8);
}

/**
 * @brief  停止采集
 * @retval 无
 */
void ads1292_stop(void)
{  
    ads1292_write_command(ADS1292_SDATAC); // 停止连续读取
    ADS1292_START_L;
    ADS1292_CS_H;
}

/**
 * @brief  开始采集
 * @retval 无
 */
void ads1292_start(void)
{
    memset(spi_tx_buf, 0x00, sizeof(spi_tx_buf));
    ads1292_write_command(ADS1292_RDATAC); // 开启连续读取
    ADS1292_START_H;
    ADS1292_CS_L;
}

/**
 * @brief  读取数据
 * @param  ch_val: 通道数据数组指针
 * @retval 无
 */
void ads1292_read_data(float *ch_val)
{    
	  // 读取9个字节，前三个字节为状态字节，后6个字节是2个通道的数据，每个通道占3个字节
    ads1292_spi_transfer(spi_tx_buf, spi_rx_buf, 9);

    // 从状态字提取 LOFF_STAT[4:0]
    // 24-bit 状态字格式：1100 + LOFF_STAT[4:0] + GPIO[1:0] + 13'b0
    // status[23:20]=1100, status[19:15]=LOFF_STAT[4:0], status[14:13]=GPIO[1:0]
    // 接收字节0 = status[23:16], 字节1 = status[15:8]
    ads1292_loff_stat = ((spi_rx_buf[0] & 0x0F) << 1) | ((spi_rx_buf[1] & 0x80) >> 7);
	
	  // 转化成电压,单位uV
    for (uint8_t ch = 0; ch < 2; ch++)
    {
        uint8_t index = (uint8_t)(3U + 3U * ch);
        uint32_t raw24 = ((uint32_t)spi_rx_buf[index] << 16)
                       | ((uint32_t)spi_rx_buf[index + 1U] << 8)
                       |  (uint32_t)spi_rx_buf[index + 2U];
        int32_t signed_raw = (raw24 & 0x00800000UL) != 0U
                           ? (int32_t)(raw24 | 0xFF000000UL)
                           : (int32_t)raw24;
        ch_val[ch] = (float)signed_raw;
        
        // ((2*2.42)/2^24)*10^6  = 0.288486 
        ch_val[ch] = (ch_val[ch] * 0.288486f) / ads1292_info.pga;  // 单位uV 	
    }
}
