/*********************************************************************************************************************
 * STC32G144K Opensourec Library 即（STC32G144K 开源库）是一个基于官方 SDK 接口的第三方开源库
 * Copyright (c) 2025 SEEKFREE 逐飞科技
 *
 * 本文件是STC32G144K开源库的一部分
 *
 * STC32G144K 开源库 是免费软件
 * 您可以根据自由软件基金会发布的 GPL（GNU General Public License，即 GNU通用公共许可证）的条款
 * 即 GPL 的第3版（即 GPL3.0）或（您选择的）任何后来的版本，重新发布和/或修改它
 *
 * 本开源库的发布是希望它能发挥作用，但并未对其作任何的保证
 * 甚至没有隐含的适销性或适合特定用途的保证
 * 更多细节请参见 GPL
 *
 * 您应该在收到本开源库的同时收到一份 GPL 的副本
 * 如果没有，请参阅<https://www.gnu.org/licenses/>
 *
 * 额外注明：
 * 本开源库使用 GPL3.0 开源许可证协议 以上许可申明为译文版本
 * 许可申明英文版在 libraries/doc 文件夹下的 GPL3_permission_statement.txt 文件中
 * 许可证副本在 libraries 文件夹下 即该文件夹下的 LICENSE 文件
 * 欢迎各位使用并传播本程序 但修改内容时必须保留逐飞科技的版权声明（即本声明）
 *
 * 文件名称
 * 公司名称          成都逐飞科技有限公司
 * 版本信息          查看 libraries/doc 文件夹内 version 文件 版本说明
 * 开发环境          MDK FOR C251
 * 适用平台          STC32G144K
 * 店铺链接          https://seekfree.taobao.com/
 *
 * 修改记录
 * 日期              作者           备注
 * 2025-11-20         大W            first version
 ********************************************************************************************************************/

#include "zf_common_headfile.h"

// 本文件只保留向量号 0~31 的中断，C251 的 interrupt 属性不支持更大的编号。
// 向量号大于 31 的扩展中断（P0~PB、DMA UART1~8、DMA_LCM、TM5~TM18）见
// isr_extended_0.c / isr_extended_1.c，由 isr_extended_vectors.A51 完成真实向量地址映射。

void INT0_IRQHandler() interrupt INT0_VECTOR
{
	if (int_irq_handlers[0] != NULL)
    {
        int_irq_handlers[0]();
    }
}

//void INT1_IRQHandler(void) interrupt INT1_VECTOR  //外部中断1，默认留给摄像头

void INT2_IRQHandler(void) interrupt INT2_VECTOR
{
	if (int_irq_handlers[2] != NULL)
    {
		int_irq_handlers[2]();
    }
}
void INT3_IRQHandler(void) interrupt INT3_VECTOR
{
	if (int_irq_handlers[3] != NULL)
    {
		int_irq_handlers[3]();
    }
}

void INT4_IRQHandler(void) interrupt INT4_VECTOR
{
	if (int_irq_handlers[4] != NULL)
    {
		int_irq_handlers[4]();
    }
}

void TM0_IRQHandler() interrupt TMR0_VECTOR
{
    TIM0_CLEAR_FLAG;
    if (tim_irq_handlers[TIM_0] != NULL)
    {
        tim_irq_handlers[TIM_0]();
    }
}

void TM1_IRQHandler() interrupt TMR1_VECTOR
{
    TIM1_CLEAR_FLAG;
    if (tim_irq_handlers[TIM_1] != NULL)
    {
        tim_irq_handlers[TIM_1]();
    }
}

void TM2_IRQHandler() interrupt TMR2_VECTOR
{
    TIM2_CLEAR_FLAG;
    if (tim_irq_handlers[TIM_2] != NULL)
    {
        tim_irq_handlers[TIM_2]();
    }
}

void TM3_IRQHandler() interrupt TMR3_VECTOR
{
    TIM3_CLEAR_FLAG;
    if (tim_irq_handlers[TIM_3] != NULL)
    {
        tim_irq_handlers[TIM_3]();
    }
}

void TM4_IRQHandler() interrupt TMR4_VECTOR
{
    TIM4_CLEAR_FLAG;
    if (tim_irq_handlers[TIM_4] != NULL)
    {
        tim_irq_handlers[TIM_4]();
    }
}

void INT1_IRQHandler() interrupt INT1_VECTOR
{
//    if(mt9v03x_finish_flag == 1)
//    {
//        return;
//    }
    mt9v03x_vsync_handler();
}




//#define     INT0_VECTOR             0       //0003H
//#define     TMR0_VECTOR             1       //000BH
//#define     INT1_VECTOR             2       //0013H
//#define     TMR1_VECTOR             3       //001BH
//#define     UART1_VECTOR            4       //0023H
//#define     ADC_VECTOR              5       //002BH
//#define     LVD_VECTOR              6       //0033H
////#define   PCA_VECTOR              7       //003BH
//#define     UART2_VECTOR            8       //0043H
//#define     SPI_VECTOR              9       //004BH
//#define     INT2_VECTOR             10      //0053H
//#define     INT3_VECTOR             11      //005BH
//#define     TMR2_VECTOR             12      //0063H
//#define     USER_VECTOR             13      //006BH
//#define     BRK_VECTOR              14      //0073H
//#define     ICEP_VECTOR             15      //007BH
//#define     INT4_VECTOR             16      //0083H
//#define     UART3_VECTOR            17      //008BH
//#define     UART4_VECTOR            18      //0093H
//#define     TMR3_VECTOR             19      //009BH
//#define     TMR4_VECTOR             20      //00A3H
//#define     CMP_VECTOR              21      //00ABH
////#define   PWM_VECTOR              22      //00B3H
////#define   PWMFD_VECTOR            23      //00BBH
//#define     I2C_VECTOR              24      //00C3H
//#define     USB_VECTOR              25      //00CBH
//#define     PWMA_VECTOR             26      //00D3H
//#define     PWMB_VECTOR             27      //00DBH
//#define     CAN1_VECTOR             28      //00E3H
//#define     CAN2_VECTOR             29      //00EBH
//#define     LIN_VECTOR              30      //00F3H

//#define     RTC_VECTOR              36      //0123H
//#define     P0INT_VECTOR            37      //012BH
//#define     P1INT_VECTOR            38      //0133H
//#define     P2INT_VECTOR            39      //013BH
//#define     P3INT_VECTOR            40      //0143H
//#define     P4INT_VECTOR            41      //014BH
//#define     P5INT_VECTOR            42      //0153H
//#define     P6INT_VECTOR            43      //015BH
//#define     P7INT_VECTOR            44      //0163H
//#define     DMA_M2M_VECTOR          47      //017BH
//#define     DMA_ADC_VECTOR          48      //0183H
//#define     DMA_SPI_VECTOR          49      //018BH
//#define     DMA_UR1T_VECTOR         50      //0193H
//#define     DMA_UR1R_VECTOR         51      //019BH
//#define     DMA_UR2T_VECTOR         52      //01A3H
//#define     DMA_UR2R_VECTOR         53      //01ABH
//#define     DMA_UR3T_VECTOR         54      //01B3H
//#define     DMA_UR3R_VECTOR         55      //01BBH
//#define     DMA_UR4T_VECTOR         56      //01C3H
//#define     DMA_UR4R_VECTOR         57      //01CBH
//#define     DMA_LCM_VECTOR          58      //01D3H
//#define     LCM_VECTOR              59      //01DBH
//#define     DMA_I2CT_VECTOR         60      //01E3H
//#define     DMA_I2CR_VECTOR         61      //01EBH
//#define     I2S_VECTOR              62      //01F3H
//#define     DMA_I2ST_VECTOR         63      //01FBH
//#define     DMA_I2SR_VECTOR         64      //0203H
