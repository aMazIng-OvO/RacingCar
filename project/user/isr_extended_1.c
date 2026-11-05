/*********************************************************************************************************************
* STC32G144K 扩展中断服务程序（二）
*
* 覆盖向量：PA/PB（90/91）、DMA UART5~8 接收（83/85/87/89）、
*           TM5_TM6（96）、TM7_TM8（97）、TM9_TM10（98）、TM17_TM18（99）
*
* C251 的 C 中断属性只支持编号 0~31，芯片的扩展中断向量号超出了该范围，
* 所以这里改用 0~31 的局部编号，并配合 isr_extended_vectors.A51 中的跳转表，
* 由绝对地址段把中断入口放到芯片的真实向量地址上（#pragma noiv 关闭编译器自动生成的向量表）。
* 中断服务函数的内容与原 isr.c 完全一致，仅中断编号方式不同。
********************************************************************************************************************/
#include "zf_common_headfile.h"

#pragma noiv

void PA_IRQHandler() interrupt 21
{
	uint8 i;
	volatile uint8 far *intf_reg = (volatile uint8 far *)(0x7efd10 + 0x0a);//保存中断端口
	uint8 flags = *intf_reg;
	*intf_reg &= ~flags;

	for(i=0; i<8; i++)//查找中断
    {
        if(flags & (0x01 << i))
        {
            gpio_int_irq_handlers[0x0a][i]();
        }
    }
}

void PB_IRQHandler() interrupt 22
{
	uint8 i;
	volatile uint8 far *intf_reg = (volatile uint8 far *)(0x7efd10 + 0x0b);//保存中断端口
	uint8 flags = *intf_reg;
	*intf_reg &= ~flags;

	for(i=0; i<8; i++)//查找中断
    {
        if(flags & (0x01 << i))
        {
            gpio_int_irq_handlers[0x0b][i]();
        }
    }
}

void DMA_UART5_IRQHandler(void) interrupt 14
{
    if (DMA_UR5R_STA & 0x01) // 接收完成
    {
        DMA_UR5R_STA &= ~0x01;      // 清标志位
        uart_rx_start_buff(UART_5); // 设置下一次接收，务必保留

        if (uart_rx_handlers[UART_5] != NULL)
        {
            uart_rx_handlers[UART_5](uart_rx_buff[UART_5][0]);
        }
    }

    if (DMA_UR5R_STA & 0x02) // 数据丢弃
    {
        DMA_UR5R_STA &= ~0x02;      // 清标志位
        uart_rx_start_buff(UART_5); // 设置下一次接收，务必保留
        // 如果进入了这个中断，则代表UART的数据在没有取走之前被覆盖!
        // 如果进入了这个中断，则代表UART的数据在没有取走之前被覆盖!
        // 如果进入了这个中断，则代表UART的数据在没有取走之前被覆盖!
    }
}

void DMA_UART6_IRQHandler(void) interrupt 16
{
    if (DMA_UR6R_STA & 0x01) // 接收完成
    {
        DMA_UR6R_STA &= ~0x01;      // 清标志位
        uart_rx_start_buff(UART_6); // 设置下一次接收，务必保留

        if (uart_rx_handlers[UART_6] != NULL)
        {
            uart_rx_handlers[UART_6](uart_rx_buff[UART_6][0]);
        }
    }

    if (DMA_UR6R_STA & 0x02) // 数据丢弃
    {
        DMA_UR6R_STA &= ~0x02;      // 清标志位
        uart_rx_start_buff(UART_6); // 设置下一次接收，务必保留
        // 如果进入了这个中断，则代表UART的数据在没有取走之前被覆盖!
        // 如果进入了这个中断，则代表UART的数据在没有取走之前被覆盖!
        // 如果进入了这个中断，则代表UART的数据在没有取走之前被覆盖!
    }
}

void DMA_UART7_IRQHandler(void) interrupt 18
{
    if (DMA_UR7R_STA & 0x01) // 接收完成
    {
        DMA_UR7R_STA &= ~0x01;      // 清标志位
        uart_rx_start_buff(UART_7); // 设置下一次接收，务必保留

        if (uart_rx_handlers[UART_7] != NULL)
        {
            uart_rx_handlers[UART_7](uart_rx_buff[UART_7][0]);
        }
    }

    if (DMA_UR7R_STA & 0x02) // 数据丢弃
    {
        DMA_UR7R_STA &= ~0x02;      // 清标志位
        uart_rx_start_buff(UART_7); // 设置下一次接收，务必保留
        // 如果进入了这个中断，则代表UART的数据在没有取走之前被覆盖!
        // 如果进入了这个中断，则代表UART的数据在没有取走之前被覆盖!
        // 如果进入了这个中断，则代表UART的数据在没有取走之前被覆盖!
    }
}

void DMA_UART8_IRQHandler(void) interrupt 20
{
    if (DMA_UR8R_STA & 0x01) // 接收完成
    {
        DMA_UR8R_STA &= ~0x01;      // 清标志位
        uart_rx_start_buff(UART_8); // 设置下一次接收，务必保留

        if (uart_rx_handlers[UART_8] != NULL)
        {
            uart_rx_handlers[UART_8](uart_rx_buff[UART_8][0]);
        }
    }

    if (DMA_UR8R_STA & 0x02) // 数据丢弃
    {
        DMA_UR8R_STA &= ~0x02;      // 清标志位
        uart_rx_start_buff(UART_8); // 设置下一次接收，务必保留
        // 如果进入了这个中断，则代表UART的数据在没有取走之前被覆盖!
        // 如果进入了这个中断，则代表UART的数据在没有取走之前被覆盖!
        // 如果进入了这个中断，则代表UART的数据在没有取走之前被覆盖!
    }
}

void TM5_TM6_IRQHandler() interrupt 27
{
    if(TIM5_GET_FLAG)
    {
        TIM5_CLEAR_FLAG;
        if (tim_irq_handlers[TIM_5] != NULL)
        {
            tim_irq_handlers[TIM_5]();
        }
    }
    
    if(TIM6_GET_FLAG)
    {
        TIM6_CLEAR_FLAG;
        if (tim_irq_handlers[TIM_6] != NULL)
        {
            tim_irq_handlers[TIM_6]();
        }
    }
}

void TM7_IRQHandler() interrupt 28
{
    if(TIM7_GET_FLAG)
    {
        TIM7_CLEAR_FLAG;
        if (tim_irq_handlers[TIM_7] != NULL)
        {
            tim_irq_handlers[TIM_7]();
        }
    }
    
    if(TIM8_GET_FLAG)
    {
        TIM8_CLEAR_FLAG;
        if (tim_irq_handlers[TIM_8] != NULL)
        {
            tim_irq_handlers[TIM_8]();
        }
    }
}

void TM9_TM10_IRQHandler() interrupt 29
{
    if(TIM9_GET_FLAG)
    {
        TIM9_CLEAR_FLAG;
        if (tim_irq_handlers[TIM_9] != NULL)
        {
            tim_irq_handlers[TIM_9]();
        }
    }
    
    if(TIM10_GET_FLAG)
    {
        TIM10_CLEAR_FLAG;
        if (tim_irq_handlers[TIM_10] != NULL)
        {
            tim_irq_handlers[TIM_10]();
        }
    }
}

void TM17_TM18_IRQHandler() interrupt 30
{
    if(TIM17_GET_FLAG)
    {
        TIM17_CLEAR_FLAG;
        if (tim_irq_handlers[TIM_17] != NULL)
        {
            tim_irq_handlers[TIM_17]();
        }
    }
    
    if(TIM18_GET_FLAG)
    {
        TIM18_CLEAR_FLAG;
        if (tim_irq_handlers[TIM_18] != NULL)
        {
            tim_irq_handlers[TIM_18]();
        }
    }
}
