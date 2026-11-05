/*********************************************************************************************************************
* STC32G144K 扩展中断服务程序（一）
*
* 覆盖向量：P0~P9（37~46）、DMA UART1~4 接收（51/53/55/57）、DMA_LCM（58）、TM11（67）
*
* C251 的 C 中断属性只支持编号 0~31，芯片的扩展中断向量号超出了该范围，
* 所以这里改用 0~31 的局部编号，并配合 isr_extended_vectors.A51 中的跳转表，
* 由绝对地址段把中断入口放到芯片的真实向量地址上（#pragma noiv 关闭编译器自动生成的向量表）。
* 中断服务函数的内容与原 isr.c 完全一致，仅中断编号方式不同。
********************************************************************************************************************/
#include "zf_common_headfile.h"

#pragma noiv

void P0_IRQHandler() interrupt 0
{
	uint8 i;
	volatile uint8 far *intf_reg = (volatile uint8 far *)(0x7efd10 + 0x00);//保存中断端口
	uint8 flags = *intf_reg;
	*intf_reg &= ~flags;

	for(i=0; i<8; i++)//查找中断
    {
        if(flags & (0x01 << i))
        {
            gpio_int_irq_handlers[0x01][i]();
        }
    }
}

void P1_IRQHandler() interrupt 1
{
	uint8 i;
	volatile uint8 far *intf_reg = (volatile uint8 far *)(0x7efd10 + 0x01);//保存中断端口
	uint8 flags = *intf_reg;
	*intf_reg &= ~flags;

	for(i=0; i<8; i++)//查找中断
    {
        if(flags & (0x01 << i))
        {
            gpio_int_irq_handlers[0x01][i]();
        }
    }
}

void P2_IRQHandler() interrupt 2
{
	uint8 i;
	volatile uint8 far *intf_reg = (volatile uint8 far *)(0x7efd10 + 0x02);//保存中断端口
	uint8 flags = *intf_reg;
	*intf_reg &= ~flags;

	for(i=0; i<8; i++)//查找中断
    {
        if(flags & (0x01 << i))
        {
            gpio_int_irq_handlers[0x02][i]();
        }
    }
}

void P3_IRQHandler() interrupt 3
{
	uint8 i;
	volatile uint8 far *intf_reg = (volatile uint8 far *)(0x7efd10 + 0x03);//保存中断端口
	uint8 flags = *intf_reg;
	*intf_reg &= ~flags;

	for(i=0; i<8; i++)//查找中断
    {
        if(flags & (0x01 << i))
        {
            gpio_int_irq_handlers[0x03][i]();
        }
    }
}

void P4_IRQHandler() interrupt 4
{
	uint8 i;
	volatile uint8 far *intf_reg = (volatile uint8 far *)(0x7efd10 + 0x04);//保存中断端口
	uint8 flags = *intf_reg;
	*intf_reg &= ~flags;

	for(i=0; i<8; i++)//查找中断
    {
        if(flags & (0x01 << i))
        {
            gpio_int_irq_handlers[0x04][i]();
        }
    }
}

void P5_IRQHandler() interrupt 5
{
	uint8 i;
	volatile uint8 far *intf_reg = (volatile uint8 far *)(0x7efd10 + 0x05);//保存中断端口
	uint8 flags = *intf_reg;
	*intf_reg &= ~flags;

	for(i=0; i<8; i++)//查找中断
    {
        if(flags & (0x01 << i))
        {
            gpio_int_irq_handlers[0x05][i]();
        }
    }
}

void P6_IRQHandler() interrupt 6
{
	uint8 i;
	volatile uint8 far *intf_reg = (volatile uint8 far *)(0x7efd10 + 0x06);//保存中断端口
	uint8 flags = *intf_reg;
	*intf_reg &= ~flags;

	for(i=0; i<8; i++)//查找中断
    {
        if(flags & (0x01 << i))
        {
            gpio_int_irq_handlers[0x06][i]();
        }
    }
}

void P7_IRQHandler() interrupt 7
{
	uint8 i;
	volatile uint8 far *intf_reg = (volatile uint8 far *)(0x7efd10 + 0x07);//保存中断端口
	uint8 flags = *intf_reg;
	*intf_reg &= ~flags;

	for(i=0; i<8; i++)//查找中断
    {
        if(flags & (0x01 << i))
        {
            gpio_int_irq_handlers[0x07][i]();
        }
    }
}

void P8_IRQHandler() interrupt 8
{
	uint8 i;
	volatile uint8 far *intf_reg = (volatile uint8 far *)(0x7efd10 + 0x08);//保存中断端口
	uint8 flags = *intf_reg;
	*intf_reg &= ~flags;

	for(i=0; i<8; i++)//查找中断
    {
        if(flags & (0x01 << i))
        {
            gpio_int_irq_handlers[0x08][i]();
        }
    }
}

void P9_IRQHandler() interrupt 9
{
	uint8 i;
	volatile uint8 far *intf_reg = (volatile uint8 far *)(0x7efd10 + 0x09);//保存中断端口
	uint8 flags = *intf_reg;
	*intf_reg &= ~flags;

	for(i=0; i<8; i++)//查找中断
    {
        if(flags & (0x01 << i))
        {
            gpio_int_irq_handlers[0x09][i]();
        }
    }
}

void DMA_UART1_IRQHandler(void) interrupt 14
{
    static vuint8 dwon_count = 0;

    if (DMA_UR1R_STA & 0x01) // 接收完成
    {
        DMA_UR1R_STA &= ~0x01;      // 清标志位
        uart_rx_start_buff(UART_1); // 设置下一次接收，务必保留

        // 程序自动下载
        if (uart_rx_buff[UART_1][0] == 0x7F)
        {
            if (dwon_count++ > 20)
            {
                IAP_CONTR = 0x60;
            }
        }
        else
        {
            dwon_count = 0;
        }

        if (uart_rx_handlers[UART_1] != NULL)
        {
            uart_rx_handlers[UART_1](uart_rx_buff[UART_1][0]);
        }
    }

    if (DMA_UR1R_STA & 0x02) // 数据丢弃
    {
        DMA_UR1R_STA &= ~0x02;      // 清标志位
        uart_rx_start_buff(UART_1); // 设置下一次接收，务必保留
        // 如果进入了这个中断，则代表UART的数据在没有取走之前被覆盖!
        // 如果进入了这个中断，则代表UART的数据在没有取走之前被覆盖!
        // 如果进入了这个中断，则代表UART的数据在没有取走之前被覆盖!
    }
}

void DMA_UART2_IRQHandler(void) interrupt 16
{
    if (DMA_UR2R_STA & 0x01) // 接收完成
    {
        DMA_UR2R_STA &= ~0x01;      // 清标志位
        uart_rx_start_buff(UART_2); // 设置下一次接收，务必保留

        if (uart_rx_handlers[UART_2] != NULL)
        {
            uart_rx_handlers[UART_2](uart_rx_buff[UART_2][0]);
        }
    }

    if (DMA_UR2R_STA & 0x02) // 数据丢弃
    {
        DMA_UR2R_STA &= ~0x02;      // 清标志位
        uart_rx_start_buff(UART_2); // 设置下一次接收，务必保留
        // 如果进入了这个中断，则代表UART的数据在没有取走之前被覆盖!
        // 如果进入了这个中断，则代表UART的数据在没有取走之前被覆盖!
        // 如果进入了这个中断，则代表UART的数据在没有取走之前被覆盖!
    }
}

void DMA_UART3_IRQHandler(void) interrupt 18
{
    if (DMA_UR3R_STA & 0x01) // 接收完成
    {
        DMA_UR3R_STA &= ~0x01;      // 清标志位
        uart_rx_start_buff(UART_3); // 设置下一次接收，务必保留

        if (uart_rx_handlers[UART_3] != NULL)
        {
            uart_rx_handlers[UART_3](uart_rx_buff[UART_3][0]);
        }
    }

    if (DMA_UR3R_STA & 0x02) // 数据丢弃
    {
        DMA_UR3R_STA &= ~0x02;      // 清标志位
        uart_rx_start_buff(UART_3); // 设置下一次接收，务必保留
        // 如果进入了这个中断，则代表UART的数据在没有取走之前被覆盖!
        // 如果进入了这个中断，则代表UART的数据在没有取走之前被覆盖!
        // 如果进入了这个中断，则代表UART的数据在没有取走之前被覆盖!
    }
}

void DMA_UART4_IRQHandler(void) interrupt 20
{
    if (DMA_UR4R_STA & 0x01) // 接收完成
    {
        DMA_UR4R_STA &= ~0x01;      // 清标志位
        uart_rx_start_buff(UART_4); // 设置下一次接收，务必保留

        if (uart_rx_handlers[UART_4] != NULL)
        {
            uart_rx_handlers[UART_4](uart_rx_buff[UART_4][0]);
        }
    }

    if (DMA_UR4R_STA & 0x02) // 数据丢弃
    {
        DMA_UR4R_STA &= ~0x02;      // 清标志位
        uart_rx_start_buff(UART_4); // 设置下一次接收，务必保留
        // 如果进入了这个中断，则代表UART的数据在没有取走之前被覆盖!
        // 如果进入了这个中断，则代表UART的数据在没有取走之前被覆盖!
        // 如果进入了这个中断，则代表UART的数据在没有取走之前被覆盖!
    }
}

void DMA_LCM_IRQHandler() interrupt 21
{
    mt9v03x_dma_handler();
    // 一帧采集完成：立刻把采集缓冲搬进稳定快照，之后算法只读快照，避免 DMA 写下一帧时撕裂
    mt9v03x_frame_snapshot();
}

void TM11_IRQHandler() interrupt 30
{
    TIM11_CLEAR_FLAG;

    if (tim_irq_handlers[TIM_11] != NULL)
    {
        tim_irq_handlers[TIM_11]();
    }
}
