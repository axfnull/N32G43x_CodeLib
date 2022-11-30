/*
 * Copyright (c) 2006-2018, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author            Notes
 * 2018-01-06     sundm75       first version
 */

#include <rtthread.h>
#include <rtdevice.h>
#include <rthw.h>

#include <drv_can.h>
#include "rt_config.h"
#include "n32g43x_can.h"
#include "can.h"
#include "drv_gpio.h"
#include "rthw.h"

#ifdef RT_USING_CAN

CanRxMessage RxMessage;

struct n32g43x_bxcan
{
    CAN_Module *reg;
    void * irq;
};

static rt_err_t bxmodifyfilter(struct n32g43x_bxcan *pbxcan, struct rt_can_filter_item *pitem, rt_uint32_t actived)
{
    CAN_FilterInitType CAN_FilterInitStruct;
        
    CAN_Module* CANx;
    CANx = pbxcan->reg;      
    unsigned char id , idmask, mode; 
    id = pitem->id;
    idmask = pitem->mask;
    mode = (unsigned char) pitem->mode;
    
    CAN_FilterInitStruct.Filter_Num            = CAN_FILTERNUM0;
    CAN_FilterInitStruct.Filter_Mode           = mode;
    CAN_FilterInitStruct.Filter_Scale          = CAN_Filter_32bitScale;
    CAN_FilterInitStruct.Filter_HighId         = id>>16;
    CAN_FilterInitStruct.Filter_LowId          = id&0xFFFF;
    CAN_FilterInitStruct.FilterMask_HighId     = idmask>>16;
    CAN_FilterInitStruct.FilterMask_LowId      = idmask&0xFFFF;
    CAN_FilterInitStruct.Filter_FIFOAssignment = CAN_FIFO0;
    CAN_FilterInitStruct.Filter_Act            = ENABLE;
    if(CANx == CAN1)
    {
        CAN1_InitFilter(&CAN_FilterInitStruct);
    }
    else
    {
        CAN2_InitFilter(&CAN_FilterInitStruct);
    }
    
    return RT_EOK;
}

static rt_err_t setfilter(struct n32g43x_bxcan *pbxcan, struct rt_can_filter_config *pconfig)
{
    struct rt_can_filter_item *pitem = pconfig->items;
    rt_uint32_t count = pconfig->count;
    rt_err_t res;
    while (count)
    {
        res = bxmodifyfilter(pbxcan, pitem, pconfig->actived);
        if (res != RT_EOK)
        {
            return res;
        }
        pitem++;
        count--;
    }
    return RT_EOK;
}

static void bxcan_init(CAN_Module *pcan, rt_uint32_t baud, rt_uint32_t mode)
{
    CAN_InitType       CAN_InitStructure;
    CAN_FilterInitType CAN_FilterInitStructure;
    
    uint32_t bps ;
    
    /* CAN register init */
    CAN_DeInit(pcan);
    
    /* Struct init*/
    CAN_InitStruct(&CAN_InitStructure);
    switch(baud)
    {
        case CAN1MBaud:
            bps = CAN_BAUDRATE_1M;
            break;
        case CAN500kBaud:
            bps = CAN_BAUDRATE_500K;
            break;
        case CAN250kBaud:
            bps = CAN_BAUDRATE_250K;
            break;
        case CAN125kBaud:
            bps = CAN_BAUDRATE_125K;
            break;
        case CAN100kBaud:
            bps = CAN_BAUDRATE_100K;
            break;
        case CAN50kBaud:
            bps = CAN_BAUDRATE_50K;
            break;
        case CAN20kBaud:
            bps = CAN_BAUDRATE_20K;
            break;
        case CAN10kBaud:
            bps = CAN_BAUDRATE_10K;
            break;
        
        default:
            bps = CAN_BAUDRATE_100K;
            break;
    }
    
    CAN_InitStructure.BaudRatePrescaler = (uint32_t)(CAN_BTR_CALCULATE / bps);
    
    switch (mode)
    {
        case RT_CAN_MODE_NORMAL:
            CAN_InitStructure.OperatingMode = CAN_Normal_Mode;
            break;
        case RT_CAN_MODE_LISEN:
            CAN_InitStructure.OperatingMode = CAN_Silent_Mode;
            break;
        case RT_CAN_MODE_LOOPBACK:
            CAN_InitStructure.OperatingMode = CAN_LoopBack_Mode;
            break;
        case RT_CAN_MODE_LOOPBACKANLISEN:
            CAN_InitStructure.OperatingMode = CAN_Silent_LoopBack_Mode;
            break;
        
        default:
            CAN_InitStructure.OperatingMode = CAN_Normal_Mode;
            break;
    }
    
    CAN_InitStructure.TTCM          = DISABLE;
    CAN_InitStructure.ABOM          = DISABLE;
    CAN_InitStructure.AWKUM         = DISABLE;
    CAN_InitStructure.NART          = DISABLE;
    CAN_InitStructure.RFLM          = DISABLE;
    CAN_InitStructure.TXFP          = ENABLE;

    CAN_InitStructure.RSJW          = CAN_RSJW_1tq;
    CAN_InitStructure.TBS1          = CAN_TBS1_3tq;
    CAN_InitStructure.TBS2          = CAN_TBS2_2tq;

    /*Initializes the CAN */
    CAN_Init(pcan, &CAN_InitStructure);
    
     /* CAN1 filter init */
    CAN_FilterInitStructure.Filter_Num            = CAN_FILTERNUM0;
    CAN_FilterInitStructure.Filter_Mode           = CAN_Filter_IdMaskMode;
    CAN_FilterInitStructure.Filter_Scale          = CAN_Filter_32bitScale;
    CAN_FilterInitStructure.Filter_HighId         = 0x8000;
    CAN_FilterInitStructure.Filter_LowId          = 0x0000;
    CAN_FilterInitStructure.FilterMask_HighId     = 0x4000;
    CAN_FilterInitStructure.FilterMask_LowId      = 0x0000;
    CAN_FilterInitStructure.Filter_FIFOAssignment = CAN_FIFO0;
    CAN_FilterInitStructure.Filter_Act            = ENABLE;
    if(pcan == CAN1)
    {
        CAN1_InitFilter(&CAN_FilterInitStructure);
    }
    else
    {
        CAN2_InitFilter(&CAN_FilterInitStructure);
    }
    CAN_INTConfig(pcan, CAN_INT_FMP0, ENABLE);
    CAN_INTConfig(pcan, CAN_INT_TME, ENABLE);
}

/**
 * @brief  Configures the NVIC for CAN.
 */
void CAN1_NVIC_Config(void)
{
    NVIC_InitType NVIC_InitStructure;

    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);

    NVIC_InitStructure.NVIC_IRQChannel                   = USB_LP_CAN1_RX0_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority        = 0x0;
    NVIC_InitStructure.NVIC_IRQChannelCmd                = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
    
    NVIC_InitStructure.NVIC_IRQChannel                   = USB_HP_CAN1_TX_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority        = 0x1;
    NVIC_InitStructure.NVIC_IRQChannelCmd                = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}

void CAN2_NVIC_Config(void)
{
    NVIC_InitType NVIC_InitStructure;

    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);

    NVIC_InitStructure.NVIC_IRQChannel                   = CAN2_RX0_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority        = 0x0;
    NVIC_InitStructure.NVIC_IRQChannelCmd                = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
    
    NVIC_InitStructure.NVIC_IRQChannel                   = CAN2_TX_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority        = 0x0;
    NVIC_InitStructure.NVIC_IRQChannelCmd                = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
    
}

#ifdef RT_USING_CAN1
static void bxcan1_hw_init(void)
{
    /* Enable CAN1 reset state */
    CAN1_NVIC_Config();
    RCC_EnableAPB1PeriphClk(RCC_APB1_PERIPH_CAN1, ENABLE);
    RCC_EnableAPB2PeriphClk(RCC_APB2_PERIPH_AFIO, ENABLE);
    GPIOInit(GPIOD, GPIO_Mode_IPU, GPIO_Speed_50MHz, GPIO_PIN_0);
    GPIOInit(GPIOD, GPIO_Mode_AF_PP, GPIO_Speed_50MHz, GPIO_PIN_1);
    /* Remap CAN1 GPIOs */
    GPIO_ConfigPinRemap(GPIO_RMP3_CAN1, ENABLE);
}
#endif

#ifdef RT_USING_CAN2
static void bxcan2_hw_init(void)
{
    /* Enable CAN2 reset state */
    CAN2_NVIC_Config();
    /* Enable CAN2 reset state */
    RCC_EnableAPB1PeriphClk(RCC_APB1_PERIPH_CAN2, ENABLE);
    RCC_EnableAPB2PeriphClk(RCC_APB2_PERIPH_AFIO, ENABLE);
    GPIOInit(GPIOB, GPIO_Mode_IPU, GPIO_Speed_50MHz, GPIO_PIN_12);
    GPIOInit(GPIOB, GPIO_Mode_AF_PP, GPIO_Speed_50MHz, GPIO_PIN_13);
}
#endif

static rt_err_t configure(struct rt_can_device *can, struct can_configure *cfg)
{
    CAN_Module *pbxcan;

    pbxcan = ((struct n32g43x_bxcan *) can->parent.user_data)->reg;
    if (pbxcan == CAN1)
    {
#ifdef RT_USING_CAN1
        bxcan1_hw_init();  
        bxcan_init(pbxcan, cfg->baud_rate, cfg->mode);
#endif
    }
    else  if (pbxcan == CAN2)
    {
#ifdef RT_USING_CAN2
        bxcan2_hw_init();  
        bxcan_init(pbxcan, cfg->baud_rate, cfg->mode);
#endif
    }
    return RT_EOK;
}

static rt_err_t control(struct rt_can_device *can, int cmd, void *arg)
{
    struct n32g43x_bxcan *pbxcan;

    pbxcan = (struct n32g43x_bxcan *) can->parent.user_data;
    switch (cmd)
    {
        case RT_CAN_CMD_SET_FILTER:
              return setfilter(pbxcan, (struct rt_can_filter_config *) arg);            
        case RT_CAN_CMD_SET_MODE:
            break;
        case RT_CAN_CMD_SET_BAUD:
            break;
        case RT_CAN_CMD_GET_STATUS:
            break;
        default :
            break;
    }
    return RT_EOK;
}



static int sendmsg(struct rt_can_device *can, const void *buf, rt_uint32_t boxno)
{
    CAN_Module *pbxcan;
    CanTxMessage TxMessage;
    struct rt_can_msg *pmsg = (struct rt_can_msg *) buf;
    int i;

    pbxcan = ((struct n32g43x_bxcan *) can->parent.user_data)->reg;

    if(pmsg->ide)
    {
        TxMessage.ExtId = pmsg->id;
        TxMessage.StdId = 0;
    }
    else
    {
        TxMessage.StdId = pmsg->id;
        TxMessage.ExtId = 0;
    }
    
    TxMessage.RTR = pmsg->rtr;
    TxMessage.IDE = pmsg->ide;
    TxMessage.DLC = pmsg->len;
    for( i=0; i<TxMessage.DLC ;i++)
    {
      TxMessage.Data[i] = pmsg->data[i];
    }
    CAN_TransmitMessage(pbxcan, &TxMessage); 

    return RT_EOK;
}

static int recvmsg(struct rt_can_device *can, void *buf, rt_uint32_t boxno)
{    
    struct rt_can_msg *pmsg = (struct rt_can_msg *) buf;
    int i;

    pmsg->ide = (rt_uint32_t) RxMessage.IDE; 
    if(RxMessage.IDE == 1)
        pmsg->id = RxMessage.ExtId;
    else
        pmsg->id = RxMessage.StdId;
    pmsg->len = RxMessage.DLC;
    pmsg->rtr = RxMessage.RTR;
    pmsg->hdr = 0;
    for(i= 0;i< RxMessage.DLC; i++)
    {
      pmsg->data[i] = RxMessage.Data[i];
    }
    

     return RT_EOK;
}

static const struct rt_can_ops canops =
{
    configure,
    control,
    sendmsg,
    recvmsg,
};

#ifdef RT_USING_CAN1

struct rt_can_device bxcan1;

void n32g43x_can1_irqhandler(void *param)  
{    
    CAN_Module* CANx;

    CANx =  CAN1;
   
    /* �����ж� */
    if (CAN_GetIntStatus(CANx, CAN_INT_FMP0)) 
    {
        CAN_ReceiveMessage(CANx, CAN_FIFO0, &RxMessage);
        rt_hw_can_isr(&bxcan1, RT_CAN_EVENT_RX_IND);
        CAN_ClearINTPendingBit(CANx, CAN_INT_FMP0);
        rt_kprintf("\r\nCan1 int RX happened!\r\n");
    }
    /* �����ж� */
    else if (CAN_GetFlagSTS(CANx, CAN_FLAG_RQCPM0)) 
    {
       rt_hw_can_isr(&bxcan1, RT_CAN_EVENT_TX_DONE | 0 << 8);
       CAN_ClearFlag(CANx, CAN_FLAG_RQCPM0);
    }
    /* ��������ж� */
    else if (CAN_GetIntStatus(CANx, CAN_INT_FOV0)) 
    {
       rt_hw_can_isr(&bxcan1, RT_CAN_EVENT_RXOF_IND);
       rt_kprintf("\r\nCan1 int RX OF happened!\r\n");
    }
}

void USB_HP_CAN1_TX_IRQHandler(void)
{
    /* enter interrupt */
    rt_interrupt_enter();

    n32g43x_can1_irqhandler(&bxcan1);
    
    /* leave interrupt */
    rt_interrupt_leave();
}

void USB_LP_CAN1_RX0_IRQHandler(void)
{
    /* enter interrupt */
    rt_interrupt_enter();

    n32g43x_can1_irqhandler(&bxcan1);
    
    /* leave interrupt */
    rt_interrupt_leave();
}


static struct n32g43x_bxcan bxcan1data =
{
    .reg = CAN1,
    .irq = n32g43x_can1_irqhandler,
};
#endif /*RT_USING_CAN1*/

#ifdef RT_USING_CAN2
struct rt_can_device bxcan2;
void n32g43x_can2_irqhandler(void *param)  
{  
    CAN_Module* CANx;

    CANx =  CAN2;
   
    /* �����ж� */
    if (CAN_GetIntStatus(CANx, CAN_INT_FMP0)) 
    {
        CAN_ReceiveMessage(CANx, CAN_FIFO0, &RxMessage);
        rt_hw_can_isr(&bxcan2, RT_CAN_EVENT_RX_IND);
        CAN_ClearINTPendingBit(CANx, CAN_INT_FMP0);
        rt_kprintf("\r\nCan2 int RX happened!\r\n");
    }
    /* �����ж� */
    else if (CAN_GetFlagSTS(CANx, CAN_FLAG_RQCPM0)) 
    {
       rt_hw_can_isr(&bxcan2, RT_CAN_EVENT_TX_DONE | 0 << 8);
       CAN_ClearFlag(CANx, CAN_FLAG_RQCPM0);
    }
    /* ��������ж� */
    else if (CAN_GetIntStatus(CANx, CAN_INT_FOV0)) 
    {
       rt_hw_can_isr(&bxcan2, RT_CAN_EVENT_RXOF_IND);
       rt_kprintf("\r\nCan2 int RX OF happened!\r\n");
    }
}  

void CAN2_TX_IRQHandler(void)
{
    /* enter interrupt */
    rt_interrupt_enter();

    n32g43x_can2_irqhandler(&bxcan2);
    
    /* leave interrupt */
    rt_interrupt_leave();
}

void CAN2_RX0_IRQHandler(void)
{
    /* enter interrupt */
    rt_interrupt_enter();

    n32g43x_can2_irqhandler(&bxcan2);
    
    /* leave interrupt */
    rt_interrupt_leave();
}

static struct n32g43x_bxcan bxcan2data =
{
    .reg = CAN2,
    .irq = n32g43x_can2_irqhandler,
};

#endif /*RT_USING_CAN2*/

int rt_hw_can_init(void)
{

#ifdef RT_USING_CAN1
    bxcan1.config.baud_rate = CAN500kBaud;
    bxcan1.config.msgboxsz = 1;
    bxcan1.config.sndboxnumber = 1;
    bxcan1.config.mode = RT_CAN_MODE_LOOPBACK;
    bxcan1.config.privmode = 0;
    bxcan1.config.ticks = 50;
#ifdef RT_CAN_USING_HDR
    bxcan1.config.maxhdr = 2;
#endif
    rt_hw_can_register(&bxcan1, "bxcan1", &canops, &bxcan1data);
    rt_kprintf("\r\ncan1 register! \r\n");
   
#endif
#ifdef RT_USING_CAN2
    bxcan2.config.baud_rate = CAN500kBaud;
    bxcan2.config.msgboxsz = 1;
    bxcan2.config.sndboxnumber = 1;
    bxcan2.config.OperatingMode = RT_CAN_MODE_LOOPBACK;
    bxcan2.config.privmode = 0;
    bxcan2.config.ticks = 50;
#ifdef RT_CAN_USING_HDR
    bxcan2.config.maxhdr = 2;
#endif
    rt_hw_can_register(&bxcan2, "bxcan2", &canops, &bxcan2data);
    rt_kprintf("\r\ncan2 register! \r\n");

#endif
    return RT_EOK;
}

INIT_BOARD_EXPORT(rt_hw_can_init);

#endif /*RT_USING_CAN*/
