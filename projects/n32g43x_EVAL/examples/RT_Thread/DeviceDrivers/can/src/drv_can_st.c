/*
 * Copyright (c) 2006-2018, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2018-08-05     Xeon Xu      the first version
 * 2019-01-22     YLZ          port from stm324xx-HAL to bsp stm3210x-HAL
 * 2019-02-19     YLZ          add support EXTID RTR Frame. modify send, recv functions.
 *                             fix bug.port to BSP [stm32]
 * 2019-03-27     YLZ          support double can channels, support stm32F4xx (only Legacy mode).
 * 2019-06-17     YLZ          port to new STM32F1xx HAL V1.1.3.
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

/* attention !!! baud calculation example: Tclk / ((ss + bs1 + bs2) * brp)  36 / ((1 + 8 + 3) * 3) = 1MHz*/

const struct n32g43x_baud_rate_tab can_baud_rate_tab[] =
{
    {CAN1MBaud, (CAN_RSJW_2tq | CAN_TBS1_8tq  | CAN_TBS2_3tq | 3)},
    {CAN500kBaud, (CAN_RSJW_2tq | CAN_TBS1_8tq  | CAN_TBS2_3tq | 6)},
    {CAN250kBaud, (CAN_RSJW_2tq | CAN_TBS1_8tq  | CAN_TBS2_3tq | 12)},
    {CAN125kBaud, (CAN_RSJW_2tq | CAN_TBS1_8tq  | CAN_TBS2_3tq | 24)},
    {CAN100kBaud, (CAN_RSJW_2tq | CAN_TBS1_8tq  | CAN_TBS2_3tq | 30)},
    {CAN50kBaud, (CAN_RSJW_2tq | CAN_TBS1_8tq  | CAN_TBS2_3tq | 60)},
    {CAN20kBaud, (CAN_RSJW_2tq | CAN_TBS1_8tq  | CAN_TBS2_3tq | 150)},
    {CAN10kBaud, (CAN_RSJW_2tq | CAN_TBS1_8tq  | CAN_TBS2_3tq | 300)}
};

#define BAUD_DATA(TYPE,NO)       ((can_baud_rate_tab[NO].config_data & TYPE##MASK))

#ifdef RT_USING_CAN1
static struct n32g43x_can drv_can1 =
{
    .name = "can1",
    .CanHandle.Instance = CAN1,
};
#endif

#ifdef RT_USING_CAN2
static struct n32g43x_can drv_can2 =
{
    "can2",
    .CanHandle.Instance = CAN2,
};
#endif

static rt_uint32_t get_can_baud_index(rt_uint32_t baud)
{
    rt_uint32_t len, index;

    len = sizeof(can_baud_rate_tab) / sizeof(can_baud_rate_tab[0]);
    for (index = 0; index < len; index++)
    {
        if (can_baud_rate_tab[index].baud_rate == baud)
            return index;
    }

    return 0; /* default baud is CAN1MBaud */
}

static rt_err_t _can_config(struct rt_can_device *can, struct can_configure *cfg)
{
    struct n32g43x_can *drv_can;
    rt_uint32_t baud_index;

    RT_ASSERT(can);
    RT_ASSERT(cfg);
    drv_can = (struct n32g43x_can *)can->parent.user_data;
    RT_ASSERT(drv_can);

    drv_can->CanHandle.Init.TTCM = DISABLE;
    drv_can->CanHandle.Init.ABOM = ENABLE;
    drv_can->CanHandle.Init.AWKUM = DISABLE;
    drv_can->CanHandle.Init.NART = DISABLE;
    drv_can->CanHandle.Init.RFLM = DISABLE;
    drv_can->CanHandle.Init.TXFP = ENABLE;

    switch (cfg->mode)
    {
    case RT_CAN_MODE_NORMAL:
        drv_can->CanHandle.Init.OperatingMode = CAN_Normal_Mode;
        break;
    case RT_CAN_MODE_LISEN:
        drv_can->CanHandle.Init.OperatingMode = CAN_Silent_Mode;
        break;
    case RT_CAN_MODE_LOOPBACK:
        drv_can->CanHandle.Init.OperatingMode = CAN_LoopBack_Mode;
        break;
    case RT_CAN_MODE_LOOPBACKANLISEN:
        drv_can->CanHandle.Init.OperatingMode = CAN_Silent_LoopBack_Mode;
        break;
    }

    baud_index = get_can_baud_index(cfg->baud_rate);
    drv_can->CanHandle.Init.RSJW = BAUD_DATA(RSJW, baud_index);
    drv_can->CanHandle.Init.TBS1 = BAUD_DATA(TBS1, baud_index);
    drv_can->CanHandle.Init.TBS2 = BAUD_DATA(TBS2, baud_index);
    drv_can->CanHandle.Init.BaudRatePrescaler = BAUD_DATA(RRESCL, baud_index);
    /* init can */
    if (CAN_Init(&drv_can->CanHandle.Instance, &drv_can->CanHandle.Init) != CAN_InitSTS_Success)
    {
        return -RT_ERROR;
    }

    /* default filter config */
    HAL_CAN_ConfigFilter(&drv_can->CanHandle, &drv_can->FilterConfig);
    /* can start */
    HAL_CAN_Start(&drv_can->CanHandle);

    return RT_EOK;
}

/**
 * @brief  Configures the NVIC for CAN.
 */
void CAN_NVIC_Config(IRQn_Type IRQn, uint8_t PreemptionPriority, uint8_t SubPriority,FunctionalState cmd)
{
    NVIC_InitType NVIC_InitStructure;

    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);

    NVIC_InitStructure.NVIC_IRQChannel                   = IRQn;
    NVIC_InitStructure.NVIC_IRQChannelCmd                = cmd;
    if(cmd)
    {
        NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = PreemptionPriority;
        NVIC_InitStructure.NVIC_IRQChannelSubPriority        = SubPriority;
    }
    NVIC_Init(&NVIC_InitStructure);
}

static rt_err_t _can_control(struct rt_can_device *can, int cmd, void *arg)
{
    rt_uint32_t argval;
    struct n32g43x_can *drv_can;
    struct rt_can_filter_config *filter_cfg;

    RT_ASSERT(can != RT_NULL);
    drv_can = (struct n32g43x_can *)can->parent.user_data;
    RT_ASSERT(drv_can != RT_NULL);

    switch (cmd)
    {
    case RT_DEVICE_CTRL_CLR_INT:
        argval = (rt_uint32_t) arg;
        if (argval == RT_DEVICE_FLAG_INT_RX)
        {
            if (CAN1 == drv_can->CanHandle.Instance)
            {
                CAN_NVIC_Config(USB_LP_CAN1_RX0_IRQn, 1, 0, ENABLE);
                CAN_NVIC_Config(CAN1_RX1_IRQn, 1, 0, ENABLE);
            }
#ifdef CAN2
            if (CAN2 == drv_can->CanHandle.Instance)
            {
                CAN_NVIC_Config(CAN2_RX0_IRQn, 0, 0, DISABLE);
                CAN_NVIC_Config(CAN2_RX1_IRQn, 0, 0, DISABLE);
            }
#endif
            CAN_INTConfig(&drv_can->CanHandle, CAN_INT_FMP0, DISABLE);
            CAN_INTConfig(&drv_can->CanHandle, CAN_INT_FF0, DISABLE);
            CAN_INTConfig(&drv_can->CanHandle, CAN_INT_FOV0, DISABLE);
            CAN_INTConfig(&drv_can->CanHandle, CAN_INT_FMP1, DISABLE);
            CAN_INTConfig(&drv_can->CanHandle, CAN_INT_FF1, DISABLE);
            CAN_INTConfig(&drv_can->CanHandle, CAN_INT_FOV1, DISABLE);
        }
        else if (argval == RT_DEVICE_FLAG_INT_TX)
        {
            if (CAN1 == drv_can->CanHandle.Instance)
            {
                CAN_NVIC_Config(USB_HP_CAN1_TX_IRQn, 0, 0, DISABLE);
            }
#ifdef CAN2
            if (CAN2 == drv_can->CanHandle.Instance)
            {
                CAN_NVIC_Config(CAN2_TX_IRQn, 0, 0, DISABLE);
            }
#endif
            CAN_INTConfig(&drv_can->CanHandle, CAN_INT_TME, DISABLE);
        }
        else if (argval == RT_DEVICE_CAN_INT_ERR)
        {
            if (CAN1 == drv_can->CanHandle.Instance)
            {
                CAN_NVIC_Config(CAN1_SCE_IRQn, 0, 0, DISABLE);
                
            }
#ifdef CAN2
            if (CAN2 == drv_can->CanHandle.Instance)
            {
                CAN_NVIC_Config(CAN2_SCE_IRQn, 0, 0, DISABLE);
            }
#endif
            CAN_ClearFlag(&drv_can->CanHandle, CAN_FLAG_EWGFL);
            CAN_ClearFlag(&drv_can->CanHandle, CAN_FLAG_EPVFL);
            CAN_ClearFlag(&drv_can->CanHandle, CAN_FLAG_BOFFL);
            CAN_ClearFlag(&drv_can->CanHandle, CAN_FLAG_LEC);
            CAN_ClearINTPendingBit(&drv_can->CanHandle, CAN_INT_ERR);
        }
        break;
    case RT_DEVICE_CTRL_SET_INT:
        argval = (rt_uint32_t) arg;
        if (argval == RT_DEVICE_FLAG_INT_RX)
        {
            CAN_INTConfig(&drv_can->CanHandle, CAN_INT_FMP0, ENABLE);
            CAN_INTConfig(&drv_can->CanHandle, CAN_INT_FF0, ENABLE);
            CAN_INTConfig(&drv_can->CanHandle, CAN_INT_FOV0, ENABLE);
            CAN_INTConfig(&drv_can->CanHandle, CAN_INT_FMP1, ENABLE);
            CAN_INTConfig(&drv_can->CanHandle, CAN_INT_FF1, ENABLE);
            CAN_INTConfig(&drv_can->CanHandle, CAN_INT_FOV1, ENABLE);

            if (CAN1 == drv_can->CanHandle.Instance)
            {
                CAN_NVIC_Config(USB_LP_CAN1_RX0_IRQn, 1, 0, ENABLE);
                CAN_NVIC_Config(CAN1_RX1_IRQn, 1, 0, ENABLE);
            }
#ifdef CAN2
            if (CAN2 == drv_can->CanHandle.Instance)
            {
                HAL_NVIC_SetPriority(CAN2_RX0_IRQn, 1, 0);
                HAL_NVIC_EnableIRQ(CAN2_RX0_IRQn);
                HAL_NVIC_SetPriority(CAN2_RX1_IRQn, 1, 0);
                HAL_NVIC_EnableIRQ(CAN2_RX1_IRQn);
            }
#endif
        }
        else if (argval == RT_DEVICE_FLAG_INT_TX)
        {
            CAN_INTConfig(&drv_can->CanHandle, CAN_INT_TME, ENABLE);

            if (CAN1 == drv_can->CanHandle.Instance)
            {
                CAN_NVIC_Config(USB_HP_CAN1_TX_IRQn, 1, 0, ENABLE);
            }
#ifdef CAN2
            if (CAN2 == drv_can->CanHandle.Instance)
            {
                CAN_NVIC_Config(CAN2_TX_IRQn, 1, 0, ENABLE);
            }
#endif
        }
        else if (argval == RT_DEVICE_CAN_INT_ERR)
        {
            CAN_INTConfig(&drv_can->CanHandle, CAN_INT_EWG, ENABLE);
            CAN_INTConfig(&drv_can->CanHandle, CAN_INT_EPV, ENABLE);
            CAN_INTConfig(&drv_can->CanHandle, CAN_INT_BOF, ENABLE);
            CAN_INTConfig(&drv_can->CanHandle, CAN_INT_LEC, ENABLE);
            CAN_INTConfig(&drv_can->CanHandle, CAN_INT_ERR, ENABLE);

            if (CAN1 == drv_can->CanHandle.Instance)
            {
                HAL_NVIC_SetPriority(CAN1_SCE_IRQn, 1, 0);
                HAL_NVIC_EnableIRQ(CAN1_SCE_IRQn);
            }
#ifdef CAN2
            if (CAN2 == drv_can->CanHandle.Instance)
            {
                HAL_NVIC_SetPriority(CAN2_SCE_IRQn, 1, 0);
                HAL_NVIC_EnableIRQ(CAN2_SCE_IRQn);
            }
#endif
        }
        break;
    case RT_CAN_CMD_SET_FILTER:
        if (RT_NULL == arg)
        {
            /* default filter config */
            HAL_CAN_ConfigFilter(&drv_can->CanHandle, &drv_can->FilterConfig);
        }
        else
        {
            filter_cfg = (struct rt_can_filter_config *)arg;
            /* get default filter */
            for (int i = 0; i < filter_cfg->count; i++)
            {
                drv_can->FilterConfig.Filter_Num = filter_cfg->items[i].hdr;
                drv_can->FilterConfig.Filter_HighId = (filter_cfg->items[i].id >> 13) & 0xFFFF;
                drv_can->FilterConfig.Filter_LowId = ((filter_cfg->items[i].id << 3) | 
                                                    (filter_cfg->items[i].ide << 2) | 
                                                    (filter_cfg->items[i].rtr << 1)) & 0xFFFF;
                drv_can->FilterConfig.FilterMask_HighId = (filter_cfg->items[i].mask >> 16) & 0xFFFF;
                drv_can->FilterConfig.FilterMask_LowId = filter_cfg->items[i].mask & 0xFFFF;
                drv_can->FilterConfig.Filter_Mode = filter_cfg->items[i].mode;
                /* Filter conf */
                HAL_CAN_ConfigFilter(&drv_can->CanHandle, &drv_can->FilterConfig);
            }
        }
        break;
    case RT_CAN_CMD_SET_MODE:
        argval = (rt_uint32_t) arg;
        if (argval != RT_CAN_MODE_NORMAL &&
                argval != RT_CAN_MODE_LISEN &&
                argval != RT_CAN_MODE_LOOPBACK &&
                argval != RT_CAN_MODE_LOOPBACKANLISEN)
        {
            return -RT_ERROR;
        }
        if (argval != drv_can->device.config.mode)
        {
            drv_can->device.config.mode = argval;
            return _can_config(&drv_can->device, &drv_can->device.config);
        }
        break;
    case RT_CAN_CMD_SET_BAUD:
        argval = (rt_uint32_t) arg;
        if (argval != CAN1MBaud &&
                argval != CAN500kBaud &&
                argval != CAN250kBaud &&
                argval != CAN125kBaud &&
                argval != CAN100kBaud &&
                argval != CAN50kBaud  &&
                argval != CAN20kBaud  &&
                argval != CAN10kBaud)
        {
            return -RT_ERROR;
        }
        if (argval != drv_can->device.config.baud_rate)
        {
            drv_can->device.config.baud_rate = argval;
            return _can_config(&drv_can->device, &drv_can->device.config);
        }
        break;
    case RT_CAN_CMD_SET_PRIV:
        argval = (rt_uint32_t) arg;
        if (argval != RT_CAN_MODE_PRIV &&
                argval != RT_CAN_MODE_NOPRIV)
        {
            return -RT_ERROR;
        }
        if (argval != drv_can->device.config.privmode)
        {
            drv_can->device.config.privmode = argval;
            return _can_config(&drv_can->device, &drv_can->device.config);
        }
        break;
    case RT_CAN_CMD_GET_STATUS:
    {
        rt_uint32_t errtype;
        errtype = drv_can->CanHandle.Instance->ESTS;
        drv_can->device.status.rcverrcnt = errtype >> 24;
        drv_can->device.status.snderrcnt = (errtype >> 16 & 0xFF);
        drv_can->device.status.lasterrtype = errtype & 0x70;
        drv_can->device.status.errcode = errtype & 0x07;

        rt_memcpy(arg, &drv_can->device.status, sizeof(drv_can->device.status));
    }
    break;
    }

    return RT_EOK;
}

static int _can_sendmsg(struct rt_can_device *can, const void *buf, rt_uint32_t box_num)
{
    CAN_HandleTypeDef *hcan;
    hcan = &((struct n32g43x_can *) can->parent.user_data)->CanHandle;
    struct rt_can_msg *pmsg = (struct rt_can_msg *) buf;
    CanTxMessage txheader = {0};
    uint8_t state = hcan->State;

    /* Check the parameters */
    RT_ASSERT(IS_CAN_DLC(pmsg->len));

    if ((state == HAL_CAN_STATE_READY) ||
            (state == HAL_CAN_STATE_LISTENING))
    {
        /*check select mailbox  is empty */
        switch (1 << box_num)
        {
        case CAN_TX_MAILBOX0:
            if (HAL_IS_BIT_SET(hcan->Instance->TSTS, CAN_TSTS_TMEM0) != SET)
            {
                /* Change CAN state */
                hcan->State = HAL_CAN_STATE_ERROR;
                /* Return function status */
                return -RT_ERROR;
            }
            break;
        case CAN_TX_MAILBOX1:
            if (HAL_IS_BIT_SET(hcan->Instance->TSTS, CAN_TSTS_TMEM1) != SET)
            {
                /* Change CAN state */
                hcan->State = HAL_CAN_STATE_ERROR;
                /* Return function status */
                return -RT_ERROR;
            }
            break;
        case CAN_TX_MAILBOX2:
            if (HAL_IS_BIT_SET(hcan->Instance->TSTS, CAN_TSTS_TMEM2) != SET)
            {
                /* Change CAN state */
                hcan->State = HAL_CAN_STATE_ERROR;
                /* Return function status */
                return -RT_ERROR;
            }
            break;
        default:
            RT_ASSERT(0);
            break;
        }

        if (RT_CAN_STDID == pmsg->ide)
        {
            txheader.IDE = CAN_ID_STD;
            RT_ASSERT(IS_CAN_STDID(pmsg->id));
            txheader.StdId = pmsg->id;
        }
        else
        {
            txheader.IDE = CAN_ID_EXT;
            RT_ASSERT(IS_CAN_EXTID(pmsg->id));
            txheader.ExtId = pmsg->id;
        }

        if (RT_CAN_DTR == pmsg->rtr)
        {
            txheader.RTR = CAN_RTRQ_DATA;
        }
        else
        {
            txheader.RTR = CAN_RTRQ_REMOTE;
        }
        /* clear TIR */
        hcan->Instance->sTxMailBox[box_num].TMI &= CAN_TMI0_TXRQ;
        /* Set up the Id */
        if (RT_CAN_STDID == pmsg->ide)
        {
            hcan->Instance->sTxMailBox[box_num].TMI |= (txheader.StdId << 21) | txheader.RTR;
        }
        else
        {
            hcan->Instance->sTxMailBox[box_num].TMI |= (txheader.ExtId << 3) | txheader.IDE | txheader.RTR;
        }
        /* Set up the DLC */
        hcan->Instance->sTxMailBox[box_num].TMDT = pmsg->len & 0x0FU;
        /* Set up the data field */
        WRITE_REG(hcan->Instance->sTxMailBox[box_num].TMDH,
                  ((uint32_t)pmsg->data[7] << 24) |
                  ((uint32_t)pmsg->data[6] << 16) |
                  ((uint32_t)pmsg->data[5] << 8) |
                  ((uint32_t)pmsg->data[4] << 0));
        WRITE_REG(hcan->Instance->sTxMailBox[box_num].TMDL,
                  ((uint32_t)pmsg->data[3] << 24) |
                  ((uint32_t)pmsg->data[2] << 16) |
                  ((uint32_t)pmsg->data[1] << 8) |
                  ((uint32_t)pmsg->data[0] << 0));
        /* Request transmission */
        SET_BIT(hcan->Instance->sTxMailBox[box_num].TMI, CAN_TMI0_TXRQ);

        return RT_EOK;
    }
    else
    {
        /* Update error code */
        hcan->ErrorCode |= HAL_CAN_ERROR_NOT_INITIALIZED;

        return -RT_ERROR;
    }
}

static int _can_recvmsg(struct rt_can_device *can, void *buf, rt_uint32_t fifo)
{
    uint8_t status;
    CAN_HandleTypeDef *hcan;
    struct rt_can_msg *pmsg;
    CanRxMessage rxheader = {0};

    RT_ASSERT(can);

    hcan = &((struct n32g43x_can *)can->parent.user_data)->CanHandle;
    pmsg = (struct rt_can_msg *) buf;

    /* get data */
    status = HAL_CAN_GetRxMessage(hcan, fifo, &rxheader, pmsg->data);
    if (RT_EOK != status)
        return -RT_ERROR;
    /* get id */
    if (CAN_ID_STD == rxheader.IDE)
    {
        pmsg->ide = RT_CAN_STDID;
        pmsg->id = rxheader.StdId;
    }
    else
    {
        pmsg->ide = RT_CAN_EXTID;
        pmsg->id = rxheader.ExtId;
    }
    /* get type */
    if (CAN_RTRQ_DATA == rxheader.RTR)
    {
        pmsg->rtr = RT_CAN_DTR;
    }
    else
    {
        pmsg->rtr = RT_CAN_RTR;
    }
    /* get len */
    pmsg->len = rxheader.DLC;
    /* get hdr */
    if (hcan->Instance == CAN1)
    {
        pmsg->hdr = (rxheader.FMI + 1) >> 1;
    }
#ifdef CAN2
    else if (hcan->Instance == CAN2)
    {
       pmsg->hdr = (rxheader.FMI >> 1) + 14;
    }
#endif

    return RT_EOK;
}


static const struct rt_can_ops _can_ops =
{
    _can_config,
    _can_control,
    _can_sendmsg,
    _can_recvmsg,
};

static void _can_rx_isr(struct rt_can_device *can, rt_uint32_t fifo)
{
    CAN_HandleTypeDef *hcan;
    RT_ASSERT(can);
    hcan = &((struct n32g43x_can *) can->parent.user_data)->CanHandle;

    switch (fifo)
    {
    case CAN_FIFO0:
        /* save to user list */
        if (CAN_GetFlagSTS(hcan->Instance, CAN_FIFO0) && CAN_GetFlagSTS(hcan->Instance, CAN_INT_FMP0))
        {
            rt_hw_can_isr(can, RT_CAN_EVENT_RX_IND | fifo << 8);
        }
        /* Check FULL flag for FIFO0 */
        if (__HAL_CAN_GET_FLAG(hcan, CAN_FLAG_FFULL0) && __HAL_CAN_GET_IT_SOURCE(hcan, CAN_INT_FF0))
        {
            /* Clear FIFO0 FULL Flag */
            __HAL_CAN_CLEAR_FLAG(hcan, CAN_FLAG_FFULL0);
        }

        /* Check Overrun flag for FIFO0 */
        if (__HAL_CAN_GET_FLAG(hcan, CAN_FLAG_FFOVR0) && __HAL_CAN_GET_IT_SOURCE(hcan, CAN_INT_FOV0))
        {
            /* Clear FIFO0 Overrun Flag */
            __HAL_CAN_CLEAR_FLAG(hcan, CAN_FLAG_FFOVR0);
            rt_hw_can_isr(can, RT_CAN_EVENT_RXOF_IND | fifo << 8);
        }
        break;
    case CAN_FIFO1:
        /* save to user list */
        if (HAL_CAN_GetRxFifoFillLevel(hcan, CAN_FIFO1) && __HAL_CAN_GET_IT_SOURCE(hcan, CAN_INT_FMP1))
        {
            rt_hw_can_isr(can, RT_CAN_EVENT_RX_IND | fifo << 8);
        }
        /* Check FULL flag for FIFO1 */
        if (__HAL_CAN_GET_FLAG(hcan, CAN_INT_FF1) && __HAL_CAN_GET_IT_SOURCE(hcan, CAN_INT_FF1))
        {
            /* Clear FIFO1 FULL Flag */
            __HAL_CAN_CLEAR_FLAG(hcan, CAN_INT_FF1);
        }

        /* Check Overrun flag for FIFO1 */
        if (__HAL_CAN_GET_FLAG(hcan, CAN_INT_FOV1) && __HAL_CAN_GET_IT_SOURCE(hcan, CAN_INT_FOV1))
        {
            /* Clear FIFO1 Overrun Flag */
            __HAL_CAN_CLEAR_FLAG(hcan, CAN_INT_FOV1);
            rt_hw_can_isr(can, RT_CAN_EVENT_RXOF_IND | fifo << 8);
        }
        break;
    }
}

#ifdef RT_USING_CAN1
/**
 * @brief This function handles CAN1 TX interrupts. transmit fifo0/1/2 is empty can trigger this interrupt
 */
void USB_HP_CAN1_TX_IRQHandler(void)
{
    rt_interrupt_enter();
    CAN_HandleTypeDef *hcan;
    hcan = &drv_can1.CanHandle;
    if (__HAL_CAN_GET_FLAG(hcan, CAN_FLAG_RQCP0))
    {
        if (__HAL_CAN_GET_FLAG(hcan, CAN_FLAG_TXOK0))
        {
            rt_hw_can_isr(&drv_can1.device, RT_CAN_EVENT_TX_DONE | 0 << 8);
        }
        else
        {
            rt_hw_can_isr(&drv_can1.device, RT_CAN_EVENT_TX_FAIL | 0 << 8);
        }
        /* Write 0 to Clear transmission status flag RQCPx */
        SET_BIT(hcan->Instance->TSTS, CAN_TSR_RQCP0);
    }
    else if (__HAL_CAN_GET_FLAG(hcan, CAN_FLAG_RQCP1))
    {
        if (__HAL_CAN_GET_FLAG(hcan, CAN_FLAG_TXOK1))
        {
            rt_hw_can_isr(&drv_can1.device, RT_CAN_EVENT_TX_DONE | 1 << 8);
        }
        else
        {
            rt_hw_can_isr(&drv_can1.device, RT_CAN_EVENT_TX_FAIL | 1 << 8);
        }
        /* Write 0 to Clear transmission status flag RQCPx */
        SET_BIT(hcan->Instance->TSTS, CAN_TSR_RQCP1);
    }
    else if (__HAL_CAN_GET_FLAG(hcan, CAN_FLAG_RQCP2))
    {
        if (__HAL_CAN_GET_FLAG(hcan, CAN_FLAG_TXOK2))
        {
            rt_hw_can_isr(&drv_can1.device, RT_CAN_EVENT_TX_DONE | 2 << 8);
        }
        else
        {
            rt_hw_can_isr(&drv_can1.device, RT_CAN_EVENT_TX_FAIL | 2 << 8);
        }
        /* Write 0 to Clear transmission status flag RQCPx */
        SET_BIT(hcan->Instance->TSTS, CAN_TSR_RQCP2);
    }
    rt_interrupt_leave();
}

/**
 * @brief This function handles CAN1 RX0 interrupts.
 */
void USB_LP_CAN1_RX0_IRQHandler(void)
{
    rt_interrupt_enter();
    _can_rx_isr(&drv_can1.device, CAN_FIFO0);
    rt_interrupt_leave();
}

/**
 * @brief This function handles CAN1 RX1 interrupts.
 */
void CAN1_RX1_IRQHandler(void)
{
    rt_interrupt_enter();
    _can_rx_isr(&drv_can1.device, CAN_FIFO1);
    rt_interrupt_leave();
}

/**
 * @brief This function handles CAN1 SCE interrupts.
 */
void CAN1_SCE_IRQHandler(void)
{
    rt_uint32_t errtype;
    CAN_HandleTypeDef *hcan;

    hcan = &drv_can1.CanHandle;
    errtype = hcan->Instance->ESR;

    rt_interrupt_enter();
    HAL_CAN_IRQHandler(hcan);

    switch ((errtype & 0x70) >> 4)
    {
    case RT_CAN_BUS_BIT_PAD_ERR:
        drv_can1.device.status.bitpaderrcnt++;
        break;
    case RT_CAN_BUS_FORMAT_ERR:
        drv_can1.device.status.formaterrcnt++;
        break;
    case RT_CAN_BUS_ACK_ERR:/* attention !!! test ack err's unit is transmit unit */
        drv_can1.device.status.ackerrcnt++;
        if (!READ_BIT(drv_can1.CanHandle.Instance->TSTS, CAN_FLAG_TXOK0))
            rt_hw_can_isr(&drv_can1.device, RT_CAN_EVENT_TX_FAIL | 0 << 8);
        else if (!READ_BIT(drv_can1.CanHandle.Instance->TSTS, CAN_FLAG_TXOK1))
            rt_hw_can_isr(&drv_can1.device, RT_CAN_EVENT_TX_FAIL | 1 << 8);
        else if (!READ_BIT(drv_can1.CanHandle.Instance->TSTS, CAN_FLAG_TXOK2))
            rt_hw_can_isr(&drv_can1.device, RT_CAN_EVENT_TX_FAIL | 2 << 8);
        break;
    case RT_CAN_BUS_IMPLICIT_BIT_ERR:
    case RT_CAN_BUS_EXPLICIT_BIT_ERR:
        drv_can1.device.status.biterrcnt++;
        break;
    case RT_CAN_BUS_CRC_ERR:
        drv_can1.device.status.crcerrcnt++;
        break;
    }

    drv_can1.device.status.lasterrtype = errtype & 0x70;
    drv_can1.device.status.rcverrcnt = errtype >> 24;
    drv_can1.device.status.snderrcnt = (errtype >> 16 & 0xFF);
    drv_can1.device.status.errcode = errtype & 0x07;
    hcan->Instance->MSR |= CAN_MSR_ERRI;
    rt_interrupt_leave();
}
#endif /* RT_USING_CAN1 */

#ifdef RT_USING_CAN2
/**
 * @brief This function handles CAN2 TX interrupts.
 */
void CAN2_TX_IRQHandler(void)
{
    rt_interrupt_enter();
    CAN_HandleTypeDef *hcan;
    hcan = &drv_can2.CanHandle;
    if (__HAL_CAN_GET_FLAG(hcan, CAN_FLAG_RQCP0))
    {
        if (__HAL_CAN_GET_FLAG(hcan, CAN_FLAG_TXOK0))
        {
            rt_hw_can_isr(&drv_can2.device, RT_CAN_EVENT_TX_DONE | 0 << 8);
        }
        else
        {
            rt_hw_can_isr(&drv_can2.device, RT_CAN_EVENT_TX_FAIL | 0 << 8);
        }
        /* Write 0 to Clear transmission status flag RQCPx */
        SET_BIT(hcan->Instance->TSTS, CAN_TSR_RQCP0);
    }
    else if (__HAL_CAN_GET_FLAG(hcan, CAN_FLAG_RQCP1))
    {
        if (__HAL_CAN_GET_FLAG(hcan, CAN_FLAG_TXOK1))
        {
            rt_hw_can_isr(&drv_can2.device, RT_CAN_EVENT_TX_DONE | 1 << 8);
        }
        else
        {
            rt_hw_can_isr(&drv_can2.device, RT_CAN_EVENT_TX_FAIL | 1 << 8);
        }
        /* Write 0 to Clear transmission status flag RQCPx */
        SET_BIT(hcan->Instance->TSTS, CAN_TSR_RQCP1);
    }
    else if (__HAL_CAN_GET_FLAG(hcan, CAN_FLAG_RQCP2))
    {
        if (__HAL_CAN_GET_FLAG(hcan, CAN_FLAG_TXOK2))
        {
            rt_hw_can_isr(&drv_can2.device, RT_CAN_EVENT_TX_DONE | 2 << 8);
        }
        else
        {
            rt_hw_can_isr(&drv_can2.device, RT_CAN_EVENT_TX_FAIL | 2 << 8);
        }
        /* Write 0 to Clear transmission status flag RQCPx */
        SET_BIT(hcan->Instance->TSTS, CAN_TSR_RQCP2);
    }
    rt_interrupt_leave();
}

/**
 * @brief This function handles CAN2 RX0 interrupts.
 */
void CAN2_RX0_IRQHandler(void)
{
    rt_interrupt_enter();
    _can_rx_isr(&drv_can2.device, CAN_FIFO0);
    rt_interrupt_leave();
}

/**
 * @brief This function handles CAN2 RX1 interrupts.
 */
void CAN2_RX1_IRQHandler(void)
{
    rt_interrupt_enter();
    _can_rx_isr(&drv_can2.device, CAN_FIFO1);
    rt_interrupt_leave();
}

/**
 * @brief This function handles CAN2 SCE interrupts.
 */
void CAN2_SCE_IRQHandler(void)
{
    rt_uint32_t errtype;
    CAN_HandleTypeDef *hcan;

    hcan = &drv_can2.CanHandle;
    errtype = hcan->Instance->ESR;

    rt_interrupt_enter();
    HAL_CAN_IRQHandler(hcan);

    switch ((errtype & 0x70) >> 4)
    {
    case RT_CAN_BUS_BIT_PAD_ERR:
        drv_can2.device.status.bitpaderrcnt++;
        break;
    case RT_CAN_BUS_FORMAT_ERR:
        drv_can2.device.status.formaterrcnt++;
        break;
    case RT_CAN_BUS_ACK_ERR:
        drv_can2.device.status.ackerrcnt++;
        if (!READ_BIT(drv_can2.CanHandle.Instance->TSTS, CAN_FLAG_TXOK0))
            rt_hw_can_isr(&drv_can2.device, RT_CAN_EVENT_TX_FAIL | 0 << 8);
        else if (!READ_BIT(drv_can2.CanHandle.Instance->TSTS, CAN_FLAG_TXOK1))
            rt_hw_can_isr(&drv_can2.device, RT_CAN_EVENT_TX_FAIL | 1 << 8);
        else if (!READ_BIT(drv_can2.CanHandle.Instance->TSTS, CAN_FLAG_TXOK2))
            rt_hw_can_isr(&drv_can2.device, RT_CAN_EVENT_TX_FAIL | 2 << 8);
        break;
    case RT_CAN_BUS_IMPLICIT_BIT_ERR:
    case RT_CAN_BUS_EXPLICIT_BIT_ERR:
        drv_can2.device.status.biterrcnt++;
        break;
    case RT_CAN_BUS_CRC_ERR:
        drv_can2.device.status.crcerrcnt++;
        break;
    }

    drv_can2.device.status.lasterrtype = errtype & 0x70;
    drv_can2.device.status.rcverrcnt = errtype >> 24;
    drv_can2.device.status.snderrcnt = (errtype >> 16 & 0xFF);
    drv_can2.device.status.errcode = errtype & 0x07;
    hcan->Instance->MSR |= CAN_MSR_ERRI;
    rt_interrupt_leave();
}
#endif /* RT_USING_CAN2 */

/**
 * @brief  Error CAN callback.
 * @param  hcan pointer to a CAN_HandleTypeDef structure that contains
 *         the configuration information for the specified CAN.
 * @retval None
 */
void HAL_CAN_ErrorCallback(CAN_HandleTypeDef *hcan)
{
    __HAL_CAN_ENABLE_IT(hcan, CAN_INT_EWG |
                        CAN_INT_EPV |
                        CAN_IT_BUSOFF |
                        CAN_INT_LEC |
                        CAN_IT_ERROR |
                        CAN_INT_FMP0 |
                        CAN_INT_FOV0 |
                        CAN_INT_FF0 |
                        CAN_INT_FMP1 |
                        CAN_INT_FOV1 |
                        CAN_INT_FF1 |
                        CAN_INT_TME);
}

int rt_hw_can_init(void)
{
    struct can_configure config = CANDEFAULTCONFIG;
    config.privmode = RT_CAN_MODE_NOPRIV;
    config.ticks = 50;
#ifdef RT_CAN_USING_HDR
    config.maxhdr = 14;
#ifdef CAN2
    config.maxhdr = 28;
#endif
#endif
    /* config default filter */
    CAN_FilterTypeDef filterConf = {0};
    filterConf.FilterIdHigh = 0x0000;
    filterConf.FilterIdLow = 0x0000;
    filterConf.FilterMaskIdHigh = 0x0000;
    filterConf.FilterMaskIdLow = 0x0000;
    filterConf.FilterFIFOAssignment = CAN_FILTER_FIFO0;
    filterConf.FilterBank = 0;
    filterConf.FilterMode = CAN_FILTERMODE_IDMASK;
    filterConf.FilterScale = CAN_FILTERSCALE_32BIT;
    filterConf.FilterActivation = ENABLE;
    filterConf.SlaveStartFilterBank = 14;

#ifdef RT_USING_CAN1
    filterConf.FilterBank = 0;

    drv_can1.FilterConfig = filterConf;
    drv_can1.device.config = config;
    /* register CAN1 device */
    rt_hw_can_register(&drv_can1.device,
                       drv_can1.name,
                       &_can_ops,
                       &drv_can1);
#endif /* RT_USING_CAN1 */

#ifdef RT_USING_CAN2
    filterConf.FilterBank = filterConf.SlaveStartFilterBank;

    drv_can2.FilterConfig = filterConf;
    drv_can2.device.config = config;
    /* register CAN2 device */
    rt_hw_can_register(&drv_can2.device,
                       drv_can2.name,
                       &_can_ops,
                       &drv_can2);
#endif /* RT_USING_CAN2 */

    return 0;
}

INIT_BOARD_EXPORT(rt_hw_can_init);

#endif /* RT_USING_CAN */

/************************** end of file ******************/
