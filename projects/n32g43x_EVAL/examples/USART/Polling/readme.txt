1������˵��

	�ò�����ʾ��USARTy��USARTz��ͨ����ѯ����ʶʵ�ֵĻ���ͨ�š�
    ���ȣ�USARTy����TxBuffer1������USARTz��USARTz�������ݴ���RxBuffer1��
�ȽϽ��������뷢�����ݣ��ȽϽ������TransferStatus������
    ���USARTz����TxBuffer2������USARTy��USARTy�������ݴ���RxBuffer2��
�ȽϽ��������뷢�����ݣ��ȽϽ������TransferStatus������
    USARTy��USARTz������USART1��USART2��USART3��UART4��UART4��UART5��


2��ʹ�û���

	�������������KEIL MDK-ARM Professional Version 5.26.2.0

        Ӳ����������Сϵͳ��N32L43XM-STB_V1.1/N32L40XM-STB_V1.1


3��ʹ��˵��
	
    ϵͳʱ���������£�
    - ʱ��Դ = HSE + PLL
    - ϵͳʱ�� = 108MHz
    
    USART�������£�
    - ������ = 115200 baud
    - �ֳ� = 8����λ
    - 1ֹͣλ
    - У����ƽ���
    - Ӳ�������ƽ��ã�RTS��CTS�źţ�
    - �������ͷ�����ʹ��
    
    USART�����������£�
    - USART1_Tx.PA4    <------->   USART2_Rx.PA3
    - USART1_Rx.PA5    <------->   USART2_Tx.PA2 
    ��
    - USART3_Tx.PB10   <------->   UART4_Rx.PB1
    - USART3_Rx.PB11   <------->   UART4_Tx.PB0
    ��
    - UART4_Tx.PB14    <------->   UART5_Rx.PB5
    - UART4_Rx.PB15    <------->   UART5_Tx.PB4

    
    ���Բ���������
    - Demo��KEIL�����±����������MCU
    - ��λ���У������β鿴����TransferStatus�����У�PASSEDΪ����ͨ����
      FAILEDΪ�����쳣


4��ע������