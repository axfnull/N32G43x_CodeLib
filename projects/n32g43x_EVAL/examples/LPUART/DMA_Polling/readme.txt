1������˵��

    �ò�����ʾ��LPUART��USARTz��ͨ��DMAʵ�ֵĻ���ͨ�š�USARTz������USART3
��UART4��
    ���ȣ�DMA����TxBuffer1������LPUART�������ݼĴ�����������ݷ�����
USARTz��USARTz���մ�DMA���������ݣ�����RxBuffer2���Ƚ��ա������ݣ��Ƚ�
�������TransferStatus2������
    ͬʱ��DMA����TxBuffer2������USARTz�������ݼĴ�����������ݷ�����
LPUART��LPUART���մ�DMA���������ݣ�����RxBuffer1���Ƚ��ա������ݣ��Ƚ�
�������TransferStatus1������   


2��ʹ�û���

	�������������KEIL MDK-ARM Professional Version 5.26.2.0

        Ӳ����������Сϵͳ��N32L43XM-STB_V1.1/N32L40XM-STB_V1.1


3��ʹ��˵��
	
    ϵͳʱ���������£�
    - ʱ��Դ = HSE + PLL
    - ϵͳʱ�� = 108MHz
    
    LPUART�������£�
    - ������ = 9600 baud
    - �ֳ� = 8����λ���̶���
    - 1ֹͣλ���̶���
    - У����ƽ���
    - Ӳ�������ƽ��ã�RTS��CTS�źţ�
    - �������ͷ�����ʹ��
    - DMA����ģʽ��DMA����ģʽʹ��
    
    USART�������£�
    - ������ = 9600 baud
    - �ֳ� = 8����λ
    - 1ֹͣλ
    - У����ƽ���
    - Ӳ�������ƽ��ã�RTS��CTS�źţ�
    - �������ͷ�����ʹ��
    - DMA����ģʽ��DMA����ģʽʹ��
    
    LPUART��USART�����������£�
    - LPUART_Tx.PC10   <------->   USART3_Rx.PB11
    - LPUART_Rx.PC11   <------->   USART3_Tx.PB10
    ��
    - LPUART_Tx.PA1    <------->   UART4_Rx.PB1
    - LPUART_Rx.PA0    <------->   UART4_Tx.PB0

    
    ���Բ���������
    - Demo��KEIL�����±����������MCU
    - ��λ���У����β鿴����TransferStatus1��TransferStatus2�����У�
      PASSEDΪ����ͨ����FAILEDΪ�����쳣


4��ע������