1������˵��

    �ò�����ʾ��LPUART��USARTz��ʹ��Ӳ��������ͨ���ж�ʵ�ֵĻ���ͨ�š�
    LPUART����TxBuffer1������USARTz��USARTz�������ݴ���RxBuffer2��ͬʱ��
USARTz����TxBuffer2������LPUART��LPUART�������ݴ���RxBuffer1��
    ��󣬷ֱ�Ƚ�������������뷢�����ݣ��ȽϽ������TransferStatus1����
��TransferStatus2������


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
    - Ӳ��������ʹ�ܣ�RTS��CTS�źţ�
    - �������ͷ�����ʹ��
    
    USART�������£�
    - ������ = 9600 baud
    - �ֳ� = 8����λ
    - 1ֹͣλ
    - У����ƽ���
    - Ӳ��������ʹ�ܣ�RTS��CTS�źţ�
    - �������ͷ�����ʹ��
    
    LPUART��USART�����������£�
    - LPUART_Tx.PB10    <------->   USART2_Rx.PA3
    - LPUART_Rx.PB11    <------->   USART2_Tx.PA2
    - LPUART_RTS.PB12    <------->   USART2_CTS.PA0
    - LPUART_CTS.PB13    <------->   USART2_RTS.PA1
    ���Բ���������
    - Demo��KEIL�����±����������MCU
    - ��λ���У����β鿴����TransferStatus1��TransferStatus2�����У�
      PASSEDΪ����ͨ����FAILEDΪ�����쳣


4��ע������