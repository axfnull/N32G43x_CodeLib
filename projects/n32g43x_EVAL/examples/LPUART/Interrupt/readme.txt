1������˵��

    �ò�����ʾ��LPUART��USARTz��ͨ���ж�ʵ�ֵĻ���ͨ�š�USARTz������USART2
��UART5��
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
    - Ӳ�������ƽ��ã�RTS��CTS�źţ�
    - �������ͷ�����ʹ��
    
    USART�������£�
    - ������ = 9600 baud
    - �ֳ� = 8����λ
    - 1ֹͣλ
    - У����ƽ���
    - Ӳ�������ƽ��ã�RTS��CTS�źţ�
    - �������ͷ�����ʹ��
    
    LPUART��USART�����������£�
    - LPUART_Tx.PB10    <------->   USART2_Rx.PB5
    - LPUART_Rx.PB11    <------->   USART2_Tx.PB4
    ��
    - LPUART_Tx.PC4    <------->    UART5_Rx.PB9
    - LPUART_Rx.PC5    <------->    UART5_Tx.PB8

    
    ���Բ���������
    - Demo��KEIL�����±����������MCU
    - ��λ���У����β鿴����TransferStatus1��TransferStatus2�����У�
      PASSEDΪ����ͨ����FAILEDΪ�����쳣


4��ע������