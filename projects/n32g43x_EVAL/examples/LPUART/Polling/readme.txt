1������˵��

	�ò�����ʾ��LPUART��USARTz��ͨ����ѯ����ʶʵ�ֵĻ���ͨ�š�USARTz
������USART1��UART4��
    ���ȣ�LPUART����TxBuffer1������USARTz��USARTz�������ݴ���RxBuffer1��
�ȽϽ��������뷢�����ݣ��ȽϽ������TransferStatus������
    ���USARTz����TxBuffer2������LPUART��LPUART�������ݴ���RxBuffer2��
�ȽϽ��������뷢�����ݣ��ȽϽ������TransferStatus������


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
    - LPUART_Tx.PA4   <------->   USART1_Rx.PA10
    - LPUART_Rx.PA3   <------->   USART1_Tx.PA9
    ��
    - LPUART_Tx.PB6   <------->   UART4_Rx.PD12
    - LPUART_Rx.PB7   <------->   UART4_Tx.PD13

    
    ���Բ���������
    - Demo��KEIL�����±����������MCU
    - ��λ���У������β鿴����TransferStatus�����У�PASSEDΪ����ͨ����
      FAILEDΪ�����쳣


4��ע������