1������˵��

    �ò�����ʾ��USARTy��USARTz��ʵ�ִ���IrDA�͹���ģʽ������빦�ܵĻ���ͨ�š�
    ���ȣ�USARTy����TxBuffer1������USARTz��USARTzͨ���жϽ������ݴ���RxBuffer1��
    ��󣬱ȽϽ��������뷢�����ݣ��ȽϽ������TransferStatus������
    USARTy��USARTz������USART1��USART2��USART3��UART4��UART4��UART5��


2��ʹ�û���

	�������������KEIL MDK-ARM Professional Version 5.26.2.0

        Ӳ����������Сϵͳ��N32L43XM-STB_V1.1/N32L40XM-STB_V1.1��IrDA��������������


3��ʹ��˵��
	
    ϵͳʱ���������£�
    - ʱ��Դ = HSE + PLL
    - ϵͳʱ�� = 8MHz
    
    USART�������£�
    - ������ = 2400 baud
    - �ֳ� = 8����λ
    - 1ֹͣλ
    - У����ƽ���
    - Ӳ�������ƽ��ã�RTS��CTS�źţ�
    - �������ͷ�����ʹ��
    - IrDAģʽʹ��
    
    USART�����������£�
    - USART1_Tx.PB8    <------->   IrDA Transmitter
    - USART2_Rx.PA3    <------->   IrDA Receiver
    ��
    - USART3_Tx.PC10   <------->   IrDA Transmitter
    - UART4_Rx.PB15    <------->   IrDA Receiver
    ��
    - UART4_Tx.PC10    <------->    IrDA Transmitter
    - UART5_Rx.PB5     <------->    IrDA Receiver
    
    - GPIO.PC1        <------->    38kHz carrier

    
    ���Բ���������
	- ��λ����MCU���鿴����TransferStatus�����У�PASSEDΪ����ͨ����FAILEDΪ�����쳣


4��ע������