1������˵��

    �ò�����ʾ��USARTy��USARTz��ͨ��DMAʵ�ֵĻ���ͨ�š�USARTy��USARTz
������USART1��USART2��USART3��UART4��UART4��UART5��
    ���ȣ�DMA����TxBuffer1������USARTy�������ݼĴ�����������ݷ�����
USARTz��USARTz���մ�DMA���������ݣ�����RxBuffer2���Ƚ��ա������ݣ��Ƚ�
�������TransferStatus2������
    ͬʱ��DMA����TxBuffer2������USARTz�������ݼĴ�����������ݷ�����
USARTy��USARTy���մ�DMA���������ݣ�����RxBuffer1���Ƚ��ա������ݣ��Ƚ�
�������TransferStatus1������   


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
    - DMA����ģʽ��DMA����ģʽʹ��
    
    USART�����������£�
    - USART1_Tx.PB6    <------->   USART2_Rx.PA3
    - USART1_Rx.PB7    <------->   USART2_Tx.PA2 
    ��
    - USART3_Tx.PB10   <------->   UART4_Rx.PB1
    - USART3_Rx.PB11   <------->   UART4_Tx.PB0
    ��
    - UART4_Tx.PC10    <------->   UART5_Rx.PB5
    - UART4_Rx.PC11    <------->   UART5_Tx.PB4

    
    ���Բ���������
    - Demo��KEIL�����±����������MCU
    - ��λ���У����β鿴����TransferStatus1��TransferStatus2�����У�
      PASSEDΪ����ͨ����FAILEDΪ�����쳣


4��ע������