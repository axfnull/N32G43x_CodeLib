1������˵��

    �ò�����ʾ��USARTy��USARTz��ͨ��DMAʵ�ֵĻ���ͨ�š�USARTy��USARTz
������USART1��USART2��USART3��UART4��UART4��UART5��
    ���ȣ�DMA����TxBuffer1������USARTy�������ݼĴ�����������ݷ�����
USARTz��USARTz�����жϽ������ݣ�����RxBuffer2��
    ͬʱ��DMA����TxBuffer2������USARTz�������ݼĴ�����������ݷ�����
USARTy��USARTyͨ����ѯ����־�������ݣ�����RxBuffer1��
    ��󣬷ֱ�Ƚ������ա������ݣ��ȽϽ������TransferStatus1����
��TransferStatus2������


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
    - DMA����ģʽʹ�ܣ�DMA����ģʽ����
    
    USART�����������£�
    - USART1_Tx.PB8    <------->   USART2_Rx.PB5
    - USART1_Rx.PB7    <------->   USART2_Tx.PB4 
    ��
    - USART3_Tx.PC10   <------->   UART4_Rx.PB15
    - USART3_Rx.PC11   <------->   UART4_Tx.PB14
    ��
    - UART4_Tx.PD13    <------->   UART5_Rx.PB9
    - UART4_Rx.PD12    <------->   UART5_Tx.PB8

    
    ���Բ���������
    - Demo��KEIL�����±����������MCU
    - ��λ���У����β鿴����TransferStatus1��TransferStatus2�����У�
      PASSEDΪ����ͨ����FAILEDΪ�����쳣


4��ע������