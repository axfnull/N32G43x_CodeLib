1������˵��

    �ò�����ʾ��LPUART��USARTz��ͨ��DMAʵ�ֵĻ���ͨ�š�USARTz������USART3
��UART5��
    ���ȣ�DMA����TxBuffer1������LPUART�������ݼĴ�����������ݷ�����
USARTz��USARTz�����жϽ������ݣ�����RxBuffer2��
    ͬʱ��DMA����TxBuffer2������USARTz�������ݼĴ�����������ݷ�����
USARTy��LPUARTͨ����ѯ����־�������ݣ�����RxBuffer1��
    ��󣬷ֱ�Ƚ������ա������ݣ��ȽϽ������TransferStatus1����
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
    - DMA����ģʽ��DMA����ģʽʹ��
    
    LPUART�������£�
    - ������ = 9600 baud
    - �ֳ� = 8����λ���̶���
    - 1ֹͣλ���̶���
    - У����ƽ���
    - Ӳ�������ƽ��ã�RTS��CTS�źţ�
    - �������ͷ�����ʹ��
    - DMA����ģʽ��DMA����ģʽʹ��
    
    USART�����������£�
    - LPUART_Tx.PA4   <------->   USART3_Rx.PC11
    - LPUART_Rx.PA3   <------->   USART3_Tx.PC10
    ��
    - LPUART_Tx.PB6    <------->   UART5_Rx.PB5
    - LPUART_Rx.PB7    <------->   UART5_Tx.PB4

    
    ���Բ���������
    - Demo��KEIL�����±����������MCU
    - ��λ���У����β鿴����TransferStatus1��TransferStatus2�����У�
      PASSEDΪ����ͨ����FAILEDΪ�����쳣


4��ע������