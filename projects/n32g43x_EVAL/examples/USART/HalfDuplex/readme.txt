1������˵��

	�ò�����ʾ��USARTy��USARTz��ͨ����ѯ����ʶ��ʵ�ְ�˫��ģʽ��
����ͨ�š�
    ���ȣ�USARTy����TxBuffer1������USARTz��USARTz�������ݴ���RxBuffer2��
    ���USARTz����TxBuffer2������USARTy��USARTy�������ݴ���RxBuffer1��
    ��󣬷ֱ�Ƚ�������������뷢�����ݣ��ȽϽ������TransferStatus1����
��TransferStatus2������
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
    - ��˫��ģʽʹ��
    
    USART�����������£�
    - USART1_Tx.PB6    <------->   USART2_Tx.PA2
    ��
    - USART3_Tx.PB10   <------->   UART4_Tx.PC10
    ��
    - UART4_Tx.PD13    <------->    UART5_Tx.PC12

    
    ���Բ���������
    - Demo��KEIL�����±����������MCU
    - ��λ���к����β鿴����TransferStatus1��TransferStatus2�����У�
      PASSEDΪ����ͨ����FAILEDΪ�����쳣


4��ע������
    USART_Tx������������衣