1������˵��

	�ò�����ʾ��USARTy��ͬ��ģʽ����SPIy��ͨ����ѯ����ʶʵ�ֵĻ���ͨ�š�
USARTy��SPIy������USART1��SPI1��USART3��SPI1��USART2��SPI2��
    ���ȣ�ͨ��TXC����־��USARTy����TxBuffer1������SPIy����SPIy�������ݣ�
���ѯRNE����־�����յ����ݴ���RxBuffer1��
    ���SPIyͨ����ѯTE����־������TxBuffer2������USARTy��USARTy������
RXDNE����־�������ݣ��������RxBuffer2��
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
    - ʱ��ʹ��
    - ʱ�Ӽ��ԣ������ⷢ��ʱ���ָߵ�ƽ
    - ʱ����λ���ڵڶ���ʱ�ӱ��ز�����һ������
    - ���һλʱ�����壺���һλ���ݵ�ʱ�������CK���
    
    SPI�������£�
    - ���� = ��˫��˫��ģʽ
    - ģʽ = ��ģʽ
    - ���ݴ�С = 8λ����֡
    - CPOL = ����״̬ʱ��ʱ�ӱ��ָߵ�ƽ
    - CPHA = ���ݲ����ӵڶ���ʱ�ӱ��ؿ�ʼ
    - NSS = ����������豸����
    - ��1λ = ��1λΪLSB
    
    
    USART�����������£�    
    - USART1_Tx.PA9    <------->   SPI1_MOSI.PA7
    - USART1_Rx.PA10   <------->   SPI1_MISO.PA6
    - USART1_Clk.PA8   <------->   SPI1_SCK.PA5     
    ��
    - USART3_Tx.PC10   <------->   SPI1_MOSI.PD6
    - USART3_Rx.PC11   <------->   SPI1_MISO.PD5
    - USART3_Clk.PC12  <------->   SPI1_SCK.PD4
    ��
    - USART2_Tx.PA2    <------->   SPI2_MOSI.PB15
    - USART2_Rx.PA3    <------->   SPI2_MISO.PB14
    - USART2_Clk.PA4   <------->   SPI2_SCK.PB13

    
    ���Բ���������
    - Demo��KEIL�����±����������MCU
    - ��λ���У����β鿴����TransferStatus1��TransferStatus2�����У�
      PASSEDΪ����ͨ����FAILEDΪ�����쳣


4��ע������