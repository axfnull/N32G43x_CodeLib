1������˵��

	�ò�����ʾ�����ʹ��USART�ദ����ģʽ��USARTy��USARTz������USART1
��USART2��USART3��UART4��UART4��UART5��
    ���ȣ��ֱ�����USARTy��USARTz�ĵ�ַΪ0x1��0x2��USARTy������USARTz
�����ַ�0x33��USARTz�յ�0x33���㷭תLED1��LED2��LED3�����š�
    һ��KEY1_INT_EXTI_LINE�߼�⵽�����أ������EXTI0�жϣ���
EXTI0_IRQHandler�жϴ�������(the ControlFlag = 0)��USARTz���뾲Ĭ
ģʽ���ھ�Ĭģʽ�У�LED����ֹͣ��ת��ֱ��KEY1_INT_EXTI_LINE�߼�⵽
������(the ControlFlag = 1)����EXTI0_IRQHandler�жϴ������У�USARTy
���͵�ַ0x102����USARTz��LED��������������ת��


2��ʹ�û���

	�������������KEIL MDK-ARM Professional Version 5.26.2.0

        Ӳ����������Сϵͳ��N32L43XM-STB_V1.1/N32L40XM-STB_V1.1


3��ʹ��˵��
	
	ϵͳʱ���������£�
    - ʱ��Դ = HSE + PLL
    - ϵͳʱ�� = 108MHz
    
    USARTy�������£�
    - ������ = 115200 baud
    - �ֳ� = 9����λ
    - 1ֹͣλ
    - У����ƽ���
    - Ӳ�������ƽ��ã�RTS��CTS�źţ�
    - �������ͷ�����ʹ��  
    
    
    USART�����������£�    
    - USART1_Tx.PA9    <------->   USART2_Rx.PA3
    - USART1_Rx.PA10   <------->   USART2_Tx.PA2 
    ��
    - USART3_Tx.PB10   <------->   UART4_Rx.PC11
    - USART3_Rx.PB11   <------->   UART4_Tx.PC10
    ��
    - UART4_Tx.PD13    <------->   UART5_Rx.PB9
    - UART4_Rx.PD12    <------->   UART5_Tx.PB8   
    
    KEY1_INT_EXTI_LINE.PA0    <------->   WAKEUP_KEY
    
    LED1    <------->   PB5
    LED2    <------->   PB4
    LED3    <------->   PA8

    
    ���Բ���������
    - Demo��KEIL�����±����������MCU
    - ��λ���У��۲�LED1~3�Ƿ�����˸״̬
    - ������KEY���۲�LED1~3�Ƿ�ֹͣ��˸
    - �ٴΰ�����KEY���۲�LED1~3�Ƿ�ָ���˸


4��ע������