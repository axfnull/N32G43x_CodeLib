1������˵��
    1��ADC����ͨ������PC2��PC3���ŵ�ģ���ѹ��ע��ͨ������PA0��PA1���ŵ�ģ���ѹ
    2�����й���ת�����ͨ��DMA_CH1ͨ����ȡ������ADC_RegularConvertedValueTab[64]����
        ע��ת�����ͨ��ת�������ж϶�ȡ������ADC_InjectedConvertedValueTab[32]����
2��ʹ�û���
    �������������  KEIL MDK-ARM V5.26.2.0
    Ӳ��������      ����N32L43XM-STB����
3��ʹ��˵��
    ϵͳ���ã�
        1��ʱ��Դ��
            HSE=8M,PLL=108M,AHB=108M,APB1=27M,APB2=54M,ADC CLK=108M/16,ADC 1M CLK=HSE/8,DMA CLK=108M
        2���жϣ�
            ADCע��ת���������жϴ򿪣��������ȼ�0�������ȼ�0
            �жϴ������ע��ת�������ADC_InjectedConvertedValueTab[32]����
        3���˿����ã�
            PC2ѡ��Ϊģ�⹦��ADC1ת��ͨ��
            PC3ѡ��Ϊģ�⹦��ADC1ת��ͨ��
            PA0ѡ��Ϊģ�⹦��ADC1ת��ͨ��
            PA1ѡ��Ϊģ�⹦��ADC1ת��ͨ��
            PD11ѡ��Ϊ�ⲿEXTI�¼������ش���
            PA15ѡ��Ϊ�ⲿEXTI�¼������ش���
        4��DMA��
            DMA_CH1ͨ���ػ�ģʽ����64�����ֵ�ADC1����ͨ��ת�������ADC_RegularConvertedValueTab[64]����
        5��ADC��
            ADC����ͨ��ɨ����ģʽ��EXTI11������12λ�����Ҷ��룬ת��ͨ��PC2��PC3��ģ���ѹ����
            ADCע��ͨ��ɨ��ģʽ��EXTI15������12λ�����Ҷ��룬ת��ͨ��PA0��PA1��ģ���ѹ����
    ʹ�÷�����
        1�������򿪵���ģʽ��������ADC_RegularConvertedValueTab[64],ADC_InjectedConvertedValueTab[32]��ӵ�watch���ڹ۲�
        2��ͨ��PD11�������ؿ��Դ�������ͨ�����ݲ�����PA15�������ؿ��Դ���ע��ͨ�����ݲ���
4��ע������
    ��ϵͳ����HSEʱ��ʱ��һ��HSIҲ�Ǵ򿪵ģ���RCC_ConfigAdc1mClk(RCC_ADC1MCLK_SRC_HSE, RCC_ADC1MCLK_DIV8)��������ΪHSE����HSI
    ��ϵͳ����HSIʱ��ʱ��һ��HSE�ǹرյģ���RCC_ConfigAdc1mClk(RCC_ADC1MCLK_SRC_HSI, RCC_ADC1MCLK_DIV8)ֻ������ΪHSI