1������˵��
    1��ADC�������PC2���Զ�ע�����PC3���ŵ�ģ���ѹ����TIM1 CC2�¼��´�������
    2�����й���ת�����ͨ��DMA_CH1ͨ����ȡ������ADC_RegularConvertedValueTab[32]����
        ע��ת�����ͨ��ת�������ж϶�ȡ������ADC_InjectedConvertedValueTab[32]����
2��ʹ�û���
    �������������  KEIL MDK-ARM V5.26.2.0
    Ӳ��������      ����N32L43XM-STB����
3��ʹ��˵��
    ϵͳ���ã�
        1��ʱ��Դ��
            HSE=8M,PLL=108M,AHB=108M,APB1=27M,APB2=54M,ADC CLK=108M/16,ADC 1M CLK=HSE/8,DMA CLK=108M,TIM1 CLK=108M
        2���жϣ�
            ADCע��ת���������жϴ򿪣��������ȼ�0�������ȼ�0
            �жϺ����н�ע������ȡ��ADC_InjectedConvertedValueTab[32]���飬����תPC6��ƽ
        3���˿����ã�
            PC2ѡ��Ϊģ�⹦��ADC1ת��ͨ��
            PC3ѡ��Ϊģ�⹦��ADC1ת��ͨ��
            PC6ѡ��Ϊͨ��IO���
            PA8ѡ��ΪTIM1 CH1 PWM���
            PA11ѡ��ΪTIM1 CH4 PWM���
        4��DMA��
            DMA_CH1ͨ���ػ�ģʽ����32�����ֵ�ADC1ת�������ADC_RegularConvertedValueTab[32]����
        5��ADC��
            ADC TIM1 CC2������12λ�����Ҷ��룬����ת��ͨ��PC2���Զ�ע��ת��ͨ��PC3��ģ���ѹ����
        6��TIM��
            TIM1����CH1 CH4�����CH2��������ADCת��		
    ʹ�÷�����
        1�������򿪵���ģʽ��������ADC_RegularConvertedValueTab[32],ADC_InjectedConvertedValueTab[32]��ӵ�watch���ڹ۲�
        2��ͨ���ı�PC2 PC3���ŵĵ�ѹ����ÿ��CC2�¼�����ʱת��һ�ι����ע��ͨ�������������ڶ�Ӧ�����С�ͬʱ��PA8 PA11���Կ���TIM1 CH1��CH4�� PWM ����
4��ע������
    ��ϵͳ����HSEʱ��ʱ��һ��HSIҲ�Ǵ򿪵ģ���RCC_ConfigAdc1mClk(RCC_ADC1MCLK_SRC_HSE, RCC_ADC1MCLK_DIV8)��������ΪHSE����HSI
    ��ϵͳ����HSIʱ��ʱ��һ��HSE�ǹرյģ���RCC_ConfigAdc1mClk(RCC_ADC1MCLK_SRC_HSI, RCC_ADC1MCLK_DIV8)ֻ������ΪHSI