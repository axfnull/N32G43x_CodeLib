1������˵��
    1��ADC������ת��PC2���ŵ�ģ���ѹ
    2������ADCת�����ͨ��DMA_CH1ͨ����ȡ������ADCConvertedValue
2��ʹ�û���
    �������������  KEIL MDK-ARM V5.26.2.0
    Ӳ��������      ����N32L43XM-STB����
3��ʹ��˵��
    ϵͳ���ã�
        1��ʱ��Դ��
            HSE=8M,PLL=108M,AHB=108M,APB1=27M,APB2=54M,ADC CLK=108M/16,ADC 1M CLK=HSE/8,DMA CLK=108M
        2���˿����ã�
            PC2ѡ��Ϊģ�⹦��ADCת��ͨ��8
        3��DMA��
            DMA_CH1ͨ���ػ�ģʽ����һ�����ֵ�ADCת�������ADCConvertedValue����
        4��ADC��
            ADC����ת����ɨ��ģʽ�����������12λ�����Ҷ��룬ת��ͨ��8��PC2��ģ���ѹ����
    ʹ�÷�����
        1�������򿪵���ģʽ��������ADCConvertedValue��ӵ�watch���ڹ۲�
        2��ͨ���ı�PC2���ŵĵ�ѹ�����Կ���ת���������ͬ���ı�
4��ע������
    ��ϵͳ����HSEʱ��ʱ��һ��HSIҲ�Ǵ򿪵ģ���RCC_ConfigAdc1mClk(RCC_ADC1MCLK_SRC_HSE, RCC_ADC1MCLK_DIV8)��������ΪHSE����HSI
    ��ϵͳ����HSIʱ��ʱ��һ��HSE�ǹرյģ���RCC_ConfigAdc1mClk(RCC_ADC1MCLK_SRC_HSI, RCC_ADC1MCLK_DIV8)ֻ������ΪHSI