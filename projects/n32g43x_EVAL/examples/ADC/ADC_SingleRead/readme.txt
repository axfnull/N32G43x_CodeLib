1������˵��
    1��ADC1����ת��PC0 PC1���ŵ�ģ���ѹ
    2�������������һ�Σ��ɼ�һ�εķ�ʽ
2��ʹ�û���
    �������������  KEIL MDK-ARM V5.26.2.0
    Ӳ��������      ����N32L43XM-STB����
3��ʹ��˵��
    ϵͳ���ã�
        1��ʱ��Դ��
            HSE=8M,PLL=108M,AHB=108M,APB1=27M,APB2=54M,ADC CLK=108M/16,ADC 1M CLK=HSE/8
        2���˿����ã�
            PC0ѡ��Ϊģ�⹦��ADC1ת��ͨ��
            PC1ѡ��Ϊģ�⹦��ADC1ת��ͨ��
        3��ADC��
            ADC�������ת����12λ�����Ҷ��룬ת��PC0 PC1��ģ���ѹ����
    ʹ�÷�����
        1�������򿪵���ģʽ��������ADCConvertedValue��ӵ�watch���ڹ۲�
        2��ͨ���ı�PC0 PC1���ŵĵ�ѹ�����Կ���ת���������ͬ���ı�
4��ע������
    ��ϵͳ����HSEʱ��ʱ��һ��HSIҲ�Ǵ򿪵ģ���RCC_ConfigAdc1mClk(RCC_ADC1MCLK_SRC_HSE, RCC_ADC1MCLK_DIV8)��������ΪHSE����HSI
    ��ϵͳ����HSIʱ��ʱ��һ��HSE�ǹرյģ���RCC_ConfigAdc1mClk(RCC_ADC1MCLK_SRC_HSI, RCC_ADC1MCLK_DIV8)ֻ������ΪHSI