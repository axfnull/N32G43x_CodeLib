1������˵��
    1��ADC����ת���ڲ��¶ȴ�������ģ���ѹ����ת��Ϊ�¶�ֵ
2��ʹ�û���
    �������������  KEIL MDK-ARM V5.26.2.0
    Ӳ��������      ����N32L43XM-STB����
3��ʹ��˵��
    ϵͳ���ã�
        1��ʱ��Դ��
            HSE=8M,PLL=108M,AHB=108M,APB1=27M,APB2=54M,ADC CLK=108M/16,ADC 1M CLK=HSE/8,DMA CLK=108M
        2��DMA��
            DMA_CH1ͨ���ػ�ģʽ����һ�����ֵ�ADCת�������ADCConvertedValue����
        3��ADC��
            ADC����ת�������������12λ�����Ҷ��룬ת��ͨ��17���ڲ��¶ȴ�������ģ���ѹ����
        4���˿����ã�
            PA9ѡ��ΪUSART1��TX����
            PA10ѡ��ΪUSART1��RX����
        5��USART��
            USART1 115200�����ʡ�8λ����λ��1λֹͣλ������żУ��λ����Ӳ�����ء����ͺͽ���ʹ��
        6�����ܺ�����
            TempValue = TempCal(ADCConvertedValue)�������¶�ADCԭʼ��ʽ����תΪ�ȵĵ�λ�ĸ�ʽ
    ʹ�÷�����
        1�������򿪵���ģʽ��������ADCConvertedValue,TempValue��ӵ�watch���ڹ۲�
        2�������ڹ������ӵ�PA9���ţ����򿪴��ڽ��չ���
        3��ȫ�����У����Կ����¶ȱ�������ֵ�ڳ����½ӽ�25�����ң�ͬʱ���ڹ�����ʾʵʱоƬ�ڵ��¶�ֵ
4��ע������
    ��ϵͳ����HSEʱ��ʱ��һ��HSIҲ�Ǵ򿪵ģ���RCC_ConfigAdc1mClk(RCC_ADC1MCLK_SRC_HSE, RCC_ADC1MCLK_DIV8)��������ΪHSE����HSI
    ��ϵͳ����HSIʱ��ʱ��һ��HSE�ǹرյģ���RCC_ConfigAdc1mClk(RCC_ADC1MCLK_SRC_HSI, RCC_ADC1MCLK_DIV8)ֻ������ΪHSI