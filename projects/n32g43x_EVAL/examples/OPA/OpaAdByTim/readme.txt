1������˵��
    1��TIM1 CH4����ADCע�����OPA1 OPA2�ĵ�ѹ���ݣ�����TIM1�����COMP1ɲ������
2��ʹ�û���
    �������������  KEIL MDK-ARM V5.26.2.0
    Ӳ��������      ����N32L43XM-STB����
3��ʹ��˵��
    ϵͳ���ã�
        1��ʱ��Դ��
            HSE=8M,PLL=108M,AHB=108M,APB1=27M,APB2=54M,ADC CLK=108M/10,ADC 1M CLK=HSE/8,OPA CLK=27M,COMP CLK=27M,TIM1 CLK=108M,TIM8 CLK=108M
        2���жϣ�
            ADCע���жϴ򿪣��������ȼ�0�������ȼ�0
            ��ȡת�����
        3���˿����ã�
            PA4ѡ��Ϊģ�⹦��OPA1 VP
            PA7ѡ��Ϊģ�⹦��OPA2 VP
            PA2ѡ��Ϊģ�⹦��OPA1 OUT
            PA6ѡ��Ϊģ�⹦��OPA2 OUT
            PA0ѡ��Ϊģ�⹦��COMP1 INP
            PB5ѡ��Ϊģ�⹦��COMP1 INM
            PA11ѡ��Ϊģ�⹦��COMP1 OUT	
            PA8ѡ��ΪTIM1 CH1���
            PA9ѡ��ΪTIM1 CH2���
            PA10ѡ��ΪTIM1 CH3���
            PB13ѡ��ΪTIM1 CH1N���
            PB14ѡ��ΪTIM1 CH2N���
            PB15ѡ��ΪTIM1 CH3N���
            PB12ѡ��ΪTIM1 Breakin����
            PA0ѡ��ΪADC1_CH1ͨ������
            PC0ѡ��ΪADC12_CH6ͨ������
        4��OPA��
            OPA1 OPA2��ѹ���������ܣ�VM������
        5��ADC��
            ADCע��ģʽ��ɨ��ת����TIM1 CC4������12λ�����Ҷ��룬ע��ת��OPA1��OPA2���ģ���ѹ����
        6��COMP��
            COMP1 INPѡ��PA0��INMѡ��PB5���������TIM1��TIM8ɲ��
        7��TIM��
            TIM1 6·�����򿪣�ɲ��ʹ���Ҵ��жϣ�CH4�������
    ʹ�÷�����
        1�������򿪵���ģʽ����ʾ���������߼������ǹ۲�TIM1��������κ�ADC��ȡ�Ĳɼ�����
        2���ı�COMP1 �����״̬����ɲ��TIM1���ı�OPA1 OPA2�������ѹ���Ըı�ADC��ȡ������
4��ע������
    ��