1������˵��

    ������չʾ��I2Cģ��10bit��ַģʽ�Ķ�д������   

2��ʹ�û���

	��������������
        IDE���ߣ�KEIL MDK-ARM 5.26
    
    Ӳ��������
        ������ N32L43XM-STB


3��ʹ��˵��
	
    1��ʱ��Դ��HSE+PLL
    2����ʱ�ӣ�108MHz
    3��I2C1�����ã�
            SCL   -->  PB8
            SDA   -->  PB9
            ADDR��0x230(10bit)
            CLOCK:100KHz

    4��I2C2�����ã�
            SCL   -->  PB10
            SDA   -->  PB11
            ADDR��0x2A0(10bit)
            CLOCK:100KHz        
    
    5��USART1���ã�
            TX  -->  PA9
            �����ʣ�115200
            ����λ��8bit
            ֹͣλ��1bit
            ��У��

    6�����Բ���������
        a���öŰ�������PB8-PB10��PB9-PB11
        b���������ش��븴λ����
        c���Ӵ��ڿ���ӡ��Ϣ����֤���

4��ע������
    ��Ҫ��SDA��SCL������������衣