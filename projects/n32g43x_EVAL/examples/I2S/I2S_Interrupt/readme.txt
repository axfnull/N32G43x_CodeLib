1������˵��

	1��I2S ͨ���ж��շ�����


2��ʹ�û���

	/* Ӳ�����������̶�Ӧ�Ŀ���Ӳ��ƽ̨ */
	�����壺NS3602_LQFP100_V0.1

3��ʹ��˵��
	
	/* �������ģ�����÷���������:ʱ�ӣ�I/O�� */
	1��SystemClock��108MHz
	2��GPIO��I2S1��MASTER SPI1 WS--PA4��SCK--PA5��MOSI--PA7����
	   I2S2��SLAVE  SPI2 WS--PB12��SCK--PB13�� MOSI--PB15��,
	3���жϣ�I2S1 �ж���ں��� SPI1_IRQHandler��I2S2 �ж���ں���SPI2_IRQHandler


	/* ����Demo�Ĳ��Բ�������� */
	1.��������س���λ���У�
	2.I2S ��ʼ���� 16 λ���ݸ�ʽ��48K ����Ƶ�ʣ�I2S1�����ݷ���ʱ���� SPI1_IRQHandler �����������ݣ�I2S2 �����ݽ���ʱ���� SPI2_IRQHandler ������������ ��������ɺ󣬲鿴 TransferStatus1 ״̬Ϊ PASSED��
	  I2S ��ʼ���� 24 λ���ݸ�ʽ��16K ����Ƶ�ʣ�I2S1�����ݷ���ʱ���� SPI1_IRQHandler �����������ݣ�I2S2 �����ݽ���ʱ���� SPI2_IRQHandler ������������ ��������ɺ󣬲鿴 TransferStatus2 ״̬Ϊ PASSED��

4��ע������
	��