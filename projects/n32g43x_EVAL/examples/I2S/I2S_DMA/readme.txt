1������˵��

	1��I2S ͨ�� DMA �շ�����


2��ʹ�û���

	/* Ӳ�����������̶�Ӧ�Ŀ���Ӳ��ƽ̨ */
	�����壺NS3602_LQFP100_V0.1

3��ʹ��˵��
	
	/* �������ģ�����÷���������:ʱ�ӣ�I/O�� */
	1��SystemClock��108MHz
	2��GPIO��I2S1��MASTER SPI1 WS--PA4��SCK--PA5��MOSI--PA7����
	   I2S2��SLAVE  SPI2 WS--PB12��SCK--PB13�� MOSI--PB15��,
	3��DMA��I2S1 ���ղ��� DMA_CH2 ͨ����I2S2 ���Ͳ��� DMA_CH4 ͨ����


	/* ����Demo�Ĳ��Բ�������� */
	1.��������س���λ���У�
	2.I2S1 ͨ�� DMA_CH2 �������ݣ�I2S2 ͨ�� DMA_CH4 �������� ��������ɺ󣬲鿴 TransferStatus ״̬Ϊ PASSED


4��ע������
	��