1������˵��

	1��SPI ���ͽ������ݽ��� CRC У��


2��ʹ�û���

	/* Ӳ�����������̶�Ӧ�Ŀ���Ӳ��ƽ̨ */
	�����壺NS3602_LQFP100_V0.1

3��ʹ��˵��
	
	/* �������ģ�����÷���������:ʱ�ӣ�I/O�� */
	1��SystemClock��108MHz
	2��GPIO��SPI1: SCK-PA5   <--> SPI2: SCK-PB13
			 SPI1: MISO--PA6 <--> SPI2: MISO--PB14
			 SPI1: MOSI--PA7 <--> SPI2: MOSI--PB15


	/* ����Demo�Ĳ��Բ�������� */
	1.��������س���λ���У�
	2.SPI1��SPI2 ͬʱ�շ����ݣ�������ɺ󣬷��� CRC ���ݣ�������ݺ� CRC ֵ���鿴 TransferStatus1 �� TransferStatus2 ״̬Ϊ PASSED��
	�ٲ鿴 CRC ֵ��

4��ע������
	��