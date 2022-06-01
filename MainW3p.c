#include "stm32f10x.h"                  // Device header
#include "RTE_Components.h"             // Component selection
#include "DMA_STM32F10x.h"              // Keil::Device:DMA
#include "GPIO_STM32F10x.h"             // Keil::Device:GPIO
#include "RTE_Device.h"                 // Keil::Device:Startup
#include "misc.h"                       // Keil::Device:StdPeriph Drivers:Framework
#include "stm32f10x_gpio.h"             // Keil::Device:StdPeriph Drivers:GPIO
#include "stm32f10x_rcc.h"              // Keil::Device:StdPeriph Drivers:RCC
#include "stm32f10x_tim.h"              // Keil::Device:StdPeriph Drivers:TIM
#include "stm32f10x_adc.h"              // Keil::Device:StdPeriph Drivers:ADC
#include "math.h"
#include "ssd1306.h"
#include "flash.h"

#define CntStepFval 25    //кол-во значений для усреднения считывания регулятора оборотов. Чем меньше - тем резче регулируется
#define MinStartVoltage 260 //0 = контроль не испольузется. Минимальное рабочее напряжение (310В пост = 220 переменки! Указывается постоянка!, или переменка * 1.4)
int MotorAmperage = 0;  //0 = контроль не испольщуется. Максимальный рабочий ток двигателя 

//Тип управления запуска частотника:
//  ControlSwitch = работает пока нажат пуск; 
//  ControlTwoButtons = управление кнопками пуск + стоп
#define ControlType ControlTwoButtons 

//Скорости разгона-торможения
int StartSpeed = 400; // 1-1000
int StopSpeed = 200;  // 1-1000;  0 = торможение на выбеге. Если тормозить слишком быстро... это можеть быть опасно =)

//РАСПИНОВКА!
#define piStart GPIOB, GPIO_Pin_8 //Старт
#define piStop GPIOB, GPIO_Pin_9 //Стоп
#define piRev GPIOB, GPIO_Pin_7 //Реверс
//#define piPlus GPIOB, GPIO_Pin_6 //+
//#define piMinus GPIOB, GPIO_Pin_5 //-
//#define piSetts GPIOB, GPIO_Pin_4 //Стоп
#define piError GPIOB, GPIO_Pin_12 //Сигнал ошибки

#define poFanOn GPIOA, GPIO_Pin_1 //Включение вентилятора
#define poSoftStart GPIOA, GPIO_Pin_2 // реле мягкого старта
#define poError GPIOA, GPIO_Pin_0 //лампа ошибки
#define poBrake GPIOA, GPIO_Pin_3 // Включение тормозного резистора 
#define poStart GPIOC, GPIO_Pin_15 //Индикация пуска
#define poStop GPIOC, GPIO_Pin_13 //Индикация Стопа
#define poRev GPIOC, GPIO_Pin_14 //Индикация Реверса

//Служебное
#define PI 				 3.14159 //В военное время может достигать 4
#define TIM_PERIOD 125    
#define    DWT_CYCCNT    *(volatile unsigned long *)0xE0001004
#define    DWT_CONTROL   *(volatile unsigned long *)0xE0001000
#define    SCB_DEMCR     *(volatile unsigned long *)0xE000EDFC

#define ModuleError !GPIO_ReadInputDataBit(piError)
#define StartPressed !GPIO_ReadInputDataBit(piStart)
#define StartReleased GPIO_ReadInputDataBit(piStart)
#define StopPressed !GPIO_ReadInputDataBit(piStop)
#define StopReleased GPIO_ReadInputDataBit(piStop)

#define ControlTwoButtons 1 //кнопка старт + стоп
#define ControlSwitch 2 //работает пока нажата старт

int mTIM_Prescaler;  
#define iTIM_STEPS 510 //144  510=5гц=2,5Кгц
uint16_t TIM_STEPS = iTIM_STEPS;
uint16_t LastTIM_STEPS = 0;
bool NeedUpdateHz = false;
bool ErrorState = false;
uint16_t FSetVals[CntStepFval];
int nSetVal = 0;
bool NeedFillSinArray = true;

int CurHz = 0; //фактическая частота * 10
int SetHz = 0; //Установленная частота * 10

int CurTemp = 0;
int CurVolt = 0;

#define cntAmpVals 38
uint16_t AmpVals[cntAmpVals];
int nSetAmp = 0;
int CurrAmp = 0; // значение x10!
int ampOffset = 0;
 
bool IsRev = false; //Фаг обратного хода

#define mMain 0
#define mStart 1
#define mStop 2

int menuMode = 0;
bool HasChangesMenu = false;

/*********************************************************************/ 
 //---Фаза А ---// 
 uint16_t sinA[iTIM_STEPS] = {0}; //{56,84,113,141,169,197,225,253,281,309,337,365,392, 420,447,474,502,529,556,583,609,636,662,688,714,740,766,791,817,842,867, 891,916,940,964,988,1011,1035,1058,1080,1103,1125,1147,1169,1190,1211, 1232,1252,1272,1292,1312,1331,1350,1368,1386,1404,1422,1439,1456,1472, 1488,1504,1519,1534,1549,1563,1577,1590,1603,1616,1628,1640,1651,1662, 1673,1683,1693,1702,1711,1720,1728,1736,1743,1750,1756,1762,1768,1773, 1777,1782,1785,1789,1792,1794,1796,1798,1799,1799,1800,1799,1799,1798, 1796,1794,1792,1789,1785,1782,1777,1773,1768,1762,1756,1750,1743,1736, 1728,1720,1711,1702,1693,1683,1673,1662,1651,1640,1628,1616,1603,1590, 1577,1563,1549,1534,1519,1504,1488,1472,1456,1439,1422,1404,1386,1368, 1350,1331,1312,1292,1272,1252,1232,1211,1190,1169,1147,1125,1103,1080, 1058,1035,1011,988,964,940,916,891,867,842,817,791,766,740,714,688,662, 636,609,583,556,529,502,474,447,420,392,365,337,309,281,253,225,56};
 //---Фаза B ---// 
 uint16_t sinB[iTIM_STEPS] = {0}; //{1529,1514,1499,1483,1467,1450,1433,1416,1398,1380, 1362,1343,1324,1305,1286,1266,1245,1225,1204,1183,1161,1140,1118,1095, 1073,1050,1027,1003,980,956,932,908,883,858,833,808,783,757,732,706,680, 653,627,600,574,547,520,493,465,438,411,383,355,328,300,272,244,216,188, 160,131,103,75,47,18,9,37,65,94,122,150,178,206,234,262,290,318,346,374, 401,429,456,484,511,538,565,591,618,645,671,697,723,749,774,800,825,850, 875,900,924,948,972,996,1019,1042,1065,1088,1110,1132,1154,1176,1197, 1218,1239,1259,1279,1299,1318,1337,1356,1374,1392,1410,1428,1445,1461, 1478,1494,1509,1524,1539,1568,1581,1595,1608,1620,1632,1644,1655,1666, 1677,1687,1696,1705,1714,1723,1731,1745,1752,1758,1764,1769,1774,1779, 1783,1786,1790,1792,1795,1797,1798,1799,1799,1799,1799,1798,1797,1795, 1793,1791,1788,1784,1780,1776,1771,1766,1760,1748,1741,1733,1725,1717, 1708,1699,1690,1680,1670,1659,1529};
 //---Фаза C ---// 
 uint16_t sinC[iTIM_STEPS] = {0};//{1599,1612,1624,1636,1648,1659,1670,1680,1690,1699, 1708,1717,1725,1733,1741,1748,1754,1760,1766,1771,1776,1780,1784,1788, 1791,1793,1795,1797,1798,1799,1799,1799,1799,1798,1797,1795,1792,1790, 1786,1783,1779,1774,1769,1764,1758,1752,1745,1738,1731,1723,1714,1705, 1696,1687,1677,1666,1655,1644,1632,1620,1608,1595,1581,1568,1554,1539, 1524,1509,1494,1478,1461,1445,1428,1410,1392,1374,1356,1337,1318,1299, 1279,1259,1239,1218,1197,1176,1154,1132,1110,1088,1065,1042,1019,996, 972,948,924,900,875,850,825,800,774,749,723,697,671,645,618,591,565,538, 511,484,456,429,401,374,346,318,290,262,234,206,178,150,122,94,65,37,9, 18,47,75,103,131,160,188,216,244,272,300,328,355,383,411,438,465,493, 520,547,574,600,627,653,680,706,732,757,783,808,833,858,883,908,932,956, 980,1003,1027,1050,1073,1095,1118,1140,1161,1183,1204,1225,1245,1266, 1286,1305,1324,1343,1362,1380,1398,1416,1433,1586,1586};
 //Предрасчитанный массив:
 uint8_t sinM[iTIM_STEPS] = {0};
 /**********************************************************************/ 

//Инициализация АЦП регулятора оборотов на B1
void adc_init() 
{
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC2, ENABLE);

	// настройки ADC
	ADC_InitTypeDef ADC_InitStructure;
	ADC_StructInit(&ADC_InitStructure);
	ADC_InitStructure.ADC_Mode = ADC_Mode_Independent; // режим работы - одиночный, независимый
	ADC_InitStructure.ADC_ScanConvMode = DISABLE; // не сканировать каналы, просто измерить один канал
	ADC_InitStructure.ADC_ContinuousConvMode = DISABLE; // однократное измерение
	ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None; // без внешнего триггера
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right; //выравнивание битов результат - прижать вправо
	ADC_InitStructure.ADC_NbrOfChannel = 1; //количество каналов - одна штука
	ADC_Init(ADC2, &ADC_InitStructure);
	ADC_Cmd(ADC2, ENABLE);

	// калибровка АЦП
	ADC_ResetCalibration(ADC2);
	while (ADC_GetResetCalibrationStatus(ADC2));
	ADC_StartCalibration(ADC2);
	while (ADC_GetCalibrationStatus(ADC2));
	
}

int analogRead(uint8_t Chan)
{
	ADC_RegularChannelConfig(ADC2, Chan, 1, ADC_SampleTime_28Cycles5); 
	ADC_SoftwareStartConvCmd(ADC2, ENABLE);
	while(ADC_GetFlagStatus(ADC2, ADC_FLAG_EOC) == RESET);
	return ADC_GetConversionValue(ADC2);
}
//Инициализация АЦП контроля силового напряжения
void adcVoltage_init() //Напряжение на А6
{
//	if (MinStartVoltage == 0) return; //Значит контроль напряжения не используется.
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);
	
		//NVIC
    NVIC_InitTypeDef NVIC_InitStructure;
    NVIC_InitStructure.NVIC_IRQChannel = ADC1_2_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
 
    //ADC
    ADC_InitTypeDef ADC_InitStructure;
    ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;
    ADC_InitStructure.ADC_ScanConvMode = DISABLE;
    ADC_InitStructure.ADC_ContinuousConvMode = ENABLE; 
    ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
    ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
    ADC_InitStructure.ADC_NbrOfChannel = 1;
    ADC_Init (ADC1, &ADC_InitStructure);    
 
    ADC_RegularChannelConfig(ADC1, ADC_Channel_6, 1, ADC_SampleTime_239Cycles5);
 
	/* Ограничители низкого и высокого уровня напряжений*/
  	ADC_AnalogWatchdogThresholdsConfig(ADC1, 330 * 8.2733, 0); //Безопасный диапазон в В * коэф АЦП = 8.2733
    ADC_AnalogWatchdogSingleChannelConfig(ADC1, ADC_Channel_6);
    ADC_AnalogWatchdogCmd(ADC1, ADC_AnalogWatchdog_SingleRegEnable);
 
    /* Enable AWD interrupt */
    ADC_ITConfig(ADC1, ADC_IT_AWD, ENABLE);
 
    // Enable ADC
    ADC_Cmd (ADC1, ENABLE); //enable ADC1
 
    //  ADC calibration (optional, but recommended at power on)
    ADC_ResetCalibration(ADC1); // Reset previous calibration
    while(ADC_GetResetCalibrationStatus(ADC1));
    ADC_StartCalibration(ADC1); // Start new calibration (ADC must be off at that time)
    while(ADC_GetCalibrationStatus(ADC1));
 
    // start conversion
    ADC_Cmd (ADC1,ENABLE);  //enable ADC1
    ADC_SoftwareStartConvCmd(ADC1, ENABLE); // start conversion (will be endless as we are in continuous mode)
}

//Иноциализация пинов генерации ШИМа
void initIO(void)
{
	 //------Назначение GPIO------------- 
	 //-----------Ввод структур--------------// 
	 GPIO_InitTypeDef gpio;
	 GPIO_InitTypeDef gpio_n;

 //------------------------------------
	GPIO_StructInit(&gpio); // GPIO A - Tim1 Channel 1P, 2P, 3P, Enable
	gpio.GPIO_Mode = GPIO_Mode_AF_OD;
	gpio.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_10;
	gpio.GPIO_Speed = GPIO_Speed_10MHz;
	GPIO_Init(GPIOA, &gpio);
 //------------------------------------ 
	GPIO_StructInit(&gpio_n); /* GPIOB Configuration: Channel 1N, 2N , 3N as alternate function push-pull */ 
	gpio_n.GPIO_Pin = GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15; 
	gpio_n.GPIO_Mode = GPIO_Mode_AF_OD; 
	gpio_n.GPIO_Speed = GPIO_Speed_10MHz; 
	GPIO_Init(GPIOB, &gpio_n);

}

void InitIOPWM2Manual()
{
	GPIO_InitTypeDef gpioP;
	GPIO_InitTypeDef gpioN;
	
		//Выходы для прямого управления модулем (отключение)
	GPIO_StructInit(&gpioP); // GPIO A - Tim1 Channel 1P, 2P, 3P
	gpioP.GPIO_Mode = GPIO_Mode_Out_OD;
	gpioP.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_10;
	gpioP.GPIO_Speed = GPIO_Speed_10MHz;
	GPIO_Init(GPIOA, &gpioP);
	
	GPIO_StructInit(&gpioN); // GPIO B - Tim1 Channel 1N, 2N, 3N
	gpioN.GPIO_Mode = GPIO_Mode_Out_OD;
	gpioN.GPIO_Pin = GPIO_Pin_13|GPIO_Pin_14|GPIO_Pin_15;
	gpioN.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &gpioN);	
}
//Инициализация основных пинов управления. Ручное управление портами
void InitIO2Manual()
{
	GPIO_InitTypeDef gpiom;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB | RCC_APB2Periph_GPIOC | RCC_APB2Periph_AFIO | RCC_APB2Periph_TIM1, ENABLE); //----------Включение тактирования-------// 

	//Общие входы-выходы
	GPIO_StructInit(&gpiom); //лампа ошибки,Включение вентилятора,реле мягкого старта,Включение тормозного резистора 
	gpiom.GPIO_Mode = GPIO_Mode_Out_PP;
	gpiom.GPIO_Pin = GPIO_Pin_0 |  GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3;
	gpiom.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &gpiom);
													// GPIO С - Индикация Стопа,Индикация Реверса,Индикация пуска
	gpiom.GPIO_Mode = GPIO_Mode_Out_PP;
	gpiom.GPIO_Pin = GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;
	gpiom.GPIO_Speed = GPIO_Speed_10MHz;
	GPIO_Init(GPIOC, &gpiom);

	// GPIO A - Fault, 
	gpiom.GPIO_Mode = GPIO_Mode_IPU;
	gpiom.GPIO_Pin = GPIO_Pin_12;
	gpiom.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &gpiom);
	
	// GPIO B - Start, Stop (кнопки),Реверс, + - set
	gpiom.GPIO_Mode = GPIO_Mode_IPU;
	gpiom.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_7;// | GPIO_Pin_6 | GPIO_Pin_5 | GPIO_Pin_1 | GPIO_Pin_7;
	gpiom.GPIO_Speed = GPIO_Speed_10MHz;
	GPIO_Init(GPIOB, &gpiom);
}

void timers(void) { 
 //-----------Ввод структур--------------// 
 TIM_TimeBaseInitTypeDef TIM_Base_1;
 TIM_OCInitTypeDef TIM_PWM;

 //-----Назначение TIM1---------------- 
 TIM_TimeBaseStructInit(&TIM_Base_1);
 TIM_Base_1.TIM_Prescaler = mTIM_Prescaler; //18
 TIM_Base_1.TIM_CounterMode = TIM_CounterMode_Up;
 TIM_Base_1.TIM_Period = TIM_PERIOD;
 TIM_Base_1.TIM_ClockDivision = TIM_CKD_DIV1;
 TIM_TimeBaseInit(TIM1,&TIM_Base_1);
	
 //-------Назначение ШИМ------------------- TIM_OCStructInit(&TIM_PWM);
 TIM_PWM.TIM_OCMode = TIM_OCMode_PWM1;
	
 TIM_PWM.TIM_OutputState = TIM_OutputState_Enable;
 TIM_PWM.TIM_OutputNState = TIM_OutputNState_Enable;
	
 TIM_PWM.TIM_OCNPolarity =  TIM_OCNPolarity_Low;
 TIM_PWM.TIM_OCPolarity =  TIM_OCPolarity_Low;
	
 TIM_OC1Init(TIM1, &TIM_PWM);
 TIM_OC1PreloadConfig(TIM1, TIM_OCPreload_Enable);
 TIM_OC2Init(TIM1, &TIM_PWM);
 TIM_OC2PreloadConfig(TIM1, TIM_OCPreload_Enable);
 TIM_OC3Init(TIM1, &TIM_PWM);
 TIM_OC3PreloadConfig(TIM1, TIM_OCPreload_Enable);
 TIM_Cmd(TIM1, ENABLE);
 TIM_CtrlPWMOutputs(TIM1, ENABLE);
 //разнесение инверсионных выходов
	TIM_BDTRInitTypeDef TIM_BDTRInitStructure;
	TIM_BDTRInitStructure.TIM_OSSRState = TIM_OSSRState_Enable;
	TIM_BDTRInitStructure.TIM_OSSIState = TIM_OSSIState_Enable;
	TIM_BDTRInitStructure.TIM_LOCKLevel = TIM_LOCKLevel_1;
	TIM_BDTRInitStructure.TIM_DeadTime = 250; //Много - не мало. Но мало - дорого =)
	TIM_BDTRInitStructure.TIM_Break = TIM_Break_Disable;
	TIM_BDTRInitStructure.TIM_BreakPolarity = TIM_BreakPolarity_High;
	TIM_BDTRInitStructure.TIM_AutomaticOutput = TIM_AutomaticOutput_Enable;
	TIM_BDTRConfig(TIM1, &TIM_BDTRInitStructure);
 } 
//////Расчёт таблиц синуса
static void sin_table_update ()
{	
		double K;
		if (CurHz < 40)
			K = 0.08;
		else if (CurHz < 110) 
			K = 0.15;
		else
			K = (double)CurHz/950; //!!!
		if (K > 0.5) K = 0.5;
		
//		if (K < 0.45) //!!!
//		{
//			K = (TIM_PERIOD * K) * 1.1;
//			for(uint16_t i = 0; i < TIM_STEPS; ++i)
//			{
//				sinM[i] = (uint16_t)(K * (1 + sin(PI * (2.02 * i / (double)TIM_STEPS))));
//			}
//		}
//		else
//		{
			K = TIM_PERIOD * K * 1.15;
			for(uint16_t i = 0; i < TIM_STEPS; ++i)
			{
				sinM[i] = (uint16_t)(K * (0.87 + sin(PI * (2.02 * i / (double)TIM_STEPS)) + 0.18 * sin(PI * (2.02 * i*3 / (double)TIM_STEPS))));
			}
//		}              
}

//Инициализация каналов DMA для каждой фазы
void sinDMA_PhaseA(void) { 
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
	DMA_InitTypeDef DMA_struct;
	DMA_StructInit(&DMA_struct);
	DMA_struct.DMA_PeripheralBaseAddr =(uint32_t)&TIM1->CCR1;
	DMA_struct.DMA_MemoryBaseAddr = (uint32_t)&sinA[0];
	DMA_struct.DMA_DIR=DMA_DIR_PeripheralDST;
	DMA_struct.DMA_BufferSize=TIM_STEPS;
	DMA_struct.DMA_PeripheralInc=DMA_PeripheralInc_Disable;
	DMA_struct.DMA_MemoryInc=DMA_MemoryInc_Enable;
	DMA_struct.DMA_PeripheralDataSize=DMA_PeripheralDataSize_HalfWord;
	DMA_struct.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
	DMA_struct.DMA_Mode=DMA_Mode_Circular;
	DMA_struct.DMA_Priority = DMA_Priority_High;
	DMA_Init(DMA1_Channel2,&DMA_struct);
	DMA_Cmd(DMA1_Channel2, ENABLE);
	TIM_DMACmd(TIM1,TIM_DMA_CC1, ENABLE);

	DMA_ITConfig(DMA1_Channel2, DMA_IT_TC, ENABLE);
	NVIC_EnableIRQ(DMA1_Channel2_IRQn);
 } 

void sinDMA_PhaseB(void) { 
	DMA_InitTypeDef DMA_struct1;
	DMA_StructInit(&DMA_struct1);
	DMA_struct1.DMA_PeripheralBaseAddr =(uint32_t)&TIM1->CCR2;
	DMA_struct1.DMA_MemoryBaseAddr = (uint32_t)&sinB[0];
	DMA_struct1.DMA_DIR=DMA_DIR_PeripheralDST;
	DMA_struct1.DMA_BufferSize=TIM_STEPS;
	DMA_struct1.DMA_PeripheralInc=DMA_PeripheralInc_Disable;
	DMA_struct1.DMA_MemoryInc=DMA_MemoryInc_Enable;
	DMA_struct1.DMA_PeripheralDataSize=DMA_PeripheralDataSize_HalfWord;
	DMA_struct1.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
	DMA_struct1.DMA_Mode=DMA_Mode_Circular;
	DMA_struct1.DMA_Priority = DMA_Priority_High;
	DMA_Init(DMA1_Channel3,&DMA_struct1);
	DMA_Cmd(DMA1_Channel3, ENABLE);
	TIM_DMACmd(TIM1,TIM_DMA_CC2, ENABLE);
 } 
void sinDMA_PhaseC(void) { 
	DMA_InitTypeDef DMA_struct2;
	DMA_StructInit(&DMA_struct2);
	DMA_struct2.DMA_PeripheralBaseAddr =(uint32_t)&TIM1->CCR3;
	DMA_struct2.DMA_MemoryBaseAddr = (uint32_t)&sinC[0];
	DMA_struct2.DMA_DIR=DMA_DIR_PeripheralDST;
	DMA_struct2.DMA_BufferSize=TIM_STEPS;
	DMA_struct2.DMA_PeripheralInc=DMA_PeripheralInc_Disable;
	DMA_struct2.DMA_MemoryInc=DMA_MemoryInc_Enable;
	DMA_struct2.DMA_PeripheralDataSize=DMA_PeripheralDataSize_HalfWord;
	DMA_struct2.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
	DMA_struct2.DMA_Mode=DMA_Mode_Circular;
	DMA_struct2.DMA_Priority = DMA_Priority_High;
	DMA_Init(DMA1_Channel6,&DMA_struct2);
	DMA_Cmd(DMA1_Channel6, ENABLE);
	TIM_DMACmd(TIM1,TIM_DMA_CC3, ENABLE);
 }


void DMA1_Channel2_IRQHandler(void)
{
	if (NeedUpdateHz)
	{
		TIM_Cmd(TIM1, DISABLE);
		DMA_Cmd(DMA1_Channel6, DISABLE);
		DMA_Cmd(DMA1_Channel3, DISABLE);
		DMA_Cmd(DMA1_Channel2, DISABLE);
		
		if (TIM_STEPS != LastTIM_STEPS)
		{
			LastTIM_STEPS = TIM_STEPS;
			int oA = TIM_STEPS / 3;
			int oB = oA;
			
			if (IsRev)
				 oB = oA + oA;
			else
				 oA = oA + oA;
			
			for (int i = 0; i < TIM_STEPS; i++)
			{
					sinA[i] = sinM[i];
					sinB[i] = sinM[i + oA < TIM_STEPS ? i + oA : i - TIM_STEPS + oA];
					sinC[i] = sinM[i + oB < TIM_STEPS ? i + oB : i - TIM_STEPS + oB];
			} 
			
			DMA1_Channel6->CNDTR =TIM_STEPS;
			DMA1_Channel3->CNDTR =TIM_STEPS;
			DMA1_Channel2->CNDTR =TIM_STEPS;
		}

		TIM1->PSC = mTIM_Prescaler;
		
		DMA_Cmd(DMA1_Channel6, ENABLE);
		DMA_Cmd(DMA1_Channel3, ENABLE);
		DMA_Cmd(DMA1_Channel2, ENABLE);
		TIM_Cmd(TIM1, ENABLE);
		
		NeedUpdateHz = false;
	}
	
	DMA_ClearITPendingBit(DMA1_IT_TC2);
}

 ////////----------------------------------- Логика
 int Abs(int i1)
 {
	 if (i1 < 0)
		 return i1 * -1;
	 else 
		 return i1;
 }
 
 //Функция задержки
 void delay_ms(uint32_t ms)
{
	int32_t ms_count_tick =  ms * (SystemCoreClock/1000);
	//разрешаем использовать счётчик
	SCB_DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
         //обнуляем значение счётного регистра
	DWT_CYCCNT  = 0;
        //запускаем счётчик
	DWT_CONTROL|= DWT_CTRL_CYCCNTENA_Msk; 
	while(DWT_CYCCNT < ms_count_tick);
        //останавливаем счётчик
	DWT_CONTROL &= ~DWT_CTRL_CYCCNTENA_Msk;
}

//контроль тока. 
bool AmpControlOK()
{
	double ret;
	ret = analogRead(ADC_Channel_7);
	if (ret < 0) ret = 0;
	int Curr = MotorAmperage * 10; 
	
	//Усреднитель значений
	AmpVals[nSetAmp] = ret;
	nSetAmp++;
	if (nSetAmp == cntAmpVals)
		nSetAmp = 0;
	
	ret = 0;
	for(int i = 0; i < cntAmpVals; i++)
		ret += AmpVals[i];

	ret = ret / cntAmpVals;
	
	if(!GPIO_ReadInputDataBit(poStart))
	{
		ampOffset = ret;
		return true;
	}
	else
		ret = ret - ampOffset;
	//
	CurrAmp = ret * -1.21f; // А * 10
	if (MotorAmperage == 0 || !GPIO_ReadInputDataBit(poStart)) return true;
	return (ret < Curr); //типа ток не превышен
}

//замер значения регулятора частоты и приведение его к значениям для делителя таймера
int SeekHzVal()
{
	 int ret;
	 ret = analogRead(ADC_Channel_9); //Крутилка на B1
	 ret = 48 + ((ret * 950) / 4096);
	 return ret;
}

int GetValFriq()
{
	int ret = SeekHzVal();
	
	if (!AmpControlOK() || ModuleError)
	{
		ret = 30;
		delay_ms(10);
	}
	
	//Усреднитель значений
	FSetVals[nSetVal] = ret;
	nSetVal++;
	if (nSetVal == CntStepFval)
		nSetVal = 0;
	
	ret = 0;
	for(int i = 0; i < CntStepFval; i++)
		ret += FSetVals[i];

	ret = ret / CntStepFval;
	//
	
	return ret;
}

//Изменение  частоты 
void SetPrescaler(int val)
{
	CurHz = val;
	
	//Вычисляем длину массива
	if (CurHz > 100 && mTIM_Prescaler > 0)
	{
		int h = ((int)(53000 / CurHz) / 3) * 3;
		if (TIM_STEPS+8 > h || TIM_STEPS-8 < h)	TIM_STEPS = h;
	}
	else
		TIM_STEPS = iTIM_STEPS;
	
	//Вычисялем делитель
	mTIM_Prescaler = (double)SystemCoreClock/((double)CurHz/10)/(double)(TIM_PERIOD * TIM_STEPS);
	
	//Пересчитываем синус под новые парамтеры
	sin_table_update();
	
	NeedUpdateHz = true;
}

//Метод остановки частотника. параметр - используется ли плавная остановка (true) или выбег мотора (false)
void Stop(bool PowerStop) 
{
	if (PowerStop && StopSpeed > 0)
	{
		
		//Торможение мотора
		int ErrCycle = 0;
		int i;
		for (i = CurHz; i > 50; i--) 
		{
			UpdateLCD(mStop);
			SetPrescaler(i);	
			GPIO_ResetBits(poBrake);
			
			if (ModuleError)
			{
				ErrCycle ++;
				GPIO_SetBits(poError);
				if (ErrCycle > 50)
					break;
			}
			else
			{
				GPIO_ResetBits(poError);

				if (i > StopSpeed/24)
					i -= StopSpeed/24; //чем меньше делитель - тем больше шаг торможения
			}
			delay_ms(1000/StopSpeed);
		}
		GPIO_ResetBits(poBrake);
	}
	CurrAmp = 0;
	delay_ms(10);
	DMA_Cmd(DMA1_Channel6, DISABLE);
	DMA_Cmd(DMA1_Channel3, DISABLE);
	DMA_Cmd(DMA1_Channel2, DISABLE);
	TIM_Cmd(TIM1, DISABLE);

	
//	
	GPIO_ResetBits(poStart);
	GPIO_SetBits(poStop);
	
	InitIOPWM2Manual();
	LastTIM_STEPS = 0;
	GPIO_ResetBits(GPIOB,GPIO_Pin_13); //p
	GPIO_ResetBits(GPIOB,GPIO_Pin_14); //p
	GPIO_ResetBits(GPIOB,GPIO_Pin_15); //p
	delay_ms(1);
	GPIO_SetBits(GPIOA,GPIO_Pin_8);    //n
	GPIO_SetBits(GPIOA,GPIO_Pin_9);    //n
	GPIO_SetBits(GPIOA,GPIO_Pin_10);   //n
	delay_ms(300);
	GPIO_SetBits(GPIOB,GPIO_Pin_13); //p
	GPIO_SetBits(GPIOB,GPIO_Pin_14); //p
	GPIO_SetBits(GPIOB,GPIO_Pin_15); //p
	delay_ms(1);
	GPIO_ResetBits(GPIOA,GPIO_Pin_8);    //n
	GPIO_ResetBits(GPIOA,GPIO_Pin_9);    //n
	GPIO_ResetBits(GPIOA,GPIO_Pin_10);   //n
	
	GPIO_ResetBits(poBrake);
}

//Переход в состояние ошибки
void DoErrorState()
{
		GPIO_SetBits(poError);
		ErrorState = true;
		Stop(false);
	
		SSD1306_GotoXY(17, 16); //Устанавливаем курсор в позицию Сначала по горизонтали, потом вертикали.
		SSD1306_Puts("Error!", &Font_16x26, SSD1306_COLOR_WHITE); 
		SSD1306_UpdateScreen();
	
		delay_ms(1000);
		GPIO_ResetBits(poBrake);
		CurHz = 0;
}

//Корректор частоты при перегрузке
void OverloadCorrection()
{
				int ErrCycle = 0;
				for (int i = 0; i < 1500; i ++) 
				{
					if (ModuleError)
					{
						ErrCycle ++;
						GPIO_SetBits(poError);
						if (CurHz > 150) SetPrescaler(CurHz - 15);
						if (ErrCycle > 80)
						{
							DoErrorState();
							break;
						}
					}
					else
					{
						if (CurHz < SetHz) SetPrescaler(CurHz + 3);
						delay_ms(20);
						GPIO_ResetBits(poError);
						if (CurHz >= SetHz) return;
					}
					//Не забываем проверять кнопку стоп!
					if (ToStop())
					{
						Stop(true);
						delay_ms(300);
						break;
					}
				}
				if (!ErrorState)
					GPIO_ResetBits(poError);
}


//прерывание по значению питания. Значит, что-то идёт не так.
void ADC1_2_IRQHandler(void) 
{
   if(ADC_GetITStatus(ADC1, ADC_IT_AWD) == SET)
   {
		 int val = ADC_GetConversionValue(ADC1);
		 
		 CurVolt += (val - CurVolt)/10;
		 
		 val = CurVolt;
	
		 if (val > 370 * 8.2733 ) //3170 390+ вольт. Тормозной резистор не справился. Тупо вырубаем всё и нехай на выбеге останавливается.
		 {
			 DoErrorState(2);
		 }
		 else if (val > 340 * 8.2733) // 2900 350+ вольт  - включаем тормозной резистор и вырубаем реле запуска
		 {
				GPIO_SetBits(poBrake);
		 }
		 else if (val < 338 * 8.2733) // Выключаем тормозной резистор 
		 {
				GPIO_ResetBits(poBrake);
		 }
//		 else if (MinStartVoltage > 0) //Эти части только при включённом контроле минимального напряжения
//		 {			 
//				 if (val > (MinStartVoltage * 8.2733) * 0.85) //Конденсаторы заряжены до 85% мин напряжения старта. Замыкаем реле старта
//				 {
//					 GPIO_SetBits(poSoftStart);
//				 }
//				 else if (val > 100 && val < (MinStartVoltage * 8.2733) * 0.5) //Заряжаем конденсаторы мягким стартом
//				 {
//						GPIO_ResetBits(poSoftStart);
//				 }
//				 else if (val > 100 && val < 2813) //340- вольт - гасим тормозной резистор
//				 {
//					 GPIO_ResetBits(poBrake);
//					 GPIO_SetBits(poSoftStart);
//				 }
//		}
		 
		 ADC_ClearITPendingBit(ADC1,ADC_IT_AWD);
   }  
}

//Проверка на старт
bool ToStart()
{
		//свои основные циклы в зависимости от типа запуска
	#if (ControlType == ControlSwitch || ControlType == ControlTwoButtons)
		if (StartPressed && !GPIO_ReadInputDataBit(poStart) && !ErrorState)
		{
			delay_ms(10);
			return StartPressed; //проверка - может проскочила помеха? Против спонтанного запуска
		}
		return false;
	#endif
}
//Проверка на стоп
bool ToStop()
{
	//свои основные циклы в зависимости от типа запуска
	#if (ControlType == ControlSwitch)

		if (StartReleased && (GPIO_ReadInputDataBit(poStart) || ErrorState))
		{
			delay_ms(10);
			return StartReleased; //проверка - может проскочила помеха? Против спонтанной остановки
		}
		return false;
	#endif
	#if (ControlType == ControlTwoButtons)
		if (StopPressed)
		{
			delay_ms(10);
			return StopPressed; //проверка - может проскочила помеха? Против спонтанной остановки
		}
		return false;
	#endif
}
//Метод запуска частотника
void Start()
{
	int i, val;
	
	
////////// инициализация 
	GPIO_SetBits(poSoftStart);
	GPIO_ResetBits(poStop);
	
	bool b;
	for(i = 0; i < cntAmpVals; i++)
		b = AmpControlOK();
	
	for(i = 0; i < CntStepFval; i++) //фильтр на CntStepFval значений. Надобно заполнить
		val = GetValFriq();
	
	//0.5 - 100% напряжения на выходе. 0.25 - 50%
	SetPrescaler(25);
	DMA1_Channel2_IRQHandler(); //дабы пересчитало массив.
	
	timers();
	initIO();
	
	sinDMA_PhaseA();
	sinDMA_PhaseB();
	sinDMA_PhaseC();
	
//////////

	GPIO_SetBits(poStart);
	delay_ms(300);
	//Выход на рабочие
	int ErrCycle = 0;
	
	for (i = 25; i < val; i+= 3) 
	{
		UpdateLCD(mStart);
		
		if (ModuleError)
		{
			if (i > 25) i-= 3;
				GPIO_SetBits(poError);
		}
		else
		{
			GPIO_ResetBits(poError);
			
			
			SetPrescaler(i);
			
			if (AmpControlOK())
				i += 2;
			else
				delay_ms(200);
		}
		
		//Проверяемся на стоп
		if (ToStop())
		{
			Stop(true);
			delay_ms(300);
			return;
		}
		delay_ms(1000/StartSpeed);
	}
}

//Метод измерения частоты
void SetFrequency()
{
	int nVal = GetValFriq();
	
	if (Abs(nVal - SetHz) > 1) //минимальные изменнеия игнорируем. Возможно, тупо дребезг
	{
		SetHz = nVal;
		
		SetPrescaler(nVal);
				
		delay_ms(50);
	}
}

//запуск частотника - зарядка конденсаторов при подаче питания
void PowerInit()
{
	GPIO_SetBits(poError);
	
	int i = 0;
	while((MinStartVoltage == 0 || analogRead(ADC_Channel_6) < MinStartVoltage * 8.2733)) //Либо по датчику напряжения, ну либо по таймеру
	{
		if (i > 15) break;
		if (i%2==0)
			GPIO_SetBits(poError);
		else
			GPIO_ResetBits(poError);
		delay_ms(300);
		i++;
		
	}
	GPIO_ResetBits(poError);
	ErrorState = false;
	GPIO_SetBits(poSoftStart);
}

void UpdateLCD(int wType)
{
	SSD1306_Fill(SSD1306_COLOR_BLACK);
	char buf[8];
	
	switch (menuMode)
	{
		case 0:
			SSD1306_GotoXY(3, 0); //Устанавливаем курсор в позицию Сначала по горизонтали, потом вертикали.
		
			sprintf(buf, "%.1fA", (float)CurrAmp/10.0f);
			SSD1306_Puts(&buf, &Font_11x18, SSD1306_COLOR_WHITE); 
			SSD1306_GotoXY(83, 0); //Устанавливаем курсор в позицию Сначала по горизонтали, потом вертикали.
			//sprintf(buf, "%.0fW", CurrAmp * 22.0f);
		  //sprintf(buf, "%.0f", (float)SystemCoreClock / (float)mTIM_Prescaler / (float)TIM_PERIOD);
		  //sprintf(buf, "%.0f", (float)TIM_STEPS);
		  sprintf(buf, "%.0fC", (float)CurTemp/10);
			SSD1306_Puts(&buf, &Font_11x18, SSD1306_COLOR_WHITE); 
			SSD1306_GotoXY(6, 23); //Устанавливаем курсор в позицию Сначала по горизонтали, потом вертикали.
		
		  SSD1306_DrawRectangle(1,19,127,63,SSD1306_COLOR_WHITE);
			
			float Hz = 0;
			if ( wType != mMain)
			{
				Hz = (float)CurHz/10;
			}
			else
			{
				if (abs(SetHz-SeekHzVal()) > 1)
					SetHz = SeekHzVal();
				Hz = (float)SetHz/10;
			}
			
			sprintf(buf, "%.1f Hz", Hz);
			SSD1306_Puts(&buf, &Font_16x26, SSD1306_COLOR_WHITE);
		
			SSD1306_GotoXY(9, 50); //Устанавливаем курсор в позицию Сначала по горизонтали, потом вертикали.
			if (GPIO_ReadInputDataBit(poStart))
			{
				if (!IsRev)
					if (wType == mStart)
						SSD1306_Puts(">> Start", &Font_7x10, SSD1306_COLOR_WHITE);
					else if (wType == mStop)
						SSD1306_Puts(">> Stop", &Font_7x10, SSD1306_COLOR_WHITE);
					else
						SSD1306_Puts(" Forward", &Font_7x10, SSD1306_COLOR_WHITE);
				else
					if (wType == mStart)
						SSD1306_Puts("<< Start", &Font_7x10, SSD1306_COLOR_WHITE);
					else if (wType == mStop)
						SSD1306_Puts("<< Stop", &Font_7x10, SSD1306_COLOR_WHITE);
					else
						SSD1306_Puts(" Reverse", &Font_7x10, SSD1306_COLOR_WHITE);
			}

				if (IsRev)
					//SSD1306_Puts("<<<<<<<<", &Font_7x10, SSD1306_COLOR_WHITE);
					GPIO_SetBits(poRev);
				else
					//SSD1306_Puts(">>>>>>>>", &Font_7x10, SSD1306_COLOR_WHITE);
					GPIO_ResetBits(poRev);
			
		break;	
	}	

		SSD1306_UpdateScreen();	
}

void TempControl()
{
		int tmp = analogRead(ADC_Channel_8);
    
		float tr = 4096.0 / tmp - 1;
    tr = 18000 / tr;

    float steinhart;
    steinhart = tr / 62000; // (R/Ro)
    steinhart = log(steinhart); // ln(R/Ro)
    steinhart /= 3950; // 1/B * ln(R/Ro)
    steinhart += 1.0 / (18 + 273.15); // + (1/To)
    steinhart = 1.0 / steinhart; // Invert
    steinhart -= 273.15; 
    tmp = steinhart * 10;
	
		if (tmp < 5) return;
		
		CurTemp += (tmp - CurTemp)/30;
	if (CurTemp > 750)
	{
		Stop(false);
		SSD1306_GotoXY(17, 16); //Устанавливаем курсор в позицию Сначала по горизонтали, потом вертикали.
		SSD1306_Puts("Too hot!", &Font_16x26, SSD1306_COLOR_WHITE); 
		SSD1306_UpdateScreen();
		delay_ms(5000);
	}
	if (CurTemp > 500)
		GPIO_SetBits(poFanOn);
	if (CurTemp < 400)
		GPIO_ResetBits(poFanOn);
}

void SaveSettings()
{
	if (!HasChangesMenu) return;
	FLASH_Init();

	tpSettings settings;
	
	settings.StartDelim = StartSpeed;
	settings.StopDelim = StopSpeed;
	if (IsRev) settings.SpinMode = 1; else settings.SpinMode = 0;
	settings.MotorAmperage = MotorAmperage;
	
	FLASH_WriteSettings(&settings);
	HasChangesMenu = false;
}

void LoadSettings()
{
	tpSettings settings;
	FLASH_Init();
	
	FLASH_ReadSettings(&settings);

	if (settings.Hz > 65000)
	{
		SetHz = 50;
		StartSpeed = 21;
		StopSpeed = 21;
		IsRev = false;
		MotorAmperage = 0;
	}
	else
	{
		SetHz = (float)settings.Hz / 10.0f;
		StartSpeed = settings.StartDelim;
		StopSpeed = settings.StopDelim;
		if (settings.SpinMode) IsRev = true; else IsRev = false;
		MotorAmperage = settings.MotorAmperage;
	}
}

////////////////////////////////////////////
//Основной цикл

int main(void)
{
		SSD1306_Init();
		delay_ms(100);
		SSD1306_GotoXY(17, 16); //Устанавливаем курсор в позицию Сначала по горизонтали, потом вертикали.
		SSD1306_Puts("Hello!", &Font_16x26, SSD1306_COLOR_WHITE); 
		SSD1306_UpdateScreen();
	  
	  delay_ms(100);
		//LoadSettings();
		adc_init();
		adcVoltage_init();
		InitIO2Manual();	
		Stop(false);
		PowerInit();
	  delay_ms(100);
	
	while(1)
	{
		TempControl();
		
		//Нажали старт?
		if (ToStart())
		{		
			delay_ms(200);
			Start();
		}
		
		//Нажали стоп?
		if (ToStop())
		{
			if (ErrorState)  //Нажатие стоп после ошибки - гасит её
			{
				ErrorState = false;
				GPIO_ResetBits(poError);
				delay_ms(1500);
			}
			else
			{
				delay_ms(150);
				Stop(true);
			}
		}
		
		
		//Переключился на реверс?
		if (!GPIO_ReadInputDataBit(piRev))
		{
				delay_ms(10);
				if (!GPIO_ReadInputDataBit(piRev)) //Вдруг помеха? убеждаемся что через 10 мс "никто не передумал" =)
				{
					if (GPIO_ReadInputDataBit(poStart)) //Если на ходу
					{
							Stop(true);
							IsRev = !IsRev;
							Start();
					}
					else
					{
						IsRev = !IsRev;
						delay_ms(500);
					}
				}
		}
		
		//Проверка регулятора частоты
		if (GPIO_ReadInputDataBit(poStart))
			SetFrequency();
		
		
		//Контроль ошибки
		if (ModuleError)
		{
			OverloadCorrection();
		}
			
		UpdateLCD(mMain);
	}
}


