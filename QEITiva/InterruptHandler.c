#include "include.h"
#include "driverlib/pwm.h"
#include "driverlib/qei.h"
#include "driverlib/rom.h"

extern PIDType PIDVerLeft, PIDVerRight, PIDPosLeft, PIDPosRight;
extern UARTType UART_Bluetooth;
extern uint32_t PIDVerLoop;

int32_t PosLeftCount = 0, PosRightCount = 0;
int8_t ControlFlag = 1;
int32_t set = 0;

uint8_t *p_UARTBuf, UARTBuf[50];

int16_t avrSpeed = 70;	//115


void BoostEnable(void)
{
	GPIOPinWrite(ENABLE_PORT, BOOST_EN_PIN, BOOST_EN_PIN);
}

void BoostDisable(void)
{
	GPIOPinWrite(ENABLE_PORT, BOOST_EN_PIN, 0);
}

void HBridgeEnable(void)
{
	GPIOPinWrite(ENABLE_PORT, ENA_LEFT_PIN | ENA_RIGHT_PIN, 0xFF);
}

void HBridgeDisable(void)
{
	GPIOPinWrite(ENABLE_PORT, ENA_LEFT_PIN | ENA_RIGHT_PIN, 0);
}

void BluetoothIntHandler(void)
{
	uint32_t ulStatus;
	static uint8_t count = 0;

	ulStatus = UARTIntStatus(UART_Bluetooth.PortName, true);
	ROM_UARTIntClear(UART_Bluetooth.PortName, ulStatus);
	while(UARTCharsAvail(UART_Bluetooth.PortName))
	{
#ifdef SET_PID
		ControlFlag = 1;
		*p_UARTBuf++ = (uint8_t)(UARTCharGetNonBlocking(UART_Bluetooth.PortName));
		if (p_UARTBuf[-1] == '\n')
		{
			p_UARTBuf = &UARTBuf[0];
			set = 0;
			count = 0;
		}
		else if (p_UARTBuf[-1] == '\r')
		{
			switch (count)
			{
#ifdef PID_SPEED
				case 0:
					avrSpeed = set / 10000;
					PIDVerLeft.SetPoint = avrSpeed;
//					PIDPosLeft.SetPoint = set / 10000;
					break;
				case 1:
					PIDVerLeft.Kp = (float)set / 10000;
					break;
				case 2:
					PIDVerLeft.Ki = (float)set / 10000;
					break;
				case 3:
					PIDVerLeft.Kd = (float)set / 10000;
					break;
#endif
#ifdef PID_POSITION
				case 0:
					PIDPosLeft.SetPoint = set / 10000;
					break;
				case 1:
					PIDPosLeft.Kp = (float)set / 10000;
					PIDPosRight.Kp = (float)set / 10000;
					break;
				case 2:
					PIDPosLeft.Ki = (float)set / 10000;
					PIDPosRight.Ki = (float)set / 10000;
					break;
				case 3:
					PIDPosLeft.Kd = (float)set / 10000;
					PIDPosRight.Kd = (float)set / 10000;
					break;
#endif
#ifdef PID_WALL
//				case 1:
//					DeltaEnc = (float)set / 10000;
//					break;

				case 1:
					PIDWallRight.Kp = (float)set / 10000;
					break;
				case 2:
					PIDWallRight.Ki = (float)set / 10000;
					break;
				case 3:
					PIDWallRight.Kd = (float)set / 10000;
					break;
#endif
			}

			set = 0;
			count++;
		}
		else
		{
			set = set * 10 + (p_UARTBuf[-1] - 48);
		}
#else
		return;
#endif
	}
}

void Timer5ISR(void)
{
	static uint8_t NumSpdSet = 0;
	static int32_t SpeedLeft, SpeedRight;
	ROM_TimerIntClear(TIMER5_BASE, TIMER_TIMA_TIMEOUT);

	SpeedLeft = ROM_QEIVelocityGet(QEI0_BASE) * ROM_QEIDirectionGet(QEI0_BASE);
	SpeedRight = ROM_QEIVelocityGet(QEI1_BASE) * ROM_QEIDirectionGet(QEI1_BASE);
	PosLeftCount = ((int32_t)ROM_QEIPositionGet(QEI0_BASE)) / 2;
	PosRightCount = ((int32_t)ROM_QEIPositionGet(QEI1_BASE)) / 2;
#ifdef PID_SPEED
	UARTPutn(UART_Bluetooth.PortName, (int32_t)SpeedLeft);
	UARTCharPut(UART_Bluetooth.PortName, '\n');
#endif
#ifdef PID_POSITION
	UARTPutn(UART_Bluetooth.PortName, (int32_t)PosLeftCount);
	UARTCharPut(UART_Bluetooth.PortName, '\n');
#endif
	if (PIDVerLeft.Enable)
	{
		PIDVerCalc(&PIDVerLeft, &SpeedLeft, 90);
		SetPWM(MOTOR_LEFT, DEFAULT, (long)PIDVerLeft.PIDResult);
	}
	if (PIDVerRight.Enable)
	{
		PIDVerCalc(&PIDVerRight, &SpeedRight, 90);
		SetPWM(MOTOR_RIGHT, DEFAULT, (long)PIDVerRight.PIDResult);
	}

	NumSpdSet++;

	if (NumSpdSet == PIDVerLoop)	//PID position
	{
		NumSpdSet = 0;
		if (PIDPosLeft.Enable)
		{

			PIDPosCalc(&PIDPosLeft, PosLeftCount, avrSpeed);
			PIDSpeedSet(&PIDVerLeft, (long)PIDPosLeft.PIDResult);
		}
		if (PIDPosRight.Enable)
		{
			PIDPosCalc(&PIDPosRight, PosRightCount, avrSpeed);
			PIDSpeedSet(&PIDVerRight, (long)PIDPosRight.PIDResult);
		}
	}
}

void BattSenseISR(void)
{
	uint8_t temp;
	uint32_t BattResult;
	static uint32_t avrBattResult = 2700;
	ROM_ADCIntClear(ADC1_BASE, 3);
	ROM_ADCSequenceDataGet(ADC1_BASE, 3, &BattResult);

	avrBattResult = (4 * avrBattResult + BattResult) / 5;
	if (avrBattResult < 2500)
	{
		ROM_GPIOPinWrite(ENABLE_PORT, ENA_LEFT_PIN | ENA_RIGHT_PIN, 0x00);
		SetPWM(MOTOR_LEFT, DEFAULT, 0);
		SetPWM(MOTOR_RIGHT, DEFAULT, 0);
		BoostDisable();
		IntMasterDisable();
	}
}

void TimerTickBatt(void)
{
	ROM_TimerIntClear(TIMER3_BASE, TIMER_TIMA_TIMEOUT);
	ROM_ADCProcessorTrigger(ADC1_BASE, 3);
}
