#include "include.h"
#include "driverlib/eeprom.h"
#include "driverlib/qei.h"
#include "driverlib/pwm.h"
#include "driverlib/rom.h"

int32_t Position[2];
extern PIDType PIDVerLeft, PIDVerRight, PIDPosLeft, PIDPosRight;
extern int8_t ControlFlag;


void InitPID(void)
{
	PIDVerLeft.Kp = 0.32;
	PIDVerLeft.Ki = 0.0015;
	PIDVerLeft.Kd = 0.0055;

	PIDPosLeft.Kp = 0.18;
	PIDPosLeft.Ki = 0.0045;
	PIDPosLeft.Kd = 0.0055;
}

void main (void)
{
	ConfigSystem();
	ConfigEncoder();
	ConfigPWM();
	ConfigNetwork();
	ConfigPIDTimer(7, 2);
//	ConfigBattSense();
	InitPID();
	SetPWM(MOTOR_LEFT, DEFAULT, 0);
	SetPWM(MOTOR_RIGHT, DEFAULT, 0);
	ROM_IntMasterEnable();

	BoostEnable();
	HBridgeEnable();
//	PIDSpeedSet(&PIDVerLeft, 50);

	while(1)
	{
		if (ControlFlag)
		{
			ControlFlag = 0;
			PIDPosLeft.Enable = 0;
			PIDVerLeft.Enable = 0;
			SetPWM(MOTOR_LEFT, DEFAULT, 0);
			PIDPosLeft.iPart = 0;
			PIDPosLeft.PIDErrorTemp1 = 0;
			PIDPosLeft.PIDResult = 0;
			PIDVerLeft.iPart = 0;
			PIDVerLeft.PIDErrorTemp1 = 0;
			PIDVerLeft.PIDResult = 0;

			SysCtlDelay(SysCtlClockGet()/10);
//			PIDPosLeft.Enable = 1;
#ifdef PID_SPEED
			PIDVerLeft.Enable = 1;
#endif
#ifdef PID_POSITION
			PIDPositionSet(&PIDPosLeft, PIDPosLeft.SetPoint);
#endif
		}
	}

}
