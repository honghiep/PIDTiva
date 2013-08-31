#include "include.h"

extern int32_t PosLeftCount, PosRightCount;
PIDType PIDVerLeft, PIDVerRight, PIDPosLeft, PIDPosRight;

void PIDSpeedSet(PIDType *p_PIDVer, int32_t SpeedSet)
{
	(*p_PIDVer).Enable = 1;
	(*p_PIDVer).SetPoint = SpeedSet;
	(*p_PIDVer).iPart = 0;
}

void PIDPositionSet(PIDType *p_PID, int32_t SetPoint)
{
	(*p_PID).Enable = 1;
	(*p_PID).iPart = 0;
	(*p_PID).PIDErrorTemp1 = 0;
	(*p_PID).SetPoint = SetPoint;
	if (p_PID == &PIDPosLeft)
	{
		ROM_QEIPositionSet(QEI0_BASE, 0);
		PosLeftCount = 0;
	}
	else
	{
		ROM_QEIPositionSet(QEI1_BASE, 0);
		PosRightCount = 0;
	}
}

void PIDPosCalc(PIDType *p_PIDPos, int32_t Position, int32_t MaxResponse)
{
	(*p_PIDPos).PIDError = (*p_PIDPos).SetPoint - Position;
	(*p_PIDPos).pPart = (*p_PIDPos).Kp * (*p_PIDPos).PIDError;
	(*p_PIDPos).iPart += (*p_PIDPos).Ki * (*p_PIDPos).PIDError;
	(*p_PIDPos).dPart = (*p_PIDPos).Kd * ((*p_PIDPos).PIDError - (*p_PIDPos).PIDErrorTemp1);
//	/*
	//Uncomment to enable iPart-limit
	if ((*p_PIDPos).iPart > 5)
		(*p_PIDPos).iPart = 5;
	else if ((*p_PIDPos).iPart < -5)
		(*p_PIDPos).iPart = -5;
//	*/
	(*p_PIDPos).PIDResult = (*p_PIDPos).pPart + (*p_PIDPos).iPart + (*p_PIDPos).dPart;
	if ((*p_PIDPos).PIDResult > MaxResponse)
		(*p_PIDPos).PIDResult = (float)(MaxResponse);
	if ((*p_PIDPos).PIDResult < -1 * MaxResponse)
		(*p_PIDPos).PIDResult = (float)(-1 * MaxResponse);
}

//MaxResponse: Max Duty Cycle
void PIDVerCalc(PIDType *p_PIDVer, int32_t *Speed, int32_t MaxResponse)
{
	(*p_PIDVer).PIDError = (*p_PIDVer).SetPoint - (*Speed);
	*Speed = 0;

	(*p_PIDVer).pPart = (*p_PIDVer).Kp * (*p_PIDVer).PIDError;
	(*p_PIDVer).iPart += (*p_PIDVer).Ki * (*p_PIDVer).PIDError * 0.01;
	(*p_PIDVer).dPart = (*p_PIDVer).Kd * ((*p_PIDVer).PIDError - (*p_PIDVer).PIDErrorTemp1)/0.01;

	if ((*p_PIDVer).iPart > 40)
		(*p_PIDVer).iPart = 40;
	else if ((*p_PIDVer).iPart < -40)
		(*p_PIDVer).iPart = -40;

	(*p_PIDVer).PIDResult += ((*p_PIDVer).pPart + (*p_PIDVer).iPart + (*p_PIDVer).dPart) ;

	if ((*p_PIDVer).PIDResult > MaxResponse)
		(*p_PIDVer).PIDResult = (float)MaxResponse;
	if ((*p_PIDVer).PIDResult < -1 * MaxResponse)
		(*p_PIDVer).PIDResult = (float)(-1 * (MaxResponse));
	(*p_PIDVer).PIDErrorTemp1 = (*p_PIDVer).PIDError;
}

void Move(int32_t PositionLeft, int32_t PositionRight)
{
	PIDPositionSet(&PIDPosLeft, PositionLeft);
	PIDPositionSet(&PIDPosRight, PositionRight);
}
