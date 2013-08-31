#ifndef PID_H_
#define PID_H_

typedef struct
{
		float Kp;
		float Ki;
		float Kd;
		float pPart;
		float iPart;
		float dPart;
		int32_t SetPoint;
		float PIDResult;
		float PIDResultTemp;
		int32_t PIDError;
		int32_t PIDErrorTemp1;
		int32_t PIDErrorTemp2;
		uint8_t Enable;
} PIDType;

void PIDSpeedSet(PIDType *p_PIDVer, int32_t SpeedSet);
void PIDPositionSet(PIDType *p_PID, int32_t SetPoint);
void PIDPosCalc(PIDType *p_PIDPos, int32_t Position, int32_t MaxResponse);
void PIDWallCalc(PIDType *p_PIDWall, int32_t Distance, int32_t MaxResponse);
void PIDVerCalc(PIDType *p_PIDVer, int32_t *Speed, int32_t MaxResponse);
void WallFollow(uint8_t AvailDirection);
void Move(int32_t PositionLeft, int32_t PositionRight);
uint32_t GetWallErrInt(void);
void ResetWallErrInt(void);

#endif /* PID_H_ */
