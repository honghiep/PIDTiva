#ifndef INTERRUPTHANDLER_H_
#define INTERRUPTHANDLER_H_

void Timer5ISR(void);
void Timer4ISR(void);
void BluetoothIntHandler(void);//
void BattSenseISR(void);
void TimerTickBatt(void);
void BoostEnable(void);
void BoostDisable(void);
void HBridgeEnable(void);
void HBridgeDisable(void);

#endif /* INTERRUPTHANDLER_H_ */
