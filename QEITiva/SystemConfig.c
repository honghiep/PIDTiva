#include "include.h"
#include "inc/hw_gpio.h"
#include "driverlib/qei.h"
#include "driverlib/pwm.h"
#include "driverlib/fpu.h"
#include "driverlib/rom.h"

uint32_t PIDVerLoop = 0;
UARTType UART_Bluetooth;

extern uint8_t *p_UARTBuf, UARTBuf[];

extern void Timer5ISR(void);			//Timer 5 - used for PID control
extern void BluetoothIntHandler(void);
extern void BattSenseISR(void);			//Under-Voltage Interrupt Routine
extern void TimerTickBatt(void);

inline void ConfigSystem(void)
{
	ROM_FPULazyStackingEnable();
	ROM_FPUEnable();
	// Config clock
	ROM_SysCtlClockSet(SYSCTL_SYSDIV_2_5 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN | SYSCTL_XTAL_16MHZ);
	//Boost Converter Control
	ROM_SysCtlPeripheralEnable(ENAPORT_PERIPHERAL);
	ROM_GPIOPinTypeGPIOOutput(ENABLE_PORT, BOOST_EN_PIN);
	ROM_GPIOPinWrite(ENABLE_PORT, BOOST_EN_PIN, 0);
	//H-Bridges Control
	ROM_SysCtlPeripheralEnable(ENAPORT_PERIPHERAL);
	ROM_GPIOPinTypeGPIOOutput(ENABLE_PORT, ENA_LEFT_PIN | ENA_RIGHT_PIN);
	ROM_GPIOPinWrite(ENABLE_PORT, ENA_LEFT_PIN | ENA_RIGHT_PIN, 0x00);
}

inline void ConfigPIDTimer(uint32_t TimerIntervalms, uint32_t PIDVerlocityLoop)
{
	PIDVerLoop = PIDVerlocityLoop;
	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER5);
	ROM_TimerConfigure(TIMER5_BASE, TIMER_CFG_PERIODIC);
	ROM_TimerLoadSet(TIMER5_BASE, TIMER_A, (SysCtlClockGet() / 1000) * TimerIntervalms);	//Interval: //1:150
	TimerIntRegister(TIMER5_BASE, TIMER_A, &Timer5ISR);
	ROM_IntEnable(INT_TIMER5A);
	ROM_TimerIntEnable(TIMER5_BASE, TIMER_TIMA_TIMEOUT);
	ROM_TimerIntClear(TIMER5_BASE, TIMER_TIMA_TIMEOUT);
	ROM_TimerEnable(TIMER5_BASE, TIMER_A);
}

inline void ConfigPWM(void)
{
	ROM_SysCtlPWMClockSet(SYSCTL_PWMDIV_1);
	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM0);
	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
	ROM_GPIOPinTypePWM(GPIO_PORTB_BASE, GPIO_PIN_7 | GPIO_PIN_6);
	ROM_GPIOPinConfigure(GPIO_PB7_M0PWM1);
	ROM_GPIOPinConfigure(GPIO_PB6_M0PWM0);
	ROM_PWMGenConfigure(PWM0_BASE, PWM_GEN_0, PWM_GEN_MODE_DOWN |
                    PWM_GEN_MODE_NO_SYNC);
	ROM_PWMGenPeriodSet(PWM0_BASE, PWM_GEN_0, SysCtlClockGet() / DEFAULT);
	ROM_PWMPulseWidthSet(PWM0_BASE, PWM_OUT_0,
    		(SysCtlClockGet() / DEFAULT) / 2);
	ROM_PWMPulseWidthSet(PWM0_BASE, PWM_OUT_1,
    		(SysCtlClockGet() / DEFAULT) / 2);

	ROM_PWMOutputState(PWM0_BASE, PWM_OUT_0_BIT, true);
	ROM_PWMOutputState(PWM0_BASE, PWM_OUT_1_BIT, true);
	ROM_PWMOutputInvert(PWM0_BASE, PWM_OUT_0_BIT, false);
	ROM_PWMOutputInvert(PWM0_BASE, PWM_OUT_1_BIT, true);
	ROM_PWMGenEnable(PWM0_BASE, PWM_GEN_0);
}

void SetPWM(uint8_t MotorSel, uint32_t ulFrequency, int32_t ucDutyCycle)
{
	uint32_t ulPeriod;
	ulPeriod = SysCtlClockGet() / ulFrequency;
	if (ucDutyCycle > 90)
		ucDutyCycle = 90;
	else if (ucDutyCycle < -90)
		ucDutyCycle = -90;
	if (MotorSel == MOTOR_LEFT)
    {
		ROM_PWMGenPeriodSet(PWM0_BASE, PWM_GEN_1, ulPeriod - 1);
		ROM_PWMPulseWidthSet(PWM0_BASE, PWM_OUT_0, (100 + ucDutyCycle) * ulPeriod / 200 - 1);
		ROM_PWMPulseWidthSet(PWM0_BASE, PWM_OUT_1, (100 + ucDutyCycle) * ulPeriod / 200 - 1);
    }
	else if (MotorSel == MOTOR_RIGHT)
	{
//		ROM_PWMGenPeriodSet(PWM0_BASE, PWM_GEN_0, ulPeriod - 1);
//		ROM_PWMPulseWidthSet(PWM0_BASE, PWM_OUT_0, (100 + ucDutyCycle) * ulPeriod / 200 - 1);
	}
}

inline void ConfigEncoder(void)
{
	//QEI0: Left
	//QEI1: Right
	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
    HWREG(GPIO_PORTD_BASE + GPIO_O_LOCK) = GPIO_LOCK_KEY;
    HWREG(GPIO_PORTD_BASE + GPIO_O_CR) = 0x80;
    HWREG(GPIO_PORTD_BASE + GPIO_O_AFSEL) &= ~0x80;

    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_QEI0);
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
    ROM_QEIConfigure(QEI0_BASE, QEI_CONFIG_CAPTURE_A|QEI_CONFIG_NO_RESET|QEI_CONFIG_QUADRATURE|QEI_CONFIG_SWAP, 0xFFFFFFFF);
    ROM_GPIOPinTypeQEI(GPIO_PORTD_BASE, GPIO_PIN_6 | GPIO_PIN_7);
    ROM_GPIOPinConfigure(GPIO_PD6_PHA0);
    ROM_GPIOPinConfigure(GPIO_PD7_PHB0);
    ROM_QEIVelocityConfigure(QEI0_BASE, QEI_VELDIV_2, SysCtlClockGet() / 100);
    ROM_QEIVelocityEnable(QEI0_BASE);
    ROM_QEIEnable(QEI0_BASE);

    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_QEI1);
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);
    ROM_QEIConfigure(QEI1_BASE, QEI_CONFIG_CAPTURE_A|QEI_CONFIG_NO_RESET|QEI_CONFIG_QUADRATURE|QEI_CONFIG_SWAP, 0xFFFFFFFF);
    ROM_GPIOPinTypeQEI(GPIO_PORTC_BASE, GPIO_PIN_5 | GPIO_PIN_6);
    ROM_GPIOPinConfigure(GPIO_PC5_PHA1);
    ROM_GPIOPinConfigure(GPIO_PC6_PHB1);
    ROM_QEIVelocityConfigure(QEI1_BASE, QEI_VELDIV_2, SysCtlClockGet() / 100);
    ROM_QEIVelocityEnable(QEI1_BASE);
    ROM_QEIEnable(QEI1_BASE);
}

inline void ConfigNetwork(void)
{
	p_UARTBuf = &UARTBuf[0];

	UART_Bluetooth.PortName = UART0;
	UART_Bluetooth.BaudRate = 115200;
	UART_Bluetooth.DataBits = 8;
	UART_Bluetooth.Parity = None;
	UART_Bluetooth.StopBits = 1;
	UART_Bluetooth.ISR = &BluetoothIntHandler;
	ConfigUART(&UART_Bluetooth);
}

inline void ConfigBattSense(void)
{
	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC1);
	ROM_GPIOPinTypeADC(GPIO_PORTD_BASE, GPIO_PIN_3);
	ROM_ADCHardwareOversampleConfigure(ADC1_BASE, 64);

	ROM_ADCSequenceConfigure(ADC1_BASE, 3, ADC_TRIGGER_PROCESSOR, 0);
	ROM_ADCSequenceStepConfigure(ADC1_BASE, 3, 0, ADC_CTL_END | ADC_CTL_CH4 | ADC_CTL_IE);
	ROM_ADCSequenceEnable(ADC1_BASE, 3);
 	ADCIntRegister(ADC1_BASE, 3, &BattSenseISR);
 	ROM_ADCIntEnable(ADC1_BASE, 3);

 	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER3);

 	ROM_TimerConfigure(TIMER3_BASE, TIMER_CFG_PERIODIC);
 	ROM_TimerLoadSet(TIMER3_BASE, TIMER_A, SysCtlClockGet() / 10);	//Interval: 300s
 	TimerIntRegister(TIMER3_BASE, TIMER_A, &TimerTickBatt);
	IntEnable(INT_TIMER3A);
	ROM_TimerIntEnable(TIMER3_BASE, TIMER_TIMA_TIMEOUT);
	ROM_TimerIntClear(TIMER3_BASE, TIMER_TIMA_TIMEOUT);
	ROM_TimerEnable(TIMER3_BASE, TIMER_A);
}
