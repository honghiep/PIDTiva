#ifndef DEFINE_H_
#define DEFINE_H_

//#define NETWORK_ENABLE

//#define DEBUG_WALL_FOLLOW
//#define TEST_ENCODER
//#define TEST_IR
#define SET_PID
#define PID_SPEED
//#define PID_WALL
//#define PID_POSITION
//#define TEST_AVAIL_DIR

//#define SAVE_EEPROM

#define DEFAULT		20000	//H-Bridge Freq (Hz)

#define ENAPORT_PERIPHERAL	SYSCTL_PERIPH_GPIOB
#define ENABLE_PORT			GPIO_PORTB_BASE
#define ENA_LEFT_PIN		GPIO_PIN_5
#define ENA_RIGHT_PIN		GPIO_PIN_7
#define BOOST_EN_PIN		GPIO_PIN_2

#define MOTOR_LEFT		0x01
#define MOTOR_RIGHT		0x02

#endif /* DEFINE_H_ */
