#define	USE_WATCHDOG

#define	XONXOFF

#define	MS		* (F_CPU / 1000UL)
#define	US		* (F_CPU / 1000000UL)

#define VIN_R1				33.0
#define VIN_R2				1.0

#define VSERVO_R1			10.0
#define VSERVO_R2			1.0

#define ISENSE_R			0.05
#define ISENSE_MR			50.0

#define NO_FAULT_TIME		122

#define ANTIPHASE_PWM_LIMIT	10

#define BRAKE_TIME_US		400000

#define MAX_CURRENT_MA		15000
#define MIN_VOLTAGE_MV		9000
