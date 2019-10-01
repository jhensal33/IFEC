#define FIRSTDIGIT 201
#define SECONDDIGIT 202
#define THIRDDIGIT 203
#define FourHundredmA 0.4
#define maxHRPWM 119

//These are defined by the linker
extern uint16_t RamfuncsLoadStart;
extern uint16_t RamfuncsLoadSize;
extern uint16_t RamfuncsRunStart;

const int phaseShiftMin = 10; //minimum phase
const int phaseShiftMax = 145; //maximum phase
const int period = 170;       //period of the PWM

/////////////MOST IMPORTANT VARIABLES (PID)
double 	kp = 0.0001,
        ki = 0.000075,
		currentSetPointCharging = 2,
		currentSetPointDischarging = 2;
		
int initialEPWMcharging = 156, 
	initialEPWMdischarging = 3,
	startingHRcharging = 0, 
	startingHRdischarging = 95,
	HRphase = 0,
	ePWMphase = 0;
/////////////MOST IMPORTANT VARIABLES (PID)

double  VbatMin,            //40
        VbatMax,            //50
        IchargeMax,         //25
        IdischargeMax,      //3
        Icharge,
        Idischarge,
        InitialLV = 0,
        actualIHV = 0,      //variables for display
		actualILV = 0,
		displayCurr = 0,
		actualIHV_33 = 0, 
		actualILV_33 = 0,
		adcVoltage = 0,
		actualVLV = 0,
		actualVLV_33 = 0,
        VLVplustemp = 0,    //variables for battery voltage calc
        VLVminustemp = 0,
        VLVsumPlus = 0,
        VLVsumMinus = 0,
        error = 0,          //closed loop control variables
        integral = 0,
        phase,
        measuredIHV[32],
        measuredILV[32],
        PIfactor,
        phaseStore;

int iLVcast = 0,            //variables for display
    iHVcast = 0,
    printCount = 0,
    VLVpluscast = 0,
	VLVminuscast = 0,
    digitOne = 0,
    setCurrent,
    buttonCount = 0,
	measureIndex = 0,       //variables for closed loop control
	sumOfIHV = 0,
	sumOfILV = 0, 
	sumOfVLVplus = 0, 
	sumOfVLVminus = 0, 
	sumOfVHV = 0,
	phaseCode,
	first32Flag = 0,
	t,                      //variables for printing measurements
	stack[4], i = 0;
	
//for converting HRPWM to course phase
int wholeNum = 0;
double phaseLeftover = 0;

//used for measurement display
long int sum, cast, newFlag = 0, displayChar = 0;
char * msgGlobal = "";

uint32_t count = 0, testCount = 0;  //used necessary for delays

enum UIstate {
    standby,
    modeSelect,
    changeLimits,
    chargeSetCurrent,
    chargeDisplay,
    dischargeCurrent,
    dischargeDisplay
};
enum UIstate state = standby;

enum params {
    vbatMin,
    vbatMax,
    ichargeMax,
    idischargeMax,
    icharge,
    idischarge
};
enum params parameter = vbatMin;

//peripheral handlers
ADC_Handle myAdc;
CLK_Handle myClk;
FLASH_Handle myFlash;
GPIO_Handle myGpio;
PIE_Handle myPie;
PWM_Handle myPwm1, myPwm2, myPwm3, myPwm4;
SCI_Handle mySci;
TIMER_Handle myTimer0;
CPU_Handle myCpu;
PLL_Handle myPll;
WDOG_Handle myWDog;
