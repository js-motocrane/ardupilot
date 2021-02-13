#include "Copter.h"

extern const AP_HAL::HAL& hal;

// adding this to access voltage from battery monitoring
const AP_BattMonitor &battery = AP::battery();

AP_HAL::AnalogSource* chan;    //delare a pointer to AnalogSource object. AnalogSource class can be found in : AP_HAL->AnalogIn.h

// reported mode from the generator:
enum GenMode {
    IDLE = 0,
    RUN = 1,
    CHARGE = 2,
    BALANCE = 3,
    OFF = 4,
};

// un-packed data from the generator:
struct Reading {
    uint32_t    runtime; //seconds
    uint32_t    seconds_until_maintenance;
    uint16_t    errors;
    uint16_t    rpm;
    float       output_voltage;
    float       output_current;
    GenMode     mode;
    float 		pwrIntegral;
    float		pwrGenerated;
};

// declare some variables to use
struct Reading last_reading;

float lastCurrent;
float fuelPctLocal;
uint32_t last_reading_ms;
uint8_t timeCaptured;
uint32_t lastMs;


#ifdef USERHOOK_INIT
void Copter::userhook_init()
{
    // put your initialisation code here
    // this will be called once at start-up
    // initialize analog channel for reading external current sensor on ADC port
    chan = hal.analogin->channel(15);    //initialization of chan variable. AnalogIn class can be found in : AP_HAL->AnalogIn.h

    chan->set_pin(15);
}
#endif

#ifdef USERHOOK_FASTLOOP
void Copter::userhook_FastLoop()
{
    // put your 100Hz code here
}
#endif

#ifdef USERHOOK_50HZLOOP
void Copter::userhook_50Hz()
{
    // put your 50Hz code here
	// update the readings here

	// temporary - just counting seconds from boot up of flight controller with this
    last_reading.runtime = (uint32_t)(AP_HAL::millis()*0.001);
    last_reading.seconds_until_maintenance = (uint32_t)(AP_HAL::millis()*0.001);

    // temporary - just setting errors to 0
    uint16_t errors;
    errors = 0;
    last_reading.errors = errors;

    // temporary - just setting rpm to arbitrary number
    uint16_t rpmTemp;
    rpmTemp = 12000;
    last_reading.rpm = rpmTemp;

    // get voltage from battery monitor
    last_reading.output_voltage = battery.voltage();

    //last_reading.output_voltage = 51.0;

    //gcs().send_text(MAV_SEVERITY_CRITICAL, "GEN update %.2f",last_reading.output_voltage);

    // get analog reading and convert it to current
	float I_vm  = chan->voltage_average();
	I_vm = I_vm/2.0; // scale voltage according to the documentation (it is multiplied by 2)

	//gcs().send_text(MAV_SEVERITY_CRITICAL, "GEN update %.2f",I_vm);

	float cur_local;
	// scale into current
	//cur_local = (I_vm*0.000805664 - 0.5) * 16.667;
	cur_local = (I_vm - 0.5) * 16.667;

	if (cur_local < 0.0)
	{
		cur_local = 0.0;
	}

	// filter it
	last_reading.output_current = lastCurrent * 0.7 + cur_local * 0.3;

	if (last_reading.output_current < 0.5)
	{
		// cap it to zero
		last_reading.output_current = 0.0;
	}

	lastCurrent = last_reading.output_current;

	last_reading_ms = AP_HAL::millis();

	uint8_t tempMode = 0x01; // RUN
    last_reading.mode = (GenMode)tempMode;

    // calculate instantaneous power

    // TEMP!!
    //last_reading.output_voltage = 51.2;

    last_reading.pwrGenerated = last_reading.output_current * last_reading.output_voltage;

    // *************************************************************************************************
    // *********** ENERGY INTEGRAL CALCULATION *********************************************************
    // *************************************************************************************************

	float dt;

	if (!timeCaptured)
	{
		timeCaptured = 1;

		dt = 0.0;
		lastMs = AP_HAL::millis();
	}
	else
	{
		// we got the value once already
		dt = ((float)(AP_HAL::millis() - lastMs)) * 0.001; // convert to seconds
		// update previous time
		lastMs = AP_HAL::millis();
	}

	if (last_reading.pwrIntegral > GEN_ENERGY_MAX_KJ)
	{
		last_reading.pwrIntegral = GEN_ENERGY_MAX_KJ;
	}
	else
	{
		last_reading.pwrIntegral += last_reading.pwrGenerated * dt * 0.001; // converting to kJ
	}




    // *************************************************************************************************
    // *********** END OF ENERGY INTEGRAL CALCULATION **************************************************
    // *************************************************************************************************

	fuelPctLocal = 100.0 - last_reading.pwrIntegral / GEN_ENERGY_THRESH_KJ * 100.0;

	fuelPctLocal /= 100.0; // to keep within 0 and 1 bounds that is expected

	if (fuelPctLocal < 0.0)
	{
		fuelPctLocal = 0.0;
	}




	static uint8_t counter1 = 25;
	counter1++;
	if (counter1 > 100) {
	    counter1 = 0;
	    //gcs().send_text(MAV_SEVERITY_CRITICAL, "GEN: %.1f A, %.1f kW, %.1f %% ",last_reading.output_current,last_reading.pwrGenerated*0.001,fuelPctLocal*100);
	    //gcs().send_text(MAV_SEVERITY_NOTICE, "GEN: %.1f A, %.2f kW, %.1f %% ",last_reading.output_current,last_reading.pwrGenerated*0.001,fuelPctLocal*100);
	    gcs().send_text(MAV_SEVERITY_INFO, "GEN: %.1f A, %.2f kW, %.1f %% ",last_reading.output_current,last_reading.pwrGenerated*0.001,fuelPctLocal*100);
	}

//	static uint8_t counter2 = 0;
//	counter2++;
//	if (counter2 > 50) {
//	    counter2 = 0;
//	    //gcs().send_text(MAV_SEVERITY_CRITICAL, "GEN update %.2f",I_vm);
//	    gcs().send_text(MAV_SEVERITY_CRITICAL, "PWR: %.3f kW",last_reading.pwrGenerated*0.001);
//	}
//
//	static uint8_t counter3 = 75;
//	counter3++;
//	if (counter3 > 100) {
//	    counter3 = 0;
//	    gcs().send_text(MAV_SEVERITY_CRITICAL, "Fuel: %.3f %%",fuelPctLocal*100);
//	}

	// TODO - make this work
//    mavlink_msg_generator_status_send(
//        channel.get_chan(),
//        status,//
//        last_reading.rpm, // generator_speed
//        std::numeric_limits<double>::quiet_NaN(), // battery_current; current into/out of battery
//		last_reading.output_current, // load_current; Current going to UAV
//        //std::numeric_limits<double>::quiet_NaN(), // power_generated; the power being generated
//		last_reading.pwrGenerated,
//        last_reading.output_voltage, // bus_voltage; Voltage of the bus seen at the generator
//        INT16_MAX, // rectifier_temperature
//        std::numeric_limits<double>::quiet_NaN(), // bat_current_setpoint; The target battery current
//        INT16_MAX, // generator temperature
//        last_reading.runtime,
//        (int32_t)last_reading.seconds_until_maintenance
//        );


//	float cur_local;
//	// scale into current
//	//cur_local = (I_vm*0.000805664 - 0.5) * 16.667;
//	cur_local = (I_vm - 0.5) * 16.667;
//
//	if (cur_local < 0.0)
//	{
//		cur_local = 0.0;
//	}

}
#endif

#ifdef USERHOOK_MEDIUMLOOP
void Copter::userhook_MediumLoop()
{
    // put your 10Hz code here
}
#endif

#ifdef USERHOOK_SLOWLOOP
void Copter::userhook_SlowLoop()
{
    // put your 3.3Hz code here
}
#endif

#ifdef USERHOOK_SUPERSLOWLOOP
void Copter::userhook_SuperSlowLoop()
{
    // put your 1Hz code here
}
#endif

#ifdef USERHOOK_AUXSWITCH
void Copter::userhook_auxSwitch1(uint8_t ch_flag)
{
    // put your aux switch #1 handler here (CHx_OPT = 47)
}

void Copter::userhook_auxSwitch2(uint8_t ch_flag)
{
    // put your aux switch #2 handler here (CHx_OPT = 48)
}

void Copter::userhook_auxSwitch3(uint8_t ch_flag)
{
    // put your aux switch #3 handler here (CHx_OPT = 49)
}
#endif
