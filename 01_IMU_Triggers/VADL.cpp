#include <python3.7m/Python.h>
#include <iostream>
// #include <pigpio.h>
#include "config.hpp"
// #include "VADL.hpp"
#include "LOG.hpp"
#include "IMU.hpp"

using namespace std;

void *VADL::GDSReleaseTimeoutThread(void *userData)
{
	VADL *data = (VADL *)userData;
	cout << "GDS: Start " << GDS_RELEASE_TIMEOUT << " Seconds Timeout Cooldown" << endl;
	gpioSleep(PI_TIME_RELATIVE, GDS_RELEASE_TIMEOUT, 0);
	data->GDSTimeout = 1;
}

VADL::VADL()
{
	cout << "Main: Initiating" << endl;

	connect_GPIO();
	connect_Python();
	mImu = new IMU();
	mLog = new LOG(mImu);

	mImu->receive();
	mLog->receive();

	cout << "Main: Initiated" << endl;
}

VADL::~VADL()
{
	cout << "Main: Destorying" << endl;

	delete (mLog);
	delete (mImu);

	cout << "Main: Destoryed" << endl;
}

void VADL::connect_GPIO()
{
	cout << "GPIO: Connecting" << endl;

	if (gpioInitialise() < 0)
	{
		cout << "GPIO: Failed to Connect" << endl;
		exit(1);
	}

	cout << "GPIO: Connected" << endl;
}

void VADL::disconnect_GPIO()
{
	cout << "GPIO: Disconnecting" << endl;

	gpioTerminate();

	cout << "GPIO: Disconnected" << endl;
}

void VADL::connect_Python()
{
	cout << "Python: Connecting" << endl;

	Py_Initialize();

	cout << "Python: Connected" << endl;
}

void VADL::disconnect_Python()
{
	cout << "Python: Disconnecting" << endl;

	Py_Finalize();

	cout << "Python: Disconnected" << endl;
}

// Ground Detection System
void VADL::GDS()
{
	cout << "GDS: Activated" << endl;

	GDSTimeout = 0;
	bool imu_flag = !IMU_ACTIVE;	 // IMU Landing Flag

	while (1)
	{
		if (!imu_flag && mImu->linearAccelBody.mag() > GDS_LANDING_IMPACT_THRESHOLD) // Criterion: IMU Body Impact > Threshold
		{
			imu_flag = 1;

			cout << "GDS: IMU Criterion Met" << endl;
			mLog->write("GDS: IMU Criterion Met");
		}
		if (imu_flag) // All Criterion Met
		{
			cout << "GDS: All Criterion Met" << endl;
			mLog->write("GDS: All Criterion Met");

			PDS();

			break;
		}
		if (GDSTimeout) // Timeout
		{
			cout << "GDS: Release Timeout Criterion Met" << endl;
			mLog->write("GDS: Release Timeout Criterion Met");

			PDS();

			break;
		}

		gpioSleep(PI_TIME_RELATIVE, 0, 1000000 / FREQUENCY);
	}
}

void VADL::PDS()
{
	if (PDS_ACTIVE)
	{
		cout << "PDS: Activated" << endl;
		mLog->write("PDS: Activated");

		gpioSleep(PI_TIME_RELATIVE, PDS_DELAY_TIME, 0);
		gpioWrite(PDS_PIN, 1);
		gpioSleep(PI_TIME_RELATIVE, 0, PDS_ACTIVATION_TIME * 1000000);
		gpioWrite(PDS_PIN, 0);

		cout << "PDS: Deactivated" << endl;
		mLog->write("PDS: Deactivated");
	}
}

void VADL::COMMS()
{
	if (COMMS_ACTIVE) {
	cout << "COMMS: Activated" << endl;
	cout << "COMMS: Uploading Images" << endl;

	system("sudo rclone sync /home/pi/VADL2021-Source/img drive:VADL\\ \\'20-21/Landing_Image");

	cout << "COMMS: Uploaded Images" << endl;
	cout << "COMMS: Deactivated" << endl;
}
}