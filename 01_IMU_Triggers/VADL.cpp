#include <python3.7m/Python.h>
#include <iostream>
#include <pigpio.h>
#include "../include/config.hpp"
#include "../include/VADL.hpp"
#include "../include/LOG.hpp"
#include "../include/IMU.hpp"
#include "../include/LIDAR.hpp"
#include "../include/LDS.hpp"
#include "../include/MOTOR.hpp"

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
	mLidar = new LIDAR();
	mLds = new LDS();
	mMotor = new MOTOR();
	mLog = new LOG(mImu, mLidar, mLds);

	mImu->receive();
	mLog->receive();

	cout << "Main: Initiated" << endl;
}

VADL::~VADL()
{
	cout << "Main: Destorying" << endl;

	delete (mLog);
	delete (mMotor);
	delete (mLds);
	delete (mLidar);
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

void VADL::GDS()
{
	cout << "GDS: Activated" << endl;

	GDSTimeout = 0;
	bool imu_flag = !IMU_ACTIVE;	 // IMU Landing Flag
	bool lidar_flag = !LIDAR_ACTIVE; // LIDAR Landing Flag

	mLds->receive(); // Get LDS Data

	if (LDS_ACTIVE)
	{
		int count = 0;
		while (count < FREQUENCY / 2) // Criterion: LDS active for half a second
		{
			count = mLds->light[0] + mLds->light[1] + mLds->light[2] + mLds->light[3] >= GDS_LDS_THREASHHOLD ? count + 1 : 0;
			gpioSleep(PI_TIME_RELATIVE, 0, 1000000 / FREQUENCY);
		}

		cout << "GDS: LDS Criterion Met" << endl;
		mLog->write("GDS: LDS Criterion Met");
	}
	else // Activate Mock LDS
	{
		for (int i = 10; i > 0; --i) // Launch Countdown
		{
			gpioSleep(PI_TIME_RELATIVE, 0, 500000);
			gpioSleep(PI_TIME_RELATIVE, 0, 500000);
			cout << i << endl;
		}
		gpioSleep(PI_TIME_RELATIVE, 0, 500000);
		gpioSleep(PI_TIME_RELATIVE, 0, 500000);
		cout << "Launch" << endl;

		gpioSleep(PI_TIME_RELATIVE, GDS_MOCK_LDS_TIMER, 0); // LDS Mock Timer

		cout << "GDS: Mock LDS Criterion Met" << endl;
		mLog->write("GDS: Mock LDS Criterion Met");
	}

	mLidar->receive(); // Get Lidar Data
	cout << "GDS: Start " << GDS_JETTISON_COOLDOWN << " Seconds Jettison Cooldown" << endl;
	mLog->write("GDS: Start " + to_string(GDS_JETTISON_COOLDOWN) + " Seconds Jettison Cooldown");
	gpioSleep(PI_TIME_RELATIVE, GDS_JETTISON_COOLDOWN, 0); // Start Jettison Cooldown;
	cout << "GDS: Jettison Cooldown Criterion Met" << endl;
	mLog->write("GDS: Jettison Cooldown Criterion Met");

	gpioStartThread(GDSReleaseTimeoutThread, this); // Start Release Timeout

	while (1)
	{
		if (!imu_flag && mImu->linearAccelBody.mag() > GDS_LANDING_IMPACT_THRESHOLD) // Criterion: IMU Body Impact > Threshold
		{
			imu_flag = 1;

			cout << "GDS: IMU Criterion Met" << endl;
			mLog->write("GDS: IMU Criterion Met");
		}
		if (!lidar_flag && mLidar->distance < GDS_LANDING_ALTITUDE_THRESHOLD) // Criterion: LIDAR Distance < Threshold
		{
			lidar_flag = 1;

			cout << "GDS: LIDAR Criterion Met" << endl;
			mLog->write("GDS: LIDAR Criterion Met");
		}
		if (imu_flag && lidar_flag) // All Criterion Met
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

	mLds->halt();
	mLidar->halt();
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

void VADL::IS()
{
	if (IS_ACTIVE)
	{
		cout << "IS: Activated" << endl;
		mLog->write("IS: Activated");

		time_t now = time(nullptr);
		char command1[45];
		char command2[45];

		sprintf(command1, "raspistill -o %s_%04d%02d%02d_%02d%02d%02d_1.jpg",
				IMG_FILE.c_str(),
				localtime(&now)->tm_year + 1900,
				localtime(&now)->tm_mon + 1,
				localtime(&now)->tm_mday,
				localtime(&now)->tm_hour,
				localtime(&now)->tm_min,
				localtime(&now)->tm_sec);
		gpioWrite(IS_PIN, 0);
		gpioSleep(PI_TIME_RELATIVE, 1, 0);
		system(command1);

		cout << "IS: Command " << command1 << " Completed" << endl;
		mLog->write(command1);

		sprintf(command2, "raspistill -o %s_%04d%02d%02d_%02d%02d%02d_2.jpg",
				IMG_FILE.c_str(),
				localtime(&now)->tm_year + 1900,
				localtime(&now)->tm_mon + 1,
				localtime(&now)->tm_mday,
				localtime(&now)->tm_hour,
				localtime(&now)->tm_min,
				localtime(&now)->tm_sec);
		gpioWrite(IS_PIN, 1);
		gpioSleep(PI_TIME_RELATIVE, 1, 0);
		system(command2);

		cout << "IS: Command " << command2 << " Completed" << endl;
		mLog->write(command2);

		cout << "IS: Deactivated" << endl;
		mLog->write("IS: Deactivated");
	}
}

void VADL::LS()
{
	cout << "LS: Activated" << endl;
	mLog->write("LS: Activated");

	mMotor->zero();
	gpioSleep(PI_TIME_RELATIVE, 0, 1000000 / FREQUENCY);

	uint8_t pitch_axis;
	int8_t pitch_sign;
	uint8_t roll_axis;
	int8_t roll_sign;

	// TODO: implemnt leg mirror selection
	if (mImu->yprNed[1] > 0)
	{
		pitch_axis = MOTOR_BLACK;
		mMotor->position_control(MOTOR_BLACK);
		mMotor->mirror_control(MOTOR_YELLOW);
		pitch_sign = -1;
	}
	else
	{
		pitch_axis = MOTOR_BLACK;
		mMotor->position_control(MOTOR_BLACK);
		mMotor->mirror_control(MOTOR_YELLOW);
		pitch_sign = -1;
	}
	if (mImu->yprNed[2] > 0)
	{
		roll_axis = MOTOR_BLUE;
		mMotor->position_control(MOTOR_BLUE);
		mMotor->mirror_control(MOTOR_RED);
		roll_sign = 1;
	}
	else
	{
		roll_axis = MOTOR_BLUE;
		mMotor->position_control(MOTOR_BLUE);
		mMotor->mirror_control(MOTOR_RED);
		roll_sign = 1;
	}
	gpioSleep(PI_TIME_RELATIVE, 0, 1000000 / FREQUENCY);

	mMotor->arm();
	gpioSleep(PI_TIME_RELATIVE, 0, 1000000 / FREQUENCY);

	while (MOTOR_ACTIVE)
	{
		float pitch = mImu->yprNed[1];
		float roll = mImu->yprNed[2];
		if (hypot(pitch, roll) > 50)
		{
			cout << "LS: Flipped" << endl;
			mLog->write("LS: Flipped");
			break;
		}

		cout << "LS: Pitch: " << pitch << "\t\tRoll: " << roll << endl;
		mLog->write("LS: Pitch " + to_string(pitch) + "\t\tRoll: " + to_string(roll));

		if (hypot(pitch, roll) < LS_ANGLE_THRESHOLD)
		{
			gpioSleep(PI_TIME_RELATIVE, LS_STABILITY_THRESHOLD, 0);
			if (hypot(pitch, roll) < LS_ANGLE_THRESHOLD)
			{
				cout << "LS: Leveled at Pitch: " << pitch << "\t\tRoll: " << roll << endl;
				mLog->write("LS: Leveled at Pitch " + to_string(pitch) + "\t\tRoll: " + to_string(roll));
				break;
			}
		}

		float pitch_I_adjustment = pitch_sign * LS_I_GAIN * pitch;
		float roll_I_adjustment = roll_sign * LS_I_GAIN * roll;

		mMotor->position(pitch_axis, mMotor->positions[MOTOR_BLACK] + pitch_I_adjustment);
		mMotor->position(roll_axis, mMotor->positions[MOTOR_BLUE] + roll_I_adjustment);
		gpioSleep(PI_TIME_RELATIVE, 2, 0);
	}

	cout << "LS: Deactivated" << endl;
	mLog->write("LS: Deactivated");
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