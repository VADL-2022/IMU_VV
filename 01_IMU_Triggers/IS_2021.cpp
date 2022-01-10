/*
* VADL 2020-2021 Imaging System code, how they captured and sent an image.  For reference.
* 
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

*/