#include <iostream>
#include <stdio.h>
#include <string.h>

#include <chrono>
#include <ctime>   

#include "RoboteqDevice.h"
#include "ErrorCodes.h"
#include "Constants.h"

using namespace std;

int MotorControllerSample(string portLeft, string portRight) 
{
	cout << endl << "Motor Controller Sample:" << endl;
	cout << "------------------------" << endl;
	string response = "";
	RoboteqDevice deviceLeft;
	RoboteqDevice deviceRight;
	int statusL = deviceLeft.Connect(portLeft);
	int statusR = deviceRight.Connect(portRight);

	if (statusL != RQ_SUCCESS || statusR != RQ_SUCCESS)
	{
		cout << "Error connecting to device: " << statusL << ", " << statusR << "." << endl;
		return 1;
	}

	// int canNodeID;
	// cout << "- Read CAN Node ID: GetConfig(_CNOD, 1)...";
	// if ((status = device.GetConfig(_CNOD, 1, canNodeID)) != RQ_SUCCESS)
	// 	cout << "failed --> " << status << endl;
	// else
	// 	cout << "returned --> " << canNodeID << endl;

	// //Wait 10 ms before sending another command to device
	// sleepms(10);

	// cout << "- Set CAN Node ID: SetConfig(_CNOD, 1, "<< canNodeID<<")...";
	// if ((status = device.SetConfig(_CNOD, 1, canNodeID)) != RQ_SUCCESS)
	// 	cout << "failed --> " << status << endl;
	// else
	// 	cout << "succeeded." << endl;

	// //Wait 10 ms before sending another command to device
	// sleepms(10);

	// cout << "- Set User Variable 10: SetCommand(_VAR, 10, 100)...";
	// if ((status = device.SetCommand(_VAR, 10, 100)) != RQ_SUCCESS)
	// 	cout << "failed --> " << status << endl;
	// else
	// 	cout << "succeeded." << endl;

	// //Wait 10 ms before sending another command to device
	// sleepms(10);

	// int result;
	// cout << "- Read User Variable 10: GetValue(_VAR, 10)...";
	// if ((status = device.GetValue(_VAR, 10, result)) != RQ_SUCCESS)
	// 	cout << "failed --> " << status << endl;
	// else
	// 	cout << "returned --> " << result << endl;

	// if ((status = device.SetCommand(_MOTVEL, 1, 150)) != RQ_SUCCESS)
	// 	cout << "failed --> " << status << endl;
	// else
	// 	cout << "returned --> " << result << endl;

	// if ((status = device.SetCommand(_MOTVEL, 2, 150)) != RQ_SUCCESS)
	// 	cout << "failed --> " << status << endl;
	// else
	// 	cout << "returned --> " << result << endl;

	auto start = std::chrono::system_clock::now();
	int i = 0;
	int pos1 = 0;
	int pos2 = 0;
	int pos3 = 0;
	int pos4 = 0;
	int status;
	int speed = 125;

	while (i < 100000) {
		// Set all motor speed
		status =  deviceLeft.SetCommand(_MOTVEL, 1, speed);
		sleepms(10);
		status =  deviceLeft.SetCommand(_MOTVEL, 2, speed);
		sleepms(10);
		status = deviceRight.SetCommand(_MOTVEL, 1, -speed);
		sleepms(10);
		status = deviceRight.SetCommand(_MOTVEL, 2, -speed);
		sleepms(10);

		int status1 = deviceLeft.GetValue(_ABSPEED, 1, pos1);
		sleepms(10);
		int status2 = deviceLeft.GetValue(_ABSPEED, 2, pos2);
		sleepms(10);
		int status3 = deviceRight.GetValue(_ABSPEED, 1, pos3);
		sleepms(10);
		int status4 = deviceRight.GetValue(_ABSPEED, 2, pos4);
		sleepms(10);

		auto end = std::chrono::system_clock::now();
		std::chrono::duration<double> elapsed_seconds = end-start;

		// std::cout << status1 << ", " << pos1 << ", " 
		// 		  << status2 << ", " << pos2 << ", "
		// 		  << status3 << ", " << pos3 << ", "
		// 		  << status4 << ", " << pos4 << ", " 
		// 		  << " -- Elapsed time: " << elapsed_seconds.count() << "s"
		// 		  << std::endl;
		if (status1 != RQ_SUCCESS || status2 != RQ_SUCCESS || 
			status3 != RQ_SUCCESS || status4 != RQ_SUCCESS) {
			std::cout << status1 << ", " << pos1 << ", " 
					<< status2 << ", " << pos2 << ", "
					<< status3 << ", " << pos3 << ", "
					<< status4 << ", " << pos4 << ", " 
					<< " -- Elapsed time: " << elapsed_seconds.count() << "s"
					<< std::endl;
		}
		i+=1;
		// sleepms(76000);
	}

	deviceLeft.Disconnect();
	deviceRight.Disconnect();
	return 0;
}

int main(int argc, char *argv[])
{
	
#ifdef linux
	string portLeft = "/dev/ttyACM1";
	string portRight = "/dev/ttyACM0";
#endif

	MotorControllerSample(portLeft, portRight);
	return 0;
}
