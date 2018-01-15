#include <mraa.hpp>
#include <iostream>
#include <pthread.h>
#include <unistd.h>
#include <boost/interprocess/shared_memory_object.hpp>
#include <boost/interprocess/mapped_region.hpp>
#include <cstring>
#include <cstdlib>
#include <string>
#include <sstream> // ostringstream
#include <csignal> // for SIGNT

#include "../include/FlightController.hpp"
#include "../include/FCLogger.hpp"

FlightController* flightController = NULL;

void _SIG_HANDLER_ (int sig) {
	std::cout << "Interrupt signal received " << sig << std::endl;

	if(flightController != NULL) {
		delete flightController;
	}

	exit(sig);
}

int main()
{
	// register SIGINT
	signal(SIGINT, _SIG_HANDLER_);

	/* System Check */
	mraa::Platform platform = mraa::getPlatformType();
	if(platform == mraa::INTEL_EDISON_FAB_C) {
		std::cout << "Found Edison Platform" << std::endl;
		mraa::setPriority(-20);
	} else {
		std::cout << "NOT RUNNING ON EDISON" << std::endl;
		return -1;
	}


	boost::interprocess::shared_memory_object shared_mem_pilot(
			boost::interprocess::open_or_create,
			"shared_mem_pilot",
			boost::interprocess::read_write
	);
	shared_mem_pilot.truncate(1002);
	boost::interprocess::mapped_region region(shared_mem_pilot, boost::interprocess::read_write);
	std::memset(region.get_address(), 1, region.get_size());
	std::memset(region.get_address(), '\0', region.get_size());
	std::strncpy((char*)region.get_address(), "0", 1000);

	//Start PID Controller
	std::cout << "Starting PID Controller..." << std::endl;


	flightController = new FlightController();
	flightController->run();

	/*start the controller from a thread*
	pthread_t flightControllerThread;
	pthread_create(&flightControllerThread, NULL, &PidController::fcLoopHelper, channels);
	std::cout << "Spawning pthread..." << std::endl;
	std::string pilotMemStream = "";
	std::memset(region.get_address(), '\0', region.get_size());
	while(true) {
		std::getline(std::cin, pilotMemStream);
		std::strncpy((char*)region.get_address(), pilotMemStream.c_str(), 1000);
		if(strncmp(pilotMemStream.c_str(), "exit", 5) == 0) {
			pthread_join (flightControllerThread, NULL);
		}
		sleep(3);
	}
	*/

	std::cout << "exit"  << std::endl;
	return mraa::SUCCESS;
}
