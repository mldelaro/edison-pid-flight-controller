#include "../include/FlightController.hpp"

#define BUFLEN 512  //Max length of buffer

FlightController::FlightController() {
	statusString = "initializing";
	pidController = NULL;
	flightControllerProperties = NULL;
	vipleMotionProperties = NULL;
	bool tcpRuntimeFound = false;
	runningSetPoints[0] = 0;
	runningSetPoints[1] = 0;
	runningSetPoints[2] = 0;
	runningBaselineThrottle = 0;
	directiveString = "standby";

	vipleString.push_back("neutral");
	vipleString.push_back("false-flight");
	vipleString.push_back("stop");
	vipleString.push_back("forward");
	vipleString.push_back("left");
	vipleString.push_back("right");
	vipleString.push_back("backward");
	vipleString.push_back("up");
	vipleString.push_back("down");
	vipleString.push_back("stop");

	// Don't run until remote is initialized
	while(!tcpRuntimeFound) {
		try {
			sharedMemTcpRX = new boost::interprocess::shared_memory_object(
					boost::interprocess::open_only,
					"shared_mem_tcp_RX",
					boost::interprocess::read_write
			);
			regionRX = new boost::interprocess::mapped_region(*sharedMemTcpRX, boost::interprocess::read_write);

			sharedMemTcpTX = new boost::interprocess::shared_memory_object(
					boost::interprocess::open_only,
					"shared_mem_tcp_TX",
					boost::interprocess::read_write
			);
			regionTX = new boost::interprocess::mapped_region(*sharedMemTcpTX, boost::interprocess::read_write);

			tcpRuntimeFound = true;
		}  catch(boost::interprocess::interprocess_exception& ex) {
			std::cout << "Failed to open shared memory for TCP receiver..." << std::endl;
			sleep(3);
		}
	}

	try {
		runtimePidControllerMemory = new boost::interprocess::shared_memory_object(
				boost::interprocess::open_only,
				"shared_mem_pilot",
				boost::interprocess::read_write
		);
		tcpRuntimeFound = true;
	}  catch(boost::interprocess::interprocess_exception& ex) {
		std::cout << "Failed to open shared memory for PID Controller..." << std::endl;
	}

	currentState = TransitionState::init;
	lastEvent = TransitionRxEvent::rxNeutral;
}

FlightController::~FlightController() {
	delete pidController;
	delete sharedMemTcpRX;
	delete sharedMemTcpTX;
	delete runtimePidControllerMemory;
	delete flightControllerProperties;
	delete vipleMotionProperties;
}

void FlightController::run() {
	// Start driver loop
	std::string newRxEventString = "neutral";
	TransitionRxEvent newRxEvent = TransitionRxEvent::rxNeutral;
	while(true) {
		newRxEventString = _parseDirectiveFromRxSharedMemory();
		newRxEvent = _stringToEvent(newRxEventString);
		if(newRxEvent != lastEvent) {
			// event changed in shared memory; check for valid next state
			_updateState(newRxEvent);
		}

		// state is either in flight, or in a false flight
		if(currentState == TransitionState::flight || currentState == TransitionState::fstart){
			_updatePidController(newRxEvent, newRxEventString);
		}
		_iterateCurrentState();
	}// end run()
}

void FlightController::SIG_STOP() {
	std::cout << "[Flight_Controller] Received stop..." << std::endl;
	if(pidController) {
		std::cout << "[Flight_Controller] Stopping PID Controller..." << std::endl;
		pidController->_STOP();
	}

	// Write 'STOP' to RX buffer
	if(regionTX) {
		std::cout << "[Flight_Controller] Writing stop message to TCP_TX..." << std::endl;
		std::string STOP_MESSAGE = "{\"status\":\"stopped\"}\n";
		std::strcpy((char*)regionTX->get_address(), STOP_MESSAGE.c_str());
	}
}

/* PRIVATE MEMBER FUNCTIONS */
std::string FlightController::_parseDirectiveFromRxSharedMemory() {
	// read event from shared memory
	std::stringstream rxStream;
	rxStream.rdbuf()->pubsetbuf((char*)regionRX->get_address(), regionRX->get_size());
	std::string rxSharedMemory = rxStream.str();
	char buffer[BUFLEN];
	std::memset(buffer, '\0', BUFLEN); // clear buffer
	strncpy(buffer, (char*)regionRX->get_address() , BUFLEN); //copy RX message to buffer
	rxStream.str(std::string()); // flush rxStream

	// parse directive from RX json
//	try {
//		if(buffer[0] == '{') {
//			json jsonRxTCP = json::parse(buffer);
//			std::string directive = jsonRxTCP.value("command", "neutral"); // default to a standby command
//			return directive;
//		} else {
//			std::cout << "Waiting for TCP runtime..." << std::endl;
//			sleep(5);
//		}
//	} catch(json::parse_error& e) {
//		std::cout << "Failed to parse " << buffer << std::endl;
//		std::cout << e.what() << " at position: " << e.byte << std::endl;
//		sleep(3);
//		return "neutral";
//	} catch(std::exception& e) {
//		std::cout << "Failed to parse " << buffer << std::endl;
//		std::cout << e.what() << std::endl;
//		sleep(3);
//		return "neutral";
//	}



//	std::cout << "parsing directive...";
	try {
			if(buffer[0] == '{') {
				json jsonRxTCP = json::parse(buffer);
				json rxMotions = jsonRxTCP["motions"]; // default to a standby command
				json rxMotion = rxMotions[0];
				int rxMotionId = rxMotion["motionId"];
				std::string directive = vipleString[rxMotionId];
//				std::cout << "rxDirective: " << directive;
				return directive;
			} else {
				std::cout << "Waiting for TCP runtime..." << std::endl;
				sleep(5);
			}
		} catch(json::parse_error& e) {
			std::cout << "Failed to parse " << buffer << std::endl;
			std::cout << e.what() << " at position: " << e.byte << std::endl;
			sleep(3);
			return "neutral";
		} catch(std::exception& e) {
			std::cout << "Failed to parse " << buffer << std::endl;
			std::cout << e.what() << std::endl;
			sleep(3);
			return "neutral";
		}
//	std::cout << "Reverting to neutral";
	return "neutral";
}

TransitionRxEvent FlightController::_stringToEvent(std::string rxEventString) {
	TransitionRxEvent eventToReturn = TransitionRxEvent::rxNeutral;
	if(rxEventString.compare(DirectiveRxMessage.Backward) == 0 ||
			rxEventString.compare(DirectiveRxMessage.Down) == 0 ||
			rxEventString.compare(DirectiveRxMessage.Forward) == 0 ||
			rxEventString.compare(DirectiveRxMessage.Left) == 0 ||
			rxEventString.compare(DirectiveRxMessage.Right) == 0 ||
			rxEventString.compare(DirectiveRxMessage.Up) == 0) {
		return TransitionRxEvent::rxDirection;
	} else if(rxEventString.compare(DirectiveRxMessage.FalseStart) == 0) {
		return TransitionRxEvent::rxFalse_start;
	} else if(rxEventString.compare(DirectiveRxMessage.Neutral) == 0) {
		return TransitionRxEvent::rxNeutral;
	} else if(rxEventString.compare(DirectiveRxMessage.Start) == 0) {
		return TransitionRxEvent::rxStart;
	} else if(rxEventString.compare(DirectiveRxMessage.Stop) == 0) {
		return TransitionRxEvent::rxStop;
	} else if(rxEventString.compare(DirectiveRxMessage.Test) == 0) {
		return TransitionRxEvent::rxTest;
	} else {
		std::cout << "Failed to convert string to event... setting to neutral" << std::endl;
		return TransitionRxEvent::rxNeutral;
	}
	return eventToReturn;
}

int FlightController::_eventToIndex(TransitionRxEvent event) {
	int indexToReturn = 0;
	switch(event) {
			case TransitionRxEvent::rxNeutral :
				indexToReturn = 0;
				break;
			case TransitionRxEvent::rxTest:
				indexToReturn = 1;
				break;
			case TransitionRxEvent::rxStart:
				indexToReturn = 2;
				break;
			case TransitionRxEvent::rxFalse_start:
				indexToReturn = 3;
				break;
			case TransitionRxEvent::rxStop:
				indexToReturn = 4;
				break;
			case TransitionRxEvent::rxDirection:
				indexToReturn = 5;
				break;
			default:
				indexToReturn = 0;
				break;
		}
	return indexToReturn;
}


void FlightController::_updateState(TransitionRxEvent rxEvent) {
	int newState = currentState;
	newState = transitionTable[currentState][_eventToIndex(rxEvent)];
	currentState = newState;
}

/* BRANCHING FLIGHT CONTROLLER*/
void FlightController::_iterateCurrentState() {
	switch(currentState) {
		case TransitionState::init :
			{
				statusString = "initializing";
				std::cout << "==== INIT ====" << std::endl;
				Properties* pidConfigProperties = new Properties("/home/root/flight-controller/flight-controller.properties");
				PidConfig* pidConfig = new PidConfig(pidConfigProperties);
				pidController = new PidController(pidConfig);
				pidController->setup();
				currentState = TransitionState::ready;
				std::cout << "==== READY ====" << std::endl;
			}
			break;
		case TransitionState::ready :
			{
				statusString = "standby";
				pidController->_STOP();
				break;
			}
		case TransitionState::test :
		{
			statusString = "testing";
			directiveString = "test";
			std::cout << "==== TEST ====" << std::endl;

			// Test is a blocking thread...
			// Write updated status before starting rotor test
			try {
				json jsonTx = {
					{"status", statusString},
					{"directive", directiveString},
					{"gyroX", pidController->getNormalizedGyroX()},
					{"gyroY", pidController->getNormalizedGyroY()},
					{"gyroZ", pidController->getNormalizedGyroZ()},
					{"accX", pidController->getAccelerationX()},
					{"accY", pidController->getAccelerationY()},
					{"accZ", pidController->getAccelerationZ()}
				};
				_trimJsonToCString(jsonTx, (char*)regionTX->get_address(), BUFLEN);
			} catch (std::exception* e) {
				std::cout << "Failed to build TX Message: " << e->what();
				json jsonTx = {
					{"status", "testing"},
					{"directive", "N/A"}
				};
				_trimJsonToCString(jsonTx, (char*)regionTX->get_address(), BUFLEN);
			}

			pidController->_TEST_ROTORS();
			currentState = TransitionState::ready;
			std::cout << "==== READY ====" << std::endl;
			break;
		}
		case TransitionState::fstart :
			{
				statusString = "false-flight";
				pidController->iterativeLoop(false);
				break;
			}
		case TransitionState::flight :
			{
				statusString = "in-flight";
				pidController->iterativeLoop(true);
				break;
			}
		default:
			{
				statusString = "BAD_STATUS";
				std::cout << "Currently at BAD_STATE" << std::endl;
				break;
			}
	}
	try {
		json jsonTx = {
			{"status", statusString},
			{"directive", directiveString},
			{"gyroX", pidController->getNormalizedGyroX()},
			{"gyroY", pidController->getNormalizedGyroY()},
			{"gyroZ", pidController->getNormalizedGyroZ()},
			{"accX", pidController->getAccelerationX()},
			{"accY", pidController->getAccelerationY()},
			{"accZ", pidController->getAccelerationZ()}
		};
		_trimJsonToCString(jsonTx, (char*)regionTX->get_address(), BUFLEN);
	} catch (std::exception* e) {
		std::cout << "Failed to build TX Message: " << e->what();
		json jsonTx = {
			{"status", "N/A"},
			{"directive", "N/A"}
		};
		_trimJsonToCString(jsonTx, (char*)regionTX->get_address(), BUFLEN);
	}
}

void FlightController::_updatePidController(TransitionRxEvent rxEvent, std::string rxEventString) {
		if(rxEventString.compare(DirectiveRxMessage.Backward) == 0) {
			directiveString = "backward";
			pidController->setPitchDPS(20);
		} else if(rxEventString.compare(DirectiveRxMessage.Forward) == 0) {
			directiveString = "forward";
			pidController->setPitchDPS(-20);
		} else if(rxEventString.compare(DirectiveRxMessage.Left) == 0) {
			directiveString = "left";
			pidController->setRollDPS(-20);
		} else if(rxEventString.compare(DirectiveRxMessage.Right) == 0) {
			directiveString = "right";
			pidController->setRollDPS(20);
		} else if(rxEventString.compare(DirectiveRxMessage.Up) == 0) {
			directiveString = "up";
			pidController->incrementBaselineThrottle(0.1);
		} else if(rxEventString.compare(DirectiveRxMessage.Down) == 0) {
			directiveString = "down";
			pidController->setPitchDPS(-0.1);
		} else if(rxEventString.compare(DirectiveRxMessage.Neutral) == 0) {
			directiveString = "standby";
			pidController->setPitchDPS(0);
			pidController->setRollDPS(0);
		}
}

void FlightController::_trimJsonToCString(json jsonToTrim, char* destinationBuffer, int buflength) {
	// copy TX_JSON message into a temporary buffer
	std::string jsonTxDump = jsonToTrim.dump();
	char tempBuffer[BUFLEN];
	strncpy((char*)tempBuffer, jsonTxDump.c_str(), buflength);
	int i = 0;
	while(tempBuffer[i] != '}' && i < buflength) {
		i++;
	}

	// append delimiter(\n) and string terminator(\0)
	if(buflength <= (i+1)) {
		tempBuffer[buflength-2] = '\n';
		tempBuffer[buflength-1] = '\0';
	} else {
		tempBuffer[i+1] = '\n';
		tempBuffer[i+2] = '\0';
	}

	strncpy(destinationBuffer, tempBuffer, buflength);
}
