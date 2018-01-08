#include "../include/FlightController.hpp"
#include "../include/json.hpp" // IMPROVE: Move to header

#define BUFLEN 512  //Max length of buffer

using json = nlohmann::json;

FlightController::FlightController() {
	pidController = NULL;
	flightControllerProperties = NULL;
	bool udpRuntimeFound = false;
	runningSetPoints[0] = 0;
	runningSetPoints[1] = 0;
	runningSetPoints[2] = 0;
	runningBaselineThrottle = 0;
	directiveString = "standby";

	// Don't run until remote is initialized
	while(!udpRuntimeFound) {
		try {
			sharedMemUdpRX = new boost::interprocess::shared_memory_object(
					boost::interprocess::open_only,
					"shared_mem_udp_RX",
					boost::interprocess::read_write
			);
			regionRX = new boost::interprocess::mapped_region(*sharedMemUdpRX, boost::interprocess::read_write);

			sharedMemUdpTX = new boost::interprocess::shared_memory_object(
					boost::interprocess::open_only,
					"shared_mem_udp_TX",
					boost::interprocess::read_write
			);
			regionTX = new boost::interprocess::mapped_region(*sharedMemUdpTX, boost::interprocess::read_write);

			udpRuntimeFound = true;
		}  catch(boost::interprocess::interprocess_exception& ex) {
			std::cout << "Failed to open shared memory for UDP receiver..." << std::endl;
			sleep(3);
		}
	}

	try {
		runtimePidControllerMemory = new boost::interprocess::shared_memory_object(
				boost::interprocess::open_only,
				"shared_mem_pilot",
				boost::interprocess::read_write
		);
		udpRuntimeFound = true;
	}  catch(boost::interprocess::interprocess_exception& ex) {
		std::cout << "Failed to open shared memory for PID Controller..." << std::endl;
	}

	currentState = FlightController::TransitionState::init;
	lastEvent = FlightController::TransitionEvent::rxNeutral;
}

FlightController::~FlightController() {
	sharedMemUdpRX = NULL;
	sharedMemUdpTX = NULL;
	runtimePidControllerMemory = NULL;
}

void FlightController::run() {
	// Start driver loop
	char charNewRxEvent;
	while(true) {
		charNewRxEvent = _parseEventCharFromRxSharedMemory();
		TransitionEvent newRxEvent = _charToEvent(charNewRxEvent);
		if(lastEvent != newRxEvent) {
			// event changed in shared memory; check for valid next state
			_updateState(newRxEvent);
		}

		// state is either in flight, or in a false flight
		if(currentState == TransitionState::flight || currentState == TransitionState::fstart){
			_updatePidController(charNewRxEvent);
		}
		_iterateCurrentState();
	}// end run()
}

/* PRIVATE MEMBER FUNCTIONS */
char FlightController::_parseEventCharFromRxSharedMemory() {
	// read event from shared memory
	std::stringstream rxStream;
	rxStream.rdbuf()->pubsetbuf((char*)regionRX->get_address(), regionRX->get_size());
	std::string rxSharedMemory = rxStream.str();
	// parse directive from RX json
//	std::cout << "Parsing: " << rxSharedMemory << std::endl;
	try {
		json jsonRxUDP = json::parse(rxSharedMemory);
		string directive = jsonRxUDP.value("command", "S");
//		std::cout << "Parsed: " << directive << std::endl;
		return directive[0];
	} catch(std::exception e) {
		std::cout << "Failed to parse " << rxSharedMemory << std::endl;
		//strncpy((char*)regionRX->get_address(), "{\"command\":\"S\"}\0", BUFLEN);
		return 'S';
	}
}

FlightController::TransitionEvent FlightController::_charToEvent(char rxEvent) {
	TransitionEvent eventToReturn = TransitionEvent::rxNeutral;
	switch(rxEvent) {
		case 'B':
			eventToReturn = TransitionEvent::rxDirection;
			break;
		case 'F':
			eventToReturn = TransitionEvent::rxDirection;
			break;
		case 'L':
			eventToReturn = TransitionEvent::rxDirection;
			break;
		case 'R':
			eventToReturn = TransitionEvent::rxDirection;
			break;
		case 'U':
			eventToReturn = TransitionEvent::rxDirection;
			break;
		case 'D':
			eventToReturn = TransitionEvent::rxDirection;
			break;
		default:
			eventToReturn = static_cast<TransitionEvent>(rxEvent);
			break;
	}
	return eventToReturn;
}

int FlightController::_eventToIndex(TransitionEvent event) {
	int indexToReturn = 0;
	switch(event) {
			case TransitionEvent::rxNeutral :
				indexToReturn = 0;
				break;
			case TransitionEvent::rxTest:
				indexToReturn = 1;
				break;
			case TransitionEvent::rxStart:
				indexToReturn = 2;
				break;
			case TransitionEvent::rxFalseStart:
				indexToReturn = 3;
				break;
			case TransitionEvent::rxStop:
				indexToReturn = 4;
				break;
			case TransitionEvent::rxDirection:
				indexToReturn = 5;
				break;
			default:
				indexToReturn = 0;
				break;
		}
	return indexToReturn;
}


void FlightController::_updateState(TransitionEvent rxEvent) {
	TransitionState newState = currentState;
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
				pilotMemStream = "S";
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
			std::cout << "==== TEST ====" << std::endl;
			pidController->_TEST_ROTORS();
			currentState = TransitionState::ready;
			std::cout << "==== READY ====" << std::endl;
			break;
		}
		case TransitionState::fstart :
			{
				statusString = "false-start";
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

	// copy TX_JSON message into the shared memory
	strncpy((char*)regionTX->get_address(), jsonTx.dump().c_str(), BUFLEN);
}

void FlightController::_updatePidController(char rxEvent) {
	switch(rxEvent) {
		case 'B':
			directiveString = "backward";
			pidController->setPitchDPS(120);
			break;
		case 'F':
			directiveString = "forward";
			pidController->setPitchDPS(-120);
			break;
		case 'L':
			directiveString = "left";
			pidController->setRollDPS(-120);
			break;
		case 'R':
			directiveString = "right";
			pidController->setRollDPS(120);
			break;
		case 'U':
			directiveString = "up";
			pidController->incrementBaselineThrottle(0.05);
			break;
		case 'D':
			directiveString = "down";
			pidController->incrementBaselineThrottle(-0.05);
			break;
		case 'S':
			directiveString = "standby";
			pidController->setPitchDPS(0);
			pidController->setRollDPS(0);
			break;
	}
}
