#include "../include/FlightController.hpp"

FlightController::FlightController() {
	pidController = NULL;
	flightControllerProperties = NULL;
	bool udpRuntimeFound = false;
	runningSetPoints[0] = 0;
	runningSetPoints[1] = 0;
	runningSetPoints[2] = 0;
	runningBaselineThrottle = 0;

	// Don't run until remote is initialized
	while(!udpRuntimeFound) {
		try {
			runtimeUdpReceiver = new boost::interprocess::shared_memory_object(
					boost::interprocess::open_only,
					"shared_mem_udp_receiver",
					boost::interprocess::read_write
			);
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
	runtimeUdpReceiver = NULL;
	runtimePidControllerMemory = NULL;
}

void FlightController::run() {
	// Start driver loop
	char charNewRxEvent;
	while(true) {
		charNewRxEvent = _parseEventCharFromRxSharedMemory();
		TransitionEvent newRxEvent = _charToEvent(charNewRxEvent);
		if(lastEvent != newRxEvent) {
			// change in shared memory; check for valid next state
			_updateState(newRxEvent);
		}

		if(currentState == TransitionState::flight || currentState == TransitionState::fstart){
			_updatePidController(charNewRxEvent);
		}
		_iterateCurrentState();
	}// end run()
}

/* PRIVATE MEMBER FUNCTIONS */
char FlightController::_parseEventCharFromRxSharedMemory() {
	// read event from shared memory
	std::stringstream stream;
	boost::interprocess::mapped_region region(*runtimeUdpReceiver, boost::interprocess::read_write);
	stream.rdbuf()->pubsetbuf((char*)region.get_address(), region.get_size());
	std::string rxSharedMemory = stream.str();

	return rxSharedMemory[0];
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
				std::cout << "==== INIT ====" << std::endl;
				pilotMemStream = "S";
				Properties* pidConfigProperties = new Properties("/home/root/runtime/FlightController/flightController.properties");
				PidConfig* pidConfig = new PidConfig(pidConfigProperties);
				pidController = new PidController(pidConfig);
				pidController->setup();
				currentState = TransitionState::ready;
				std::cout << "==== READY ====" << std::endl;
			}
			break;
		case TransitionState::ready :
			{
				pidController->_STOP();
				break;
			}
		case TransitionState::test :
		{
			std::cout << "==== TEST ====" << std::endl;
			pidController->_TEST_ROTORS();
			currentState = TransitionState::ready;
			std::cout << "==== READY ====" << std::endl;
			break;
		}
		case TransitionState::fstart :
			{
				pidController->iterativeLoop(false);
				break;
			}
		case TransitionState::flight :
			{
				pidController->iterativeLoop(true);
				break;
			}
		default:
			{
				std::cout << "Currently at BAD_STATE" << std::endl;
				break;
			}
	}
}

void FlightController::_updatePidController(char rxEvent) {
	switch(rxEvent) {
		case 'B':
			pidController->setPitchDPS(10);
			break;
		case 'F':
			pidController->setPitchDPS(-10);
			break;
		case 'L':
			pidController->setRollDPS(10);
			break;
		case 'R':
			pidController->setRollDPS(-10);
			break;
		case 'U':
			pidController->incrementBaselineThrottle(1);
			break;
		case 'D':
			pidController->incrementBaselineThrottle(-1);
			break;
	}
}
