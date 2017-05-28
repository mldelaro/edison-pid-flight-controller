#include "../include/FlightController.hpp"



FlightController::FlightController() {
	bool udpRuntimeFound = false;
	while(!udpRuntimeFound) {
		try {
			runtimeFlightControllerMemory = new boost::interprocess::shared_memory_object(
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
	currentState = FlightController::TransitionState::init;
	lastEvent = FlightController::TransitionEvent::rxNeutral;
}

FlightController::~FlightController() {
	runtimeFlightControllerMemory = NULL;
}

void FlightController::run() {
	TransitionEvent currentRxEvent = TransitionEvent::rxNeutral;

	// Start driver loop
	while(true) {
		TransitionEvent newRxEvent = _charToEvent(_parseEventCharFromRxSharedMemory());
		if(currentRxEvent != newRxEvent) {
			// change in shared memory; check for valid next state
			_updateState(newRxEvent);
		} else {
			_iterateCurrentState();
		}
	}// end run()
}

/* PRIVATE MEMBER FUNCTIONS */
char FlightController::_parseEventCharFromRxSharedMemory() {
	// read event from shared memory
	std::stringstream stream;
	boost::interprocess::mapped_region region(*runtimeFlightControllerMemory, boost::interprocess::read_write);
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
			std::cout << "Currently at INIT" << std::endl;

			break;
		case TransitionState::ready :
			std::cout << "Currently at READY" << std::endl;
			break;
		case TransitionState::test :
			std::cout << "Currently at TEST" << std::endl;
			break;
		case TransitionState::fstart :
			std::cout << "Currently at FSTART" << std::endl;
			break;
		case TransitionState::flight :
			std::cout << "Currently at FLIGHT" << std::endl;
			break;
		default:
			std::cout << "Currently at BAD_STATE" << std::endl;
			break;
	}
	usleep(1000000);
}


