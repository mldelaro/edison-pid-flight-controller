#ifndef _FLIGHT_CONTROLLER
#define _FLIGHT_CONTROLLER

#include <boost/interprocess/shared_memory_object.hpp>
#include <boost/interprocess/mapped_region.hpp>
#include <sstream>	//stringstream
#include <iostream>	//cout


#include "../include/utility/Properties.hpp"
#include "../include/PidController.hpp"


class FlightController
{
public:
	FlightController();
	~FlightController();
	void run();

private:
	enum TransitionState { init, ready, test, fstart, flight, BAD_STATE};
	enum class TransitionEvent : char {
							rxNeutral = 'S',
							rxTest = 'T',
							rxStart = 'V',
							rxFalseStart = 'Y',
							rxStop = 'W',
							rxDirection = 'X' //translated from up/down/left/right/forward/backward
						 };

	// nextState = transition[currentState][incomingEvent]
	const TransitionState transitionTable[5][6] = {
			// rxNeutral				rxTest						rxStart						rxFalseStart				rxStop						rxDirection
			{TransitionState::init,		TransitionState::init,		TransitionState::init,		TransitionState::init,		TransitionState::init,		TransitionState::init},		// init
			{TransitionState::ready,	TransitionState::test,		TransitionState::flight,	TransitionState::fstart,	TransitionState::ready,		TransitionState::ready},	// ready
			{TransitionState::test,		TransitionState::test,		TransitionState::test,		TransitionState::test,		TransitionState::ready,		TransitionState::test},		// test
			{TransitionState::fstart,	TransitionState::fstart,	TransitionState::fstart,	TransitionState::fstart,	TransitionState::ready,		TransitionState::fstart},	// fstart
			{TransitionState::flight,	TransitionState::flight,	TransitionState::flight,	TransitionState::flight,	TransitionState::ready,		TransitionState::flight}		// flight
	};

	boost::interprocess::shared_memory_object* runtimeUdpReceiver;
	boost::interprocess::shared_memory_object* runtimePidControllerMemory;
	TransitionState currentState;
	TransitionEvent lastEvent;

	pthread_t flightControllerThread;
	Properties* flightControllerProperties;
	volatile int* channels[4];
	std::string pilotMemStream;

	PidController* pidController;

	char _parseEventCharFromRxSharedMemory();
	TransitionEvent _charToEvent(char event);
	int _eventToIndex(TransitionEvent event);
	void _updateState(TransitionEvent rxEvent);
	void _iterateCurrentState();



};

#endif //_FLIGHT_CONTROLLER
