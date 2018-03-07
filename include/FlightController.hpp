#ifndef _FLIGHT_CONTROLLER
#define _FLIGHT_CONTROLLER

#include <boost/interprocess/shared_memory_object.hpp>
#include <boost/interprocess/mapped_region.hpp>
#include <sstream>	//stringstream
#include <iostream>	//cout

#include "../include/json.hpp"
using json = nlohmann::json;

#include "../include/utility/Properties.hpp"
#include "../include/PidController.hpp"

enum TransitionState { init, ready, test, fstart, flight, BAD_STATE };
enum TransitionRxEvent { rxNeutral, rxTest, rxStart, rxFalse_start, rxStop, rxDirection };

const struct {
	const std::string Neutral = "neutral";
	const std::string Test = "test";
	const std::string Start = "flight";
	const std::string FalseStart = "false-flight";
	const std::string Stop = "stop";
	const std::string Backward = "backward";
	const std::string Forward = "forward";
	const std::string Left = "left";
	const std::string Right = "right";
	const std::string Up = "up";
	const std::string Down = "down";
} DirectiveRxMessage;

class FlightController
{
public:
	FlightController();
	~FlightController();
	void run();
	void SIG_STOP();

private:
	// nextState = transition[currentState][incomingEvent]
	const TransitionState transitionTable[5][6] = {
			// rxNeutral				rxTest						rxStart						rxFalseStart				rxStop						rxDirection
			{TransitionState::init,		TransitionState::init,		TransitionState::init,		TransitionState::init,		TransitionState::init,		TransitionState::init},		// init
			{TransitionState::ready,	TransitionState::test,		TransitionState::flight,	TransitionState::fstart,	TransitionState::ready,		TransitionState::ready},	// ready
			{TransitionState::test,		TransitionState::test,		TransitionState::test,		TransitionState::test,		TransitionState::ready,		TransitionState::test},		// test
			{TransitionState::fstart,	TransitionState::fstart,	TransitionState::fstart,	TransitionState::fstart,	TransitionState::ready,		TransitionState::fstart},	// fstart
			{TransitionState::flight,	TransitionState::flight,	TransitionState::flight,	TransitionState::flight,	TransitionState::ready,		TransitionState::flight}		// flight
	};

	boost::interprocess::shared_memory_object* sharedMemTcpRX;
	boost::interprocess::shared_memory_object* sharedMemTcpTX;
	boost::interprocess::shared_memory_object* runtimePidControllerMemory;
	boost::interprocess::mapped_region* regionRX;
	boost::interprocess::mapped_region* regionTX;
	int currentState;
	TransitionRxEvent lastEvent;

	Properties* flightControllerProperties;
	Properties* vipleMotionProperties;
	volatile int* channels[4];
	double runningSetPoints[3];
	double runningBaselineThrottle;
	PidController* pidController;
	const char* statusString;
	const char* directiveString;
	vector<std::string> vipleString;

	std::string _parseDirectiveFromRxSharedMemory();
	TransitionRxEvent _stringToEvent(std::string event);
	int _eventToIndex(TransitionRxEvent rxEvent);
	void _updateState(TransitionRxEvent rxEvent);
	void _iterateCurrentState();
	void _updatePidController(TransitionRxEvent rxEvent, std::string rxEventString);
	void _trimJsonToCString(json jsonToTrim, char* destinationBuffer, int buflength);

};

#endif //_FLIGHT_CONTROLLER
