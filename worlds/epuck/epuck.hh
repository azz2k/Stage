#ifndef EPUCK_HH
#define EPUCK_HH
#include <stage.hh>

using namespace Stg;

#define NUM_IRS						8		//number of ir proximity sensor
#define MAX_PROXIMITY_RANGE 		115		//maximum proximity sensor range, in mm

#define WHEEL_DIA 					0.04
#define WHEEL_SEP 					0.052


enum fsm_state_t{FLOCKING=0,STATE_COUNT};
const char *state_name[STATE_COUNT]={"Flocking"};

class Robot
{
private:
	ModelPosition* pos;
	ModelRanger* ir;
	static int IRUpdate( Model* , Robot *);
	static int PositionUpdate( Model* , Robot* );

public:
	Robot(ModelPosition* pos);
	~Robot();
	void SetSpeed(int lspeed, int rspeed);
	int proximity[NUM_IRS];
	char *name;
	void Avoidance();
	void Flocking();
	void PrintProximitySensor();
	void PrintState();
	fsm_state_t current_state, last_state, previous_state;

	unsigned char bumped;
};

#endif
