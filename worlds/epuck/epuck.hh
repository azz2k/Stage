#ifndef EPUCK_HH
#define EPUCK_HH
#include <stage.hh>

using namespace Stg;

#define NUM_IRS						8		//number of ir proximity sensor
#define MAX_PROXIMITY_RANGE 		115		//maximum proximity sensor range, in mm

#define WHEEL_DIA 					0.04
#define WHEEL_SEP 					0.052

class Robot
{
private:
	ModelPosition* pos;
	ModelRanger* ir;
	static int IRUpdate( Model* , Robot *);
	static int PositionUpdate( Model* , Robot* );
  int MAXSPEED;

public:
	Robot(ModelPosition* pos);
	~Robot();
	void SetSpeed(int lspeed, int rspeed);
	int proximity[NUM_IRS];
	char *name;
	void Avoidance();
	void PrintProximitySensor();

	unsigned char bumped;
};

#endif
