#ifndef EPUCKBASE_HH
#define EPUCKBASE_HH
#include <stage.hh>

using namespace Stg;

#define NUM_IRS						8		//number of ir proximity sensor
#define MAX_PROXIMITY_RANGE 		115		//maximum proximity sensor range, in mm

#define WHEEL_DIA 					0.04
#define WHEEL_SEP 					0.052

class RobotBase
{
  protected:
    ModelPosition* pos;
    ModelRanger* ir;
    static int IRUpdate( Model* , RobotBase *);
    static int PositionUpdate( Model* model, RobotBase* robot) 
    {
      return robot->myPositionUpdate(model, robot);
    }
    virtual int myPositionUpdate(Model* model, RobotBase* robot) 
    {
      std::cout << "RobotBase::myPositionUpdate() was called, which should never happen" << std::endl;
      return -1;
    }
    int MAXSPEED;

  public:
    RobotBase(ModelPosition* pos);
    ~RobotBase();
    void SetSpeed(int lspeed, int rspeed);
    int proximity[NUM_IRS];
    char *name;
    void Avoidance();
    void PrintProximitySensor();

    unsigned char bumped;
};

#endif
