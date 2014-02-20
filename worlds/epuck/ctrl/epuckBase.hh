#ifndef EPUCKBASE_HH
#define EPUCKBASE_HH
#include <stage.hh>
#include <boost/random.hpp>
#include <boost/random/normal_distribution.hpp>

using namespace Stg;

#define NUM_IRS						8		//number of ir proximity sensor
#define MAX_PROXIMITY_RANGE 		115		//maximum proximity sensor range, in mm

#define WHEEL_DIA 					0.04
#define WHEEL_SEP 					0.052

class RobotBase
{
  protected:
    Model* myModel;
    ModelPosition* pos;
    ModelRanger* ir;
    static int IRUpdate( Model* , RobotBase *);
    static int PositionUpdate( Model* model, RobotBase* robot) 
    {
      robot->myModel = model;
      return robot->myPositionUpdate(model, robot);
    }
    virtual int myPositionUpdate(Model* model, RobotBase* robot) 
    {
      std::cout << "RobotBase::myPositionUpdate() was called, which should never happen" << std::endl;
      return -1;
    }
    boost::mt19937 rng;
    int MAXSPEED;
    float LeftWheelVelocity;
    float RightWheelVelocity;

  public:
    RobotBase(ModelPosition* pos);
    ~RobotBase();
    void SetSpeed(int lspeed, int rspeed);
    
    void GoStraight(float speed); // speed from [-1.0; 1.0]
    void TurnLeft();
    void TurnRight();
    bool Stop();
    void Follow(std::string name, float desiredDist);
    void MoveTo(float x, float y);
    void Avoidance();
    
    void PrintProximitySensor();

    int proximity[NUM_IRS];
    char *name;
    unsigned char bumped;
};

#endif