#ifndef EPUCKAVOIDANCE_HH
#define EPUCKAVOIDANCE_HH
#include <stage.hh>

using namespace Stg;

#include "epuckBase.cc"

class RobotAvoidance: public RobotBase
{
  protected:
    virtual int myPositionUpdate(Model* model, RobotBase* robot)
    {
      this->GoStraight(0.8);
      this->Avoidance();
      
      this->SetSpeed(LeftWheelVelocity,RightWheelVelocity);
      return 0;
    }

  public:
    RobotAvoidance(ModelPosition* pos):
      RobotBase(pos)
    {
      std::cout << "RobotAvoidance constructor" << std::endl;
    }
    ~RobotAvoidance()
    {
      if (name)
      {
          free(name);
          name = NULL;
      }
    }
};

// Stage calls this when the model starts up
extern "C" int Init( Model* mod )
{
    RobotAvoidance *robot = new RobotAvoidance((ModelPosition*)mod);
    return 0; //ok
}

#endif
