#ifndef EPUCKFOLLOW_HH
#define EPUCKFOLLOW_HH
#include <stage.hh>

using namespace Stg;

#include "epuckBase.cc"

class RobotFollow: public RobotBase
{
  protected:
    virtual int myPositionUpdate(Model* model, RobotBase* robot)
    {
      this->MoveTo(0.0, 0.0);
      this->Avoidance();
      
      this->SetSpeed(LeftWheelVelocity,RightWheelVelocity);
      return 0;
    }

  public:
    RobotFollow(ModelPosition* pos):
      RobotBase(pos)
    {
      std::cout << "RobotFollow constructor" << std::endl;
    }
    ~RobotFollow()
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
    RobotFollow *robot = new RobotFollow((ModelPosition*)mod);
    return 0; //ok
}

#endif
