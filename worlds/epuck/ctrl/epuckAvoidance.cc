#ifndef EPUCKAVOIDANCE_HH
#define EPUCKAVOIDANCE_HH
#include <stage.hh>

using namespace Stg;

#include "epuckBase.cc"

class RobotAvoidance: public RobotBase
{
  public:
    RobotAvoidance(ModelPosition* pos):
      RobotBase(pos)
    {
      std::cout << "RobotAvoidance constructor" << std::endl;
      this->ctrlString = "GoStraight 0.6;Avoidance";
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
