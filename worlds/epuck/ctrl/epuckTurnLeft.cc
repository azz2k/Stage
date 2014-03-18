#ifndef EPUCKTurnLeft_HH
#define EPUCKTurnLeft_HH
#include <stage.hh>

using namespace Stg;

#include "epuckBase.cc"

class RobotTurnLeft: public RobotBase
{
  public:
    RobotTurnLeft(ModelPosition* pos):
      RobotBase(pos)
    {
      std::cout << "RobotTurnLeft constructor" << std::endl;
      this->ctrlString = "TurnLeft 0.2";
    }
    ~RobotTurnLeft()
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
    RobotTurnLeft *robot = new RobotTurnLeft((ModelPosition*)mod);
    return 0; //ok
}

#endif
