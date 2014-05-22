#ifndef EPUCKFLOCK_HH
#define EPUCKFLOCK_HH
#include <stage.hh>

using namespace Stg;

#include "epuckBase.cc"

class RobotFlock: public RobotBase
{
  public:
    RobotFlock(ModelPosition* pos):
      RobotBase(pos)
    {
      std::cout << "RobotFlock constructor" << std::endl;
      this->ctrlString = "Flock 0.3 epuck12 epuck21 epuck22 epuck29 epuck32 epuck46;Avoidance";
    }
    ~RobotFlock()
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
    RobotFlock *robot = new RobotFlock((ModelPosition*)mod);
    return 0; //ok
}

#endif
