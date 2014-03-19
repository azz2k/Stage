#ifndef EPUCKStop_HH
#define EPUCKStop_HH
#include <stage.hh>

using namespace Stg;

#include "epuckBase.cc"

class RobotStop: public RobotBase
{
  public:
    RobotStop(ModelPosition* pos):
      RobotBase(pos)
    {
      std::cout << "RobotStop constructor" << std::endl;
      this->ctrlString = "Stop";
    }
    ~RobotStop()
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
    RobotStop *robot = new RobotStop((ModelPosition*)mod);
    return 0; //ok
}

#endif
