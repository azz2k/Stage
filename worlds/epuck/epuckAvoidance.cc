#ifndef EPUCKAVOIDANCE_HH
#define EPUCKAVOIDANCE_HH
#include <stage.hh>

using namespace Stg;

#include "epuckBase.cc"

class RobotAvoidance: public RobotBase
{
  private:
    int myPositionUpdate(Model* model, RobotBase* robot)
    {
      std::cout << "RobotAvoidance::myPositionUpdate() was called correctly" << std::endl;
      robot->Avoidance();
      return 0;
    }


  public:
    RobotAvoidance(ModelPosition* pos):
      RobotBase(pos)
    { }
    ~RobotAvoidance()
    {
      if (name)
      {
          free(name);
          name = NULL;
      }
    }
  };

#endif
