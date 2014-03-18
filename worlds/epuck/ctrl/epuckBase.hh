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
    static int PositionUpdate( Model* model, RobotBase* robot); 
    boost::mt19937 rng;
    int MAXSPEED;
    float LeftWheelVelocity;
    float RightWheelVelocity;
    std::string ctrlString;

  public:
    RobotBase(ModelPosition* pos);
    ~RobotBase();
    void SetCtrlString(std::string);
    void SetSpeed(int lspeed, int rspeed);
    
    void GoStraight(float speed); // speed from [-1.0; 1.0]
    void TurnLeft(float speed);
    void TurnRight(float speed);
    bool Stop();
    void Follow(std::string name, float desiredDist);
    void MoveTo(float x, float y);
    void Avoidance();
    
    void PrintProximitySensor();

    int proximity[NUM_IRS];
    float smoothProximity;
    int offsetProximity[NUM_IRS];
    char *name;
    unsigned char bumped;
};

#endif
