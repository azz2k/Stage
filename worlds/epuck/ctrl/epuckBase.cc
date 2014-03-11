#ifndef EPUCKBASE_CC
#define EPUCKBASE_CC
#include "epuckBase.hh"
#include <iostream>
#include <ctime>
#include <map>

int avoid_weightleft[8] = { -8, -3, -1, 2, 2, 1, 3, 6};
int avoid_weightright[8] = {6, 3, 1, 2, 2, -1, -3, -8};

#define DEBUG

#define IR_NUM 8
#define MAX_PROXIMITY_RANGE 115

#define clip(x) x * 1000 > MAX_PROXIMITY_RANGE ? MAX_PROXIMITY_RANGE : (x<0 ? 0 :(int)(x*1000))

double proximity_data[][2]={
	{974.40,	0.66},
	{962.40,	0.66},
	{956.40,	0.66},
	{953.59,	1.03},
	{950.21,	2.25},
	{944.27,	2.14},
	{938.90,	3.92},
	{930.82,	5.04},
	{901.28,	41.59},
	{753.71,	49.29},
	{635.67,	31.23},
	{518.06,	42.25},
	{428.50,	15.64},
	{371.08,	26.54},
	{312.98,	18.28},
	{274.02,	16.82},
	{244.17,	14.90},
	{213.76,	9.99},
	{190.03,	9.16},
	{171.25,	7.21},
	{152.12,	6.51},
	{138.31,	5.13},
	{126.22,	5.82},
	{117.58,	9.17},
	{107.05,	7.13},
	{94.30,	2.18},
	{87.67,	3.61},
	{79.73,	2.71},
	{76.10,	4.09},
	{71.13,	6.58},
	{64.09,	1.96},
	{61.11,	5.85},
	{59.19,	6.49},
	{55.56,	7.31},
	{47.79,	1.49},
	{45.68,	1.16},
	{43.93,	2.77},
	{41.19,	2.54},
	{38.72,	2.79},
	{37.64,	3.63},
	{33.85,	1.47},
	{34.35,	3.83},
	{31.57,	4.26},
	{29.06,	2.17},
	{27.16,	0.75},
	{25.89,	0.79},
	{25.79,	2.86},
	{24.90,	3.80},
	{22.63,	1.74},
	{22.20,	1.90},
	{20.56,	1.07},
	{20.46,	2.40},
	{18.73,	0.65},
	{18.34,	1.79},
	{17.05,	1.70},
	{17.16,	1.68},
	{17.34,	3.23},
	{15.24,	0.48},
	{14.80,	0.81},
	{14.85,	1.39},
	{14.07,	1.31},
	{13.50,	0.72},
	{12.63,	0.99},
	{12.79,	1.14},
	{12.73,	2.06},
	{12.16,	1.56},
	{11.59,	1.65},
	{11.38,	1.89},
	{11.05,	1.15},
	{11.20,	1.84},
	{10.70,	2.19},
	{9.91,	0.86},
	{9.66,	0.83},
	{10.06,	1.64},
	{9.13,	1.33},
	{9.66,	2.18},
	{8.99,	1.74},
	{8.52,	1.76},
	{8.25,	1.65},
	{7.42,	0.66},
	{7.22,	0.59},
	{6.92,	0.34},
	{7.17,	0.44},
	{7.26,	0.55},
	{7.14,	0.89},
	{6.70,	0.99},
	{6.44,	0.91},
	{6.35,	1.20},
	{6.57,	1.11},
	{6.01,	0.81},
	{5.99,	0.63},
	{5.64,	0.77},
	{6.00,	0.45},
	{5.36,	0.76},
	{5.48,	0.92},
	{5.30,	0.90},
	{5.39,	1.01},
	{5.12,	1.15},
	{4.74,	0.66},
	{4.66,	0.82},
	{4.68,	0.86},
	{4.66,	0.94},
	{4.56,	0.74},
	{4.35,	0.72},
	{4.35,	0.91},
	{4.02,	0.07},
	{4.03,	0.09},
	{4.00,	0.08},
	{4.36,	0.86},
	{4.43,	1.19},
	{4.40,	1.29},
	{3.86,	0.35},
	{4.00,	0.76},
	{4.40,	1.24},
	{3.51,	0.74},
	{3.40,	0.24},
	{3.00,	0.24},
	{2.40,	0.24},
	{2.00,	0.24},
	{1.40,	0.24}};

static std::map<std::string, RobotBase*> allRobotCtrl;


RobotBase::RobotBase(ModelPosition* pos):
  rng(static_cast<unsigned int>(std::time(0))),
  MAXSPEED(600),
  bumped(0)
{
    std::cout << "RobotBase constructor for " << pos->TokenStr() << std::endl;
    allRobotCtrl[pos->TokenStr()] = this;
    int i = 0;
    this->pos = pos;
    this->pos->AddCallback(Model::CB_UPDATE, (model_callback_t)this->PositionUpdate, this);
    this->pos->Subscribe();

    this->name = strdup(this->pos->Token());

    for (i = 0;i < NUM_IRS;i++)
        this->proximity[i] = 0;

    this->ir = (ModelRanger *) pos->GetUnusedModelOfType("ranger"); // find the names in libstage/typetable.cc
    if (this->ir)
    {
        this->ir->AddCallback(Model::CB_UPDATE,(model_callback_t)this->IRUpdate, this);
        this->ir->Subscribe();
    }
}

RobotBase::~RobotBase()
{
  //  printf("Destroy robot: %s\n", this->pos->Token());
    if (name)
    {
        free(name);
        name = NULL;
    }
}

// inspect the laser data and decide what to do
int RobotBase::IRUpdate( Model* mod, RobotBase *robot )
{
	
    robot->bumped = 0;
    for (int i = 0;i < NUM_IRS;i++)
    {
        const std::vector<ModelRanger::Sensor>& ir = ((ModelRanger*)mod)->GetSensors();

        float ir_min = 9999;
        for(unsigned int j=0;j<ir[i].ranges.size();j++)
        {
            if(ir[i].ranges[j]<ir_min)
            {
                ir_min = ir[i].ranges[j];
            }
        }
        robot->proximity[i] = proximity_data[clip(ir_min)][0];
        if (robot->proximity[i] > 10)
            robot->bumped |= 1 << i;
    }
    return 0;
}

int RobotBase::PositionUpdate( Model* model, RobotBase* robot) 
{
  robot->myModel = model;
 
  std::vector<std::string> commands;
  size_t pos = 0;
  while(true)
  {
    size_t nextpos = robot->ctrlString.find_first_of(";", pos);
    if(nextpos == std::string::npos)
    {
      commands.push_back(robot->ctrlString.substr(pos, robot->ctrlString.size()-1));
      break;
    } else
    {
      commands.push_back(robot->ctrlString.substr(pos, nextpos));
      pos = nextpos+1;
    }
  }

  for(std::vector<std::string>::iterator command = commands.begin(); command != commands.end(); command++)
  {
    std::cout << *command << std::endl;
  }

  robot->SetSpeed(robot->LeftWheelVelocity, robot->RightWheelVelocity);
  return 0;
}
    
void RobotBase::SetSpeed(int lspeed, int rspeed)
{
    if (lspeed < -MAXSPEED)
        lspeed = -MAXSPEED;
    if (lspeed > MAXSPEED)
        lspeed = MAXSPEED;
    if (rspeed < -MAXSPEED)
        rspeed = -MAXSPEED;
    if (rspeed > MAXSPEED)
        rspeed = MAXSPEED;

    double x, a;

    x = (lspeed + rspeed) * M_PI * WHEEL_DIA / (2 * 1000.0);
    a = (rspeed - lspeed) * M_PI * WHEEL_DIA * 2 / (WHEEL_SEP * 1000.0);

    this->pos->SetSpeed(x, 0, a);
}

void RobotBase::GoStraight(float speed)
{
  if(speed > 1.0)
    speed = 1.0;
  if(speed < -1.0)
    speed = -1.0;
  RightWheelVelocity = speed*MAXSPEED;
  LeftWheelVelocity = speed*MAXSPEED;
}

void RobotBase::TurnLeft()
{
  RightWheelVelocity = MAXSPEED*0.2;
  LeftWheelVelocity = -MAXSPEED*0.2;
}

void RobotBase::TurnRight()
{
  RightWheelVelocity = -MAXSPEED*0.2;
  LeftWheelVelocity = MAXSPEED*0.2;
}

bool RobotBase::Stop()
{
    LeftWheelVelocity = 0;
    RightWheelVelocity = 0;
    this->SetSpeed(LeftWheelVelocity, RightWheelVelocity);
    return true;
}

void RobotBase::Follow(std::string name, float desiredDist)
{
  boost::normal_distribution<> xygauss(0.0, 0.0);
  boost::normal_distribution<> agauss(0.0, 0.0);
  boost::variate_generator<boost::mt19937&, boost::normal_distribution<> > xygen(this->rng, xygauss);
  boost::variate_generator<boost::mt19937&, boost::normal_distribution<> > agen(this->rng, agauss);
  LeftWheelVelocity  = 0.0;
  RightWheelVelocity = 0.0;
//  if(robotsReal.find(name) != robotsReal.end())
  {
    float myx = this->pos->est_pose.x + xygen();
    float myy = this->pos->est_pose.y + xygen();
    float mya = this->pos->est_pose.a + agen();
    float otherx = this->myModel->GetWorld()->GetModel(name)->GetGlobalPose().x + xygen();
    float othery = this->myModel->GetWorld()->GetModel(name)->GetGlobalPose().y + xygen();

    float newa = atan2(othery-myy, otherx-myx);
    float steera = newa-mya;
    if(steera > M_PI)
      steera -= 2.0*M_PI;
    if(steera < -M_PI)
      steera += 2.0*M_PI;
    steera *= 500;
    
    float dist2 = (myx-otherx)*(myx-otherx) + (myy-othery)*(myy-othery);
    float steerd = 5000.0*(sqrt(dist2) - sqrt(desiredDist*desiredDist));
    if(steerd < 0.0)
      steerd *= 10.0;
    if(steerd > this->MAXSPEED)
      steerd = this->MAXSPEED;
    if(steerd < -this->MAXSPEED)
      steerd = -this->MAXSPEED;

    LeftWheelVelocity  = -steera + steerd;
    RightWheelVelocity = steera + steerd;

//    std::cout << robotsReal.size() << " " << robotsSim.size() << std::endl;
//    std::cout << "me: " << myx << " " << myy << " other: " << otherx << " " <<othery << std::endl;
//    std::cout << mya << " " << newa << " " << steera << std::endl;
//    std::cout << sqrt(dist2) << " " << desiredDist << " " << steerd << std::endl;
  }
}

void RobotBase::MoveTo(float x, float y)
{
  float myx = this->pos->est_pose.x;
  float myy = this->pos->est_pose.y;
  float mya = this->pos->est_pose.a;

  float newa = atan2(y-myy, x-myx);
  float steera = newa-mya;
  if(steera > M_PI)
    steera -= 2.0*M_PI;
  if(steera < -M_PI)
    steera += 2.0*M_PI;
  steera *= 500;
  
  float dist = sqrt((myx-x)*(myx-x) + (myy-y)*(myy-y));
  if(dist < 1e-2)
  {
    LeftWheelVelocity  = 0.0;
    RightWheelVelocity = 0.0;
    return;
  }
  float steerd = 5000.0*dist;
  if(steerd > this->MAXSPEED)
    steerd = this->MAXSPEED;
  if(steerd < -this->MAXSPEED)
    steerd = -this->MAXSPEED;

  LeftWheelVelocity  = -steera + steerd;
  RightWheelVelocity = steera + steerd;
}

void RobotBase::Avoidance()
{
    for (int i = 0; i < NUM_IRS; i++)
    {
        LeftWheelVelocity += avoid_weightleft[i] * (proximity[i]);
        RightWheelVelocity += avoid_weightright[i] * (proximity[i]);
    }
}

//below defined functions for debugging
void RobotBase::PrintProximitySensor()
{
    printf("%ld:%s IR --[", this->pos->GetWorld()->GetUpdateCount(), this->pos->Token());
    for (int i = 0;i < NUM_IRS;i++)
        printf(" %d", this->proximity[i]);
    printf("]\n");
}

#endif
