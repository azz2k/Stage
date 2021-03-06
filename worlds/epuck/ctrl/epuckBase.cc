#ifndef EPUCKBASE_CC
#define EPUCKBASE_CC
#include "epuckBase.hh"
#include <iostream>
#include <iterator>
#include <algorithm>
#include <ctime>
#include <map>
#include <sstream>

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


RobotBase::RobotBase(ModelPosition* pos):
  rng(static_cast<unsigned int>(std::time(0))),
  MAXSPEED(800),
  ctrlString("Stop"),
  smoothProximity(0.01),
  bumped(0)
{
    std::cout << "RobotBase constructor for " << pos->TokenStr() << std::endl;
    RobotManager &rm = RobotManager::getInstance();
    rm.registerRobot(pos->TokenStr(), this);
    int i = 0;
    this->pos = pos;
    this->pos->AddCallback(Model::CB_UPDATE, (model_callback_t)this->PositionUpdate, this);
    this->pos->Subscribe();

    this->name = strdup(this->pos->Token());

    for (i = 0;i < NUM_IRS;i++)
    {
      this->proximity[i] = 0;
      this->offsetProximity[i] = 0;
    }

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

void RobotBase::SetCtrlString(std::string ctrlString)
{
  this->ctrlString = ctrlString;
  this->SetSpeed(0.0, 0.0);
//  this->LeftWheelVelocity = 0.0;
//  this->RightWheelVelocity = 0.0;
  this->bumped = 0;
  for (int i = 0;i < NUM_IRS;i++)
  {
    this->proximity[i] = 0;
    this->offsetProximity[i] = 0;
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
        
        // mimic real robots filter etc.
//        if(robot->proximity[i] < 100)
//        {
//          robot->offsetProximity[i] = (1.0 - robot->smoothProximity)*robot->offsetProximity[i] + robot->smoothProximity*robot->proximity[i];
//          robot->proximity[i] -= robot->offsetProximity[i];
          if(robot->proximity[i] < 20)
            robot->proximity[i] = 0;
//        }
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
      commands.push_back(robot->ctrlString.substr(pos));
      break;
    } else
    {
      commands.push_back(robot->ctrlString.substr(pos, nextpos-pos));
      pos = nextpos+1;
    }
  }

  robot->LeftWheelVelocity = 0;
  robot->RightWheelVelocity = 0;
  for(std::vector<std::string>::iterator command = commands.begin(); command != commands.end(); command++)
  {
    int delim = command->find_first_of(" ");
    std::string action = command->substr(0, delim);
    if(action.compare("GoStraight") == 0)
    {
      std::stringstream ss(command->substr(delim+1));
      float speed;
      ss >> speed;
      robot->GoStraight(speed);
    } else if(action.compare("TurnLeft") == 0)
    {
      std::stringstream ss(command->substr(delim+1));
      float speed;
      ss >> speed;
      robot->TurnLeft(speed);
    } else if(action.compare("TurnRight") == 0)
    {
      std::stringstream ss(command->substr(delim+1));
      float speed;
      ss >> speed;
      robot->TurnRight(speed);
    } else if(action.compare("Stop") == 0)
    {
      robot->Stop();
    } else if(action.compare("Follow") == 0)
    {
      int delim2 = command->find_first_of(" ", delim+1);
      std::string target = command->substr(delim+1, delim2-(delim+1));
      std::stringstream ss(command->substr(delim2+1));
      float dist;
      ss >> dist;
      robot->Follow(target, dist);
    } else if(action.compare("MoveTo") == 0)
    {
      std::stringstream ss(command->substr(delim+1));
      float x, y;
      ss >> x;
      ss >> y;
      robot->MoveTo(x, y);
    } else if(action.compare("Avoidance") == 0)
    {
      robot->Avoidance();
    } else if(action.compare("CalibrateIR") == 0)
    {
      robot->SetCtrlString("Stop");
    } else if(action.compare("SetpSpeed") == 0)
    {
      std::stringstream ss(command->substr(delim+1));
      float x, y;
      ss >> x;
      ss >> y;
      robot->LeftWheelVelocity = x;
      robot->RightWheelVelocity = y;
    } else if(action.compare("PrintProximityValues") == 0)
    {
      // no method to do this, just pass
    } else if(action.compare("Flock") == 0)
    {
      std::stringstream ss(command->substr(delim+1));
      std::vector<std::string> tokens;
      std::copy(std::istream_iterator<std::string>(ss), std::istream_iterator<std::string>(), std::back_inserter<std::vector<std::string> >(tokens));
      std::stringstream ss2(tokens[0]);
      float dist;
      ss2 >> dist;
      tokens.erase(tokens.begin());
      std::vector<std::string>::iterator self = std::find(tokens.begin(), tokens.end(), std::string(robot->name));
      if(self != tokens.end())
        tokens.erase(self);
      robot->Flock(dist, tokens);
    }
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
  RightWheelVelocity += speed*MAXSPEED;
  LeftWheelVelocity += speed*MAXSPEED;
}

void RobotBase::TurnLeft(float speed)
{
  if(speed > 1.0)
    speed = 1.0;
  if(speed < -1.0)
    speed = -1.0;
  RightWheelVelocity += speed*MAXSPEED;
  LeftWheelVelocity -= speed*MAXSPEED;
}

void RobotBase::TurnRight(float speed)
{
  if(speed > 1.0)
    speed = 1.0;
  if(speed < -1.0)
    speed = -1.0;
  RightWheelVelocity -= speed*MAXSPEED;
  LeftWheelVelocity += speed*MAXSPEED;
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
        this->LeftWheelVelocity += avoid_weightleft[i] * (proximity[i]);
        this->RightWheelVelocity += avoid_weightright[i] * (proximity[i]);
    }
}

void RobotBase::Flock(float dist, std::vector<std::string> &members)
{
  float alpha = 5e0/2;

  float fx = 0.0;
  float fy = 0.0;
  
  float myx = this->pos->est_pose.x;
  float myy = this->pos->est_pose.y;
  float mya = this->pos->est_pose.a;
 
  // map for sorting, I was to lazy for the sorting class
  std::map<float, std::string> candidates;
  for(std::vector<std::string>::iterator other = members.begin(); other != members.end(); other++)
  {
    float otherx = this->myModel->GetWorld()->GetModel(*other)->GetGlobalPose().x;
    float othery = this->myModel->GetWorld()->GetModel(*other)->GetGlobalPose().y;
    float dx = otherx - myx;
    float dy = othery - myy;
    float d = sqrt(dx*dx + dy*dy);
    candidates[d] = *other;
  }
  
  // get the closest robots
  std::vector<std::string> others;
  for(std::map<float, std::string>::iterator it = candidates.begin(); it != candidates.end(); it++)
  {
    if(others.size() < 2)
      others.push_back(it->second);
  }

  // calculate force
  for(std::vector<std::string>::iterator other = others.begin(); other != others.end(); other++)
  {
    float otherx = this->myModel->GetWorld()->GetModel(*other)->GetGlobalPose().x;
    float othery = this->myModel->GetWorld()->GetModel(*other)->GetGlobalPose().y;

    float dx = otherx - myx;
    float dy = othery - myy;
  
    float d = sqrt(dx*dx + dy*dy);
    
    fx += -alpha * dx/d * (dist - d);
    fy += -alpha * dy/d * (dist - d);
  }


  // rotate the force vector to the robot frame to get a the parts parallel and
  // perpendicular to it
  float fxa = cos(-mya) * fx - sin(-mya) * fy;
  float fya = sin(-mya) * fx + cos(-mya) * fy;

  std::cout << this->name << ": ";
  for(std::vector<std::string>::iterator other = others.begin(); other != others.end(); other++)
    std::cout << *other << " ";
  std::cout << mya << " " << fx << " " << fy << " " << fxa << " " << fya << std::endl;

  this->LeftWheelVelocity  += (fya + fxa) * this->MAXSPEED;
  this->RightWheelVelocity += (-fya + fxa) * this->MAXSPEED;
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
