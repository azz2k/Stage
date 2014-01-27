#include "epuck.hh"

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




// Stage calls this when the model starts up
extern "C" int Init( Model* mod )
{
    Robot *robot = new Robot((ModelPosition*)mod);
    return 0; //ok
}


Robot::Robot(ModelPosition* pos):
        bumped(0)
{
    int i = 0;
    this->pos = pos;
    this->pos->AddCallback(Model::CB_UPDATE, (model_callback_t)this->PositionUpdate, this);
    this->pos->Subscribe();

    this->name = strdup(this->pos->Token());


    this->current_state = FLOCKING;
    this->last_state = FLOCKING;

    for (i = 0;i < NUM_IRS;i++)
        this->proximity[i] = 0;

    this->ir = (ModelRanger *) pos->GetUnusedModelOfType("ranger"); // find the names in libstage/typetable.cc
    if (this->ir)
    {
        this->ir->AddCallback(Model::CB_UPDATE,(model_callback_t)this->IRUpdate, this);
        this->ir->Subscribe();
    }
    

}

Robot::~Robot()
{
  //  printf("Destroy robot: %s\n", this->pos->Token());
    if (name)
    {
        free(name);
        name = NULL;
    }
}

// inspect the laser data and decide what to do
int Robot::IRUpdate( Model* mod, Robot *robot )
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

int Robot::PositionUpdate( Model* mod, Robot *robot)
{

    switch (robot->current_state)
    {
        case FLOCKING:
            robot->Flocking();
            break;
        default:
            break;
    }

    return 0; // run again
}

void Robot::SetSpeed(int lspeed, int rspeed)
{
    if (lspeed < -1000)
        lspeed = -1000;
    if (lspeed > 1000)
        lspeed = 1000;
    if (rspeed < -1000)
        rspeed = -1000;
    if (rspeed > 1000)
        rspeed = 1000;

    double x, a;

    x = (lspeed + rspeed) * M_PI * WHEEL_DIA / (2 * 1000.0);
    a = (rspeed - lspeed) * M_PI * WHEEL_DIA * 2 / (WHEEL_SEP * 1000.0);

    this->pos->SetSpeed(x, 0, a);
}

void Robot::Avoidance()
{
    int leftwheel, rightwheel;

    leftwheel = 200;
    rightwheel = 200;

    for (int i = 0; i < NUM_IRS; i++)
    {
        leftwheel += avoid_weightleft[i] * (proximity[i]);
        rightwheel += avoid_weightright[i] * (proximity[i]);
    }

    if (this->pos->Stalled())
    {
        leftwheel = -500;
        rightwheel = -500;

        for (int i = 0; i < NUM_IRS; i++)
        {
            leftwheel += avoid_weightleft[i] * (proximity[i] >> 1);
            rightwheel += avoid_weightright[i] * (proximity[i] >> 1);
        }
    }
    SetSpeed(leftwheel, rightwheel);
}

void Robot::Flocking()
{
    this->Avoidance();
}

//below defined functions for debugging
void Robot::PrintProximitySensor()
{
    printf("%ld:%s IR --[", this->pos->GetWorld()->GetUpdateCount(), this->pos->Token());
    for (int i = 0;i < NUM_IRS;i++)
        printf(" %d", this->proximity[i]);
    printf("]\n");
}

void Robot::PrintState()
{
    printf("%ld:%s in state [%s] %d\n", this->pos->GetWorld()->GetUpdateCount(),
           this->pos->Token(), state_name[this->current_state], (unsigned char)~(1 << 3));
}
