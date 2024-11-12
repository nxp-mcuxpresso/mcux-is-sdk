#include "KeynetikPedometer.h"
#include "KPedometerStatsCore.h"
#include "KPedometerActivity.h"

using namespace Keynetik;

unsigned int keynetikStepCount=0;
unsigned int keynetikDistance=0;
unsigned int keynetikCalories=0;
unsigned short keynetikSpeed=0;

unsigned short g_oneG;

static PedometerStatsCore* g_stats=0;
static PedometerActivity*  g_act=0;

KeynetikActivityLevel keynetikActivityLevel=KeynetikActivityUnknown;

extern "C"
void
KeynetikInitialize(unsigned short p_oneG,
                   unsigned short p_frequencyHz,
                   KeynetikConfig* config)
{
	static PedometerStatsCore stats;
	static PedometerActivity  act;
	g_stats=&stats;
	g_act=&act;
    g_oneG=(unsigned short)p_oneG;
    AccelerometerReading::Set1G(Fixed(p_oneG));
    AccelerometerReading::SetFrequency((unsigned short)p_frequencyHz);

    g_stats->SetGender(config->bits.male);
    g_stats->SetHeight(PedometerStride::HeightCm(config->height));
    g_stats->SetWeight(PedometerStatsCore::WeightKg(config->weight));
    g_stats->SetStride(PedometerStride::StrideCm(config->steplength));
    g_stats->SetStepDelay((unsigned char)config->filtersteps);
    g_stats->SetDelayTime(config->bits.filtertime);
    if (config->speedperiod > 5)
    {
        g_act->m_InstaSpeedPeriod = 5;
    }
    else if (config->speedperiod < 1)
    {
        g_act->m_InstaSpeedPeriod = 1;
    }
    else
    {
        g_act->m_InstaSpeedPeriod = config->speedperiod;
    }
    if (config->stepthreshold != 0)
    {
    	g_stats->GetPedometer().SetThreshold(config->stepthreshold);
    }
    KeynetikReset();
}

extern "C"
unsigned int
KeynetikHandleIncomingEvent(int x, int y, int z)
{
	unsigned int ret=0;

	PedometerStatsCore::Event ev=g_stats->HandleIncomingEvent((short)(x * 1000 / g_oneG), (short)(y * 1000 / g_oneG), short(z * 1000 / g_oneG));

    PedometerStatsCore::DistanceCm distance=g_stats->GetDistance();
    PedometerActivity::ActivityLevel activity;
	if (ev == PedometerStatsCore::Step)
	{
        static PedometerStatsCore::DistanceCm lastDistance=0;
		keynetikStepCount   = g_stats->GetStepCount();
        keynetikDistance    = (unsigned int)distance/100;
		keynetikCalories    = g_stats->GetCalories() / PedometerStatsCore::CaloriesScale;
        activity = g_act->HandleStep(g_stats->GetPedometer().GetReadingCount(),
                                     PedometerActivity::StepDistanceCm(distance - lastDistance));
        lastDistance = distance;
		ret |= KEYNETIK_STEP;
	}
    else
    {
        activity=g_act->HandleReading(g_stats->GetPedometer().GetReadingCount());
    }
    keynetikSpeed = g_act->GetSpeed();

	KeynetikActivityLevel newLevel;
    switch (activity)
    {
        case PedometerActivity::Rest:       newLevel=KeynetikActivityRest;     break;
        case PedometerActivity::Walking:    newLevel=KeynetikActivityWalking;  break;
        case PedometerActivity::Jogging:    newLevel=KeynetikActivityJogging;  break;
        case PedometerActivity::Running:    newLevel=KeynetikActivityRunning;  break;
        case PedometerActivity::NoValue:    newLevel=keynetikActivityLevel;    break;
        default:                            newLevel=KeynetikActivityUnknown;  break;
    }
    if (newLevel != keynetikActivityLevel)
    {
        keynetikActivityLevel=newLevel;
        ret |= KEYNETIK_ACTIVITYCHANGED;
    }

	return ret;
}

extern "C"
void
KeynetikReset()
{
    g_stats->Reset();
    g_act->Reset();
    keynetikStepCount=0;
    keynetikDistance=0;
    keynetikSpeed=0;
    keynetikCalories=0;
    keynetikActivityLevel=KeynetikActivityUnknown;
}

extern "C"
void
KeynetikTerminate()
{
}
