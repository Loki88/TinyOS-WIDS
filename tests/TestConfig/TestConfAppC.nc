	


#define NEW_PRINTF_SEMANTICS

#include "WIDS.h"
#include "printf.h"
configuration TestConfAppC
{
}
implementation
{
	components MainC, TestConfC, LedsC, new BusyWaitCounterC(TMilli, uint16_t) as ConfWait, 
		new BusyWaitCounterC(TMilli, uint16_t) as AppWait, CounterMilli16C, new TimerMilliC();
	
	components SerialPrintfC;

	components WIDSThreatModelC as Model;

	TestConfC -> MainC.Boot;
	TestConfC.ThreatModel -> Model;
	TestConfC.ModelConfig -> Model;
	TestConfC.Timer -> TimerMilliC;

	Model.Boot -> MainC.Boot;
	Model.Leds -> LedsC;
	Model.BusyWait -> ConfWait;

	ConfWait.Counter -> CounterMilli16C;
	AppWait.Counter -> CounterMilli16C;

 	TestConfC.Leds -> LedsC;
 	TestConfC.BusyWait -> AppWait;

}