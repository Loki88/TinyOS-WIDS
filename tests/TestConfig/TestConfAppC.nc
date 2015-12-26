


#define NEW_PRINTF_SEMANTICS

#include "THREATMODEL.h"
#include "printf.h"
configuration TestConfAppC
{
}
implementation
{
	components MainC, TestConfC, LedsC;
	components PrintfC, SerialStartC;

	components WIDSThreatModelC as Model;

	TestConfC -> MainC.Boot;
	TestConfC.ThreatModel -> Model;
	TestConfC.ModelConfig -> Model;

	Model.Boot -> MainC.Boot;


 	TestConfC.Leds -> LedsC;

}