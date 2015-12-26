
#include "THREATMODEL.h"

#define NEW_PRINTF_SEMANTICS

#include "printf.h"
configuration PQAppC
{
}
implementation
{
	#define HASH_NUM_EL 10

	components MainC, PQC, LedsC;
	components new TimerMilliC() as Timer;
	components new FibonacciHeapC(wids_state_trace_t) as FH;
	components new HashMapC(uint16_t, heap_node_t, HASH_NUM_EL), new SimpleHashC(HASH_NUM_EL);
	components SerialPrintfC;

	PQC -> MainC.Boot;

	HashMapC.Hash -> SimpleHashC;
	FH.HashMapInit -> HashMapC;
	FH.HashMap -> HashMapC;

	PQC.Init -> FH.Init;
	PQC.WPriorityQueue -> FH;
  	PQC.Timer -> Timer;
 	PQC.Leds -> LedsC;
}