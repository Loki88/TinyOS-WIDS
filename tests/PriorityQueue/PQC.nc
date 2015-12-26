
#include "THREATMODEL.h"
#include "WIDS.h"
#include "printf.h"

module PQC @safe()
{
  uses interface Timer<TMilli> as Timer;
  uses interface Leds;
  uses interface Boot;
  uses interface WPriorityQueue<wids_state_trace_t, uint8_t>;
  uses interface Init;
}
implementation
{
	bool s = FALSE;
	uint8_t score = 1;

	event void Boot.booted()
	{
		call Init.init();
		call Timer.startPeriodic( 3000 );
	}

	task void insert(){
		wids_state_trace_t *trace = malloc(sizeof(wids_state_trace_t)+sizeof(wids_state_t));
		if ( s==TRUE ){
			s = FALSE;
			((wids_state_t*)trace)->attack = DECEPTIVE_JAMMING;
			call Leds.led1On();
			call Leds.led2Off();
		}
		else {
			s = TRUE;
			((wids_state_t*)trace)->attack = DECEPTIVE_JAMMING;
			call Leds.led2On();
			call Leds.led1Off();
		}
		trace->alarm_value = score;

		call WPriorityQueue.insert( trace, score );
		score <<= 1;

		
		trace = call WPriorityQueue.findMax();

		printf("TRACE ALARM: %d, SCORE %d\r\n", ((wids_state_t*)trace)->attack, trace->alarm_value);
		printfflush();
	}

	event void Timer.fired()
	{
		call Leds.led0Toggle();
		post insert();
	}

}