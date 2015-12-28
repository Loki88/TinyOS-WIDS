/*
 *
 *    Copyright © 2015 Lorenzo Di Giuseppe <lorenzo.digiuseppe88@gmail.com>.
 *    All Rights Reserved.
 *
 *    Redistribution and use in source and binary forms, with or without modification, 
 *    are permitted provided that the following conditions are met:
 *
 *    1. Redistributions of source code must retain the above copyright notice, this 
 *    	list of conditions and the following disclaimer.
 *
 *    2. Redistributions in binary form must reproduce the above copyright notice, 
 *    	this list of conditions and the following disclaimer in the documentation 
 *    	and/or other materials provided with the distribution.
 *
 *    3. The name of the author may not be used to endorse or promote products derived 
 *    	from this software without specific prior written permission.
 *
 *    THIS SOFTWARE IS PROVIDED BY Lorenzo Di Giuseppe "AS IS" AND ANY EXPRESS OR IMPLIED 
 *    WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY 
 *    AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE AUTHOR BE 
 *    LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL 
 *    DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *    LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY 
 *    THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
 *    NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN 
 *    IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

 
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