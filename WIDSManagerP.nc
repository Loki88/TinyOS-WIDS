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

#include "Wids.h"
#include "ThreatModel.h"

module WIDSManagerP {

	provides interface AlarmGeneration;

	uses interface ThreatModel;
	uses interface Notify<wids_observable_t>;
	uses interface Queue<wids_state_trace_t*> as Traces;
	uses interface Queue<wids_observable_t> as Observables;

	// this keep trace of all active traces
	uses interface HashMap<uint8_t, wids_state_trace_t> as HashMap;

} implementation {

	enum{
		MAX_UINT8 = 255,
		RESET_ID = 0,
	};

	norace uint8_t reset = 0;

	inline void updateScore(wids_state_trace_t *t, uint8_t al){
		uint16_t tmp = (uint16_t)t->alarm_value + (uint16_t)al;
		if(tmp > 255)
			tmp = 255;
		t->alarm_value = tmp;
	}

	inline void initTrace(wids_state_trace_t *trace){
		trace->state = NULL;
		trace->observation_count = 0;
		trace->alarm_value = 0;
	}

	bool containsObservable(wids_state_t *s, wids_observable_t o){
		wids_obs_list_t *obsList = call ThreatModel.getObservables(s);
		while(obsList != NULL){
			if(obsList->obs == o)
				return TRUE;
			obsList = obsList->next;
		}
		return FALSE;
	}

	bool updateTrace(wids_state_trace_t *tr, wids_observable_t obs){
		linked_list_t *traceList = call ThreatModel.getNextStates(tr->state);
		bool r = TRUE;
		uint8_t score = tr->alarm_value;

		printf("Updating trace relative to attack %s in state %d for %s\n", printfAttack(tr->state->attack), 
			tr->state->id, printObservable(obs));

		if(containsObservable(tr->state, obs) == TRUE) { // state tr->state is still possible
			tr->observation_count = 0;
			signal AlarmGeneration.attackFound(tr->state->attack, tr->alarm_value);
		} else {
			tr->observation_count += 1; // reset count
			if(tr->observation_count > RESET_COUNT){
				r = FALSE;
			} else {
				signal AlarmGeneration.attackFound(tr->state->attack, tr->alarm_value);
			}
		}

		
		
		// The new states are now considered

		while(traceList != NULL){
			printf("Iteration on state %d\n", ((wids_state_t*)traceList->element)->id);
			// See if the state is possible for observable "obs"
			if(containsObservable(((wids_state_t*)traceList->element), obs) == TRUE){
				// Look for the same state in another trace, in that case we can merge
				wids_state_trace_t *trace = call HashMap.get(((wids_state_t*)traceList->element)->id);

				// If the trace exists then don't create a new trace, but update the existing one
				if(trace != NULL){
					trace->observation_count = 0; // set to zero the reset counter
					if(trace->alarm_value > score){
						updateScore(trace, trace->state->alarm_level);
					} else {
						updateScore(trace, score + trace->state->alarm_level - trace->alarm_value);
					}
				}
				else {	// there is not a trace for this state so it is needed to create it
					trace = malloc(sizeof(wids_state_trace_t));
					trace->state = ((wids_state_t*)traceList->element);
					trace->observation_count = 0;
					trace->alarm_value = score + ((wids_state_t*)traceList->element)->alarm_level;
					call Traces.enqueue(trace);
					call HashMap.insert(trace, trace->state->id);
				}

				signal AlarmGeneration.attackFound(trace->state->attack, trace->alarm_value);
			}

			traceList = traceList->next;
		}

		return r;
	}

	void updateTraces(wids_observable_t obs) {
		uint8_t i=0, size;
		wids_state_trace_t *t;

		if(call HashMap.get(RESET_ID) == NULL){
			t = malloc(sizeof(wids_state_trace_t));
			initTrace(t);
			t->state = call ThreatModel.getResetState();		
			call Traces.enqueue(t);
		}

		size=call Traces.size();
		while( i < size ){
			bool res;
			t = call Traces.dequeue();
			res = updateTrace(t, obs);
			if( t->state->id != RESET_ID){
				if(res == TRUE)
					call Traces.enqueue(t);
				else {
					call HashMap.remove(t->state->id);
					free(t);
				}
			} else {
				free(t);
			}
			i += 1;
		}
	}

	inline void resetTraces(){
		while( call Traces.size() > 0 ){
			wids_state_trace_t *tmp = call Traces.dequeue();
			call HashMap.remove(tmp->state->id);
			free(tmp);
		}
	}

	task void parseObservable(){
		wids_observable_t observable = call Observables.dequeue();
		if( observable == OBS_NONE ) {
			reset += 1;
			if ( reset >= RESET_COUNT ){
				resetTraces();
			}
		} else {
			reset = 0;
			updateTraces(observable);
		}
	}

	task void update(){
		while( call Observables.size() > 0 ){
			updateTraces(call Observables.dequeue());
		}
	}

	event void Notify.notify( wids_observable_t observable ){
		printf("Notify.notify(%s)\n", printObservable(observable));
		if( call Observables.size() == 0 ) {
		// if( observable == OBS_NONE ) {
			reset += 1;
			if ( reset >= RESET_COUNT ){
				printf("RESET\n");
				reset = 0;
				resetTraces();
			}
		} else {
			reset = 0;
			post update();
		}
		
	}

}