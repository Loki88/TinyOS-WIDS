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
	provides interface Boot as WIDSBoot;

	uses interface Boot;
	uses interface ThreatModel;
	uses interface ObservableNotify;
	uses interface AsyncQueue<wids_state_trace_t*> as Traces;
	uses interface AsyncQueue<wids_observable_t> as Observables;

	// this keep trace of all active traces
	uses interface HashMap<uint8_t, wids_state_trace_t> as HashMap;

} implementation {

	enum{
		MAX_UINT8 = 255,
		RESET_ID = 0,
	};

	bool m_booted = FALSE;

	norace uint8_t reset = 0;
	norace wids_state_t *resetState;

	event void Boot.booted(){
		printf("WIDSManagerP -> BOOT\n");
		printfflush();
		signal WIDSBoot.booted();
	}

	inline void updateScore(wids_state_trace_t *t, uint8_t al){
		uint16_t tmp = (uint16_t)t->score + (uint16_t)al;
		if(tmp > 255)
			tmp = 255;
		t->score = tmp;
	}

	inline void initTrace(wids_state_trace_t *trace){
		trace->state = NULL;
		trace->count = 0;
		trace->score = 0;
		trace->aging = 0;
	}

	bool containsObservable(wids_state_t *s, wids_observable_t o){
		wids_obs_list_t *obsList = call ThreatModel.getObservables(s);
		while(obsList != NULL){
			if(obsList->obs == o){
				return TRUE;
			}
			obsList = obsList->next;
		}

		return FALSE;
	}

	norace uint8_t traceSize = 0;

	inline void enqueueTrace(wids_state_trace_t *t){
		if(call Traces.size() == call Traces.maxSize()){
			wids_state_trace_t *tmp = call Traces.dequeue();
			// TODO: sarebbe necessario valutare che la traccia non abbia basso rischio
			traceSize -= 1;
			call Traces.enqueue(t);
		}
	}

	void findNewTraces(wids_state_trace_t *tr, wids_observable_t obs){
		wids_state_transition_t *neighbors = call ThreatModel.getNextStates(tr->state);
		wids_obs_list_t *resetObservables = call ThreatModel.getResetObservables(tr->state);
		wids_state_trace_t *trace = NULL;

		while(resetObservables != NULL){ // check if the observable should reset this trace
			if(resetObservables->obs == obs){
				tr->count += 1;
				if(tr->count > tr->state->resetCount){
					tr->state = NULL;
					tr->score = 0;
					return; // the observable resets this trace
				}
				return; // an observable can be in a list only once and it is a reset obs
			}
			resetObservables = resetObservables->next;
		}

		// state tr->state is still a possible state
		if(tr->state->loop == TRUE && containsObservable(tr->state, obs) == TRUE) {
			tr->count = 0;
			tr->aging = 0;
			signal AlarmGeneration.traceLevelUpdate(tr->state->attack, tr->score);
		}
		
		// The new states are now considered
		while(neighbors != NULL){
			// Test if the state is possible for observable "obs"
			if(containsObservable(neighbors->state, obs) == TRUE){
				// Look for the same state in another trace, in that case we can merge them
				if(call HashMap.get(neighbors->state->id, &trace) == SUCCESS){ 
					// If a trace for this state exists then don't create a new one,
					// but update the existing one and its score
					trace->count = 0; // Set the reset counter to zero and update its score
					trace->aging = 0;
					// updateScore(trace, trace->state->alarm_level);
					signal AlarmGeneration.traceLevelUpdate(trace->state->attack, trace->score);
				}
				else {
					// there is not a trace for this state so it is needed to create it
					trace = malloc(sizeof(wids_state_trace_t));
					initTrace(trace);
					trace->state = neighbors->state;
					trace->score = trace->state->alarm_level;
					
					enqueueTrace(trace);
					
					call HashMap.insert(trace->state->id, trace);

					signal AlarmGeneration.newTraceFound(trace->state->attack, trace->score);
				}
			}
			neighbors = neighbors->next;
		}
	}


	void updateTraces(wids_observable_t obs) {
		uint8_t stateId;
		wids_state_trace_t *t = NULL;
		traceSize = call Traces.size();
		
		while( traceSize > 0 ){
			t = call Traces.dequeue();
			traceSize -= 1;

			stateId = t->state->id;

			t->aging += 1;

			findNewTraces(t, obs);

			if(t->state == NULL || t->aging > call ThreatModel.getMaxAging()){
				call HashMap.remove(stateId);
				free(t);
			} else {
				enqueueTrace(t);
			}

			
		}
		
		if( obs != OBS_NONE ){
			t = malloc(sizeof(wids_state_trace_t));
			initTrace(t);
			call ThreatModel.getResetState( &(t->state) );
			findNewTraces(t, obs);
		}
	}

	norace bool running = FALSE;

	async event void ObservableNotify.notify( wids_observable_t o ){
		// printf("ObservableNotify.notify( %s )\n", printObservable(o));

		atomic{
			updateTraces(o);
		}
	}

}