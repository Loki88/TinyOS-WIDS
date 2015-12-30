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

	bool startPeriodic = FALSE;

	norace uint8_t reset = 0;

	wids_state_trace_t *alarmTrace;

	void updateTraces(wids_observable_t obs) {

		uint8_t size = call Traces.size(), i = 0;
		// wids_observable_t obs = Observables.dequeue();
		alarmTrace = NULL;

		while( i < size ) {
			linked_list_t *newStates;

			wids_state_trace_t *tmp = call Traces.dequeue();
			call HashMap.remove(tmp->state->id); 

			// manage return and update, remove or insert traces into the priority queue
			newStates = call ThreatModel.getObservedStates( tmp->state, obs );

			while( newStates != NULL ) {
				// the init trace has been removed, but we can have arrived to another state
				wids_state_trace_t *traceTmp = call HashMap.get(((wids_state_t*)newStates->element)->id);

				if( traceTmp != NULL ) { // the state is yet in a Queue

					traceTmp->observation_count = 0; // reset the observation count and then update the alarm score
					if (traceTmp->alarm_value > tmp->alarm_value){
						traceTmp->alarm_value += ((wids_state_t*)newStates->element)->alarm_level;
					} else {
						traceTmp->alarm_value = tmp->alarm_value + ((wids_state_t*)newStates->element)->alarm_level;
					}
					
				} else if( ((wids_state_t*)newStates->element)->id == tmp->state->id ) { 	// the state is not in another trace but
																// the new state is the same of the old trace
					traceTmp = tmp;
					traceTmp->observation_count += 1;
					call Traces.enqueue(traceTmp);
					call HashMap.insert(traceTmp, traceTmp->state->id);

				} else {	// there is a new state so it is needed to create a new trace
					traceTmp = malloc(sizeof(wids_state_trace_t));
					traceTmp->state = ((wids_state_t*)newStates->element);
					traceTmp->observation_count = 0;
					traceTmp->alarm_value = tmp->alarm_value + ((wids_state_t*)newStates->element)->alarm_level;
					call Traces.enqueue(traceTmp);
					call HashMap.insert(traceTmp, traceTmp->state->id);
				}

				if( alarmTrace == NULL || traceTmp->alarm_value > alarmTrace->alarm_value ){
					alarmTrace = traceTmp; // update the trace with max alarm score
				}

				newStates = newStates->next;

			}

			i += 1;
		}

		if( alarmTrace != NULL ){ // we have computed the more risky attack until the next relevation
			// TODO: signal the alarm to the alarming component
			signal AlarmGeneration.attackDone(alarmTrace->state->attack, alarmTrace->alarm_value);	
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

	event void Notify.notify( wids_observable_t observable ){
		call Observables.enqueue(observable);
		post parseObservable();
		
	}

}