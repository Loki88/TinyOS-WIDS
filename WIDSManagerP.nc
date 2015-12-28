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

#include "WIDS.h"
#include "THREATMODEL.h"

module WIDSManagerP {

	uses interface ThreatModel;

	uses interface Notify<wids_observable_t> as Detection[uint8_t id];

	uses interface Queue<wids_state_trace_t*> as Traces[uint8_t id];

	// uses interface Queue<wids_observable_t> as ObsQueue[uint8_t id];

	uses interface HashMap<uint8_t, wids_state_trace_t*> as HashMap;

} implementation {

	bool startPeriodic = FALSE;

	norace uint8_t reset = 0;

	norace wids_observable_t m_observable;

	norace uint8_t m_id;

	task void updateTraces( ) {

		uint8_t size = call Traces[m_id].size(), i = 0;
		wids_state_trace_t *alarmTrace = NULL;

		while( i < size ) {
			wids_state_trace_t *tmp = call Traces[m_id].dequeue();
			call HashMap.remove(tmp->state->id);

			// manage return and update, remove or insert traces into the priority queue
			linked_list_t* newStates = call ThreatModel.getObservedStates( tmp->state, obs );

			while( newStates != NULL ) {

				// the init trace has been removed, but we can have arrived to another state
				wids_state_trace_t *traceTmp = call HashMap.get(newStates->id);

				if( hTmp != NULL ) { // the state is yet in a Queue

					hTmp->observation_count = 0; // reset the observation count and then update the alarm score
					if (hTmp->alarm_value > tmp->alarm_value){
						hTmp->alarm_value += newStates->alarm_level;
					} else {
						hTmp->alarm_value = tmp->alarm_value + newStates->alarm_level;
					}
					
				} else if( newStates->id == tmp->state->id ) { 	// the state is not in another Queue but
																// the new state is the same
					hTmp = tmp;
					hTmp->observation_count += 1;
					call Traces[m_id].enqueue(hTmp);
					call HashMap.insert(hTmp, hTmp->state->id);

				} else {	// there is a new state so it is needed to create a new trace
					hTmp = malloc(sizeof(wids_state_trace_t));
					hTmp->state = newStates;
					hTmp->observation_count = 0;
					hTmp->alarm_value = tmp->alarm_value + newStates->alarm_level;
					call Traces[m_id].enqueue(hTmp);
					call HashMap.insert(hTmp, hTmp->state->id);
				}

				if( alarmTrace == NULL || hTmp->alarm_value > alarmTrace->alarm_value ){
					alarmTrace = hTmp; // update the trace with max alarm score
				}

				newStates = newStates->next;

			}

			i += 1;
		}

		if( alarmTrace != NULL ){ // we have computed the more risky attack until the next relevation
			// TODO: signal the alarm to the alarming component
			
		}
	
	}

	task void resetTraces(){
		while( call Traces[m_id].size() > 0 ){
			wids_state_trace_t *tmp = call Traces[m_id].dequeue();
			call HashMap.remove(tmp->state->id);
		}
	}

	async event void Detection.notify[uint8_t id]( wids_observable_t observable ){
		m_id = id;
		m_observable = observable;
		if( observable == NONE ) {
			reset += 1;
			if ( reset >= RESET_COUNT ){
				post resetTraces();
			}
		} else {
			reset = 0;
			post updateTraces();
		}
	}

}