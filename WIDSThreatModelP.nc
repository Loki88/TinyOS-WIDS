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
#include "printf.h"

module WIDSThreatModelP {

	provides interface ThreatModel;
	provides interface ModelConfig;
	provides interface Init;

	uses interface Queue<wids_state_t*>;
	uses interface HashMap<uint8_t, wids_state_t>;
	uses interface Init as HashMapInit;

} implementation {

	norace uint8_t m_id;

	command error_t Init.init(){
		call HashMapInit.init();
		if(call ModelConfig.createState(0, NO_ATTACK, 0) == SUCCESS){
			wids_state_t *reset = call ThreatModel.getResetState();
			reset -> next = NULL;
			reset->observables = NULL;
			reset->transitions = NULL;
			return SUCCESS;
		} else {
			return FAIL;
		}
		
	}

	command error_t ModelConfig.createState(uint8_t id, wids_attack_t att, uint8_t alarm){

		wids_state_t *state = malloc(sizeof(wids_state_t));
		wids_state_t *tmp = call ThreatModel.getResetState();

		state->id = id;
		state->attack = att;
		state->alarm_level = alarm;
		state->observables = NULL;
		state->transitions = NULL;
		state->next = tmp->next;

		tmp->next = state;

		if(call HashMap.insert(state, id) != SUCCESS){
			free( state );
			return FAIL;
		}
		return SUCCESS;
	}

	command error_t ModelConfig.addTransition( uint8_t idFrom, uint8_t idTo ){
		wids_state_t *from = call HashMap.get(idFrom);
		wids_state_t *to = call HashMap.get(idTo);

		if( from != NULL && to != NULL ){
			wids_state_transition_t *transition = malloc(sizeof(wids_state_transition_t));
			transition->state = to;
			transition->next = from->transitions;
			from->transitions = transition;	
			return SUCCESS;			
		} else {
			return FAIL;
		}	
	}

	command error_t ModelConfig.addObservable( uint8_t stateId, wids_observable_t obs ){
		wids_state_t *state = call HashMap.get( stateId );

		if( state != NULL ) {
			wids_obs_list_t *obsEntry = malloc(sizeof(wids_obs_list_t));
			obsEntry->obs = obs;
			obsEntry->next = state->observables;
			state->observables = obsEntry;
			return SUCCESS;
		} else {
			return FAIL;
		}
	}

	void visitSubtree( wids_state_t *curState ){
		wids_state_transition_t *tmpPrev = NULL;
		wids_state_transition_t *tmp = curState->transitions;
		while( tmp != NULL ) {
			if( tmp->state->flag == FALSE ){
				call Queue.enqueue(tmp->state);
				tmp->state->flag = TRUE;
			}
			if( tmp->state->id == m_id ){ // questa transition va rimossa
				if( tmpPrev == NULL ){
					curState->transitions = tmp->next;
					free(tmp);
					tmp = curState->transitions;
				} else {
					tmpPrev->next = tmp->next;
					free(tmp);
					tmp = tmpPrev->next;
				}
			} else {
				tmpPrev = tmp;
				tmp = tmp->next;
			}
			
		}
	}

	command error_t ModelConfig.removeState( uint8_t stateId ) {
		wids_state_t *reset = call ThreatModel.getResetState();
		wids_state_t *nextState = reset->next;
		m_id = stateId;

		while( nextState != NULL ){
			nextState->flag = FALSE;
			nextState = nextState->next;
		}

		call Queue.enqueue(reset);
		while( call Queue.empty() == FALSE){
			visitSubtree( call Queue.dequeue() );
		}

		reset = call HashMap.get(stateId);
		call HashMap.remove(stateId);
		free(reset);
		return SUCCESS;
	}

	async command wids_state_t* ThreatModel.getResetState(){
		return call HashMap.get(0);
	}

	async command wids_state_t* ThreatModel.getState( uint8_t id ){
		return call HashMap.get( id );
	}

	async command linked_list_t* ThreatModel.getNextStates( wids_state_t *state ) {

		// wids_state_transition_t *neighbour = state->transitions; // visit all the states near the current one
		// linked_list_t *observedStates = NULL, *tmp = NULL;
		// wids_obs_list_t *observables = NULL;

		// while( neighbour != NULL ) {
		// 	observables = neighbour->state->observables;

		// 	while( observables != NULL ) {
		// 		if( observables->obs == observable ) {
		// 			// printf("FOUND state id %d for observable %s\n", neighbour->state->id, printObservable(observables->obs));

		// 			tmp = malloc(sizeof(linked_list_t));
		// 			tmp -> next = observedStates;
		// 			observedStates = tmp;
					
		// 			tmp->element = neighbour->state;
		// 			break; // don't continue this while since the state has yet been added
		// 		}

		// 		observables = observables->next;
		// 	}
		// 	neighbour = neighbour -> next;
		// }

		// return observedStates;
		return (linked_list_t*)state->transitions;
	}

	async command wids_obs_list_t* ThreatModel.getObservables( wids_state_t *state ){
		return state->observables;
	}

	command error_t ModelConfig.sync(){
		return SUCCESS;
	}
}