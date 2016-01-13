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

	enum{
		RESET_ID = 0,
		RESET_SCORE = 0,
	};

	norace uint8_t m_id;
	wids_state_t *resetState = NULL;
	uint8_t maxAging = 5;

	command error_t Init.init(){
		printf("WIDSThreatModelP -> INIT\n");
		call HashMapInit.init();

		if(call ModelConfig.createState(RESET_ID, NO_ATTACK, RESET_SCORE) == SUCCESS){
			call ThreatModel.getState(RESET_ID, &resetState);

			resetState->next = NULL;
			resetState->observables = NULL;
			resetState->transitions = NULL;

			return SUCCESS;
		} else {
			return FAIL;
		}
	}

	void initState(wids_state_t *state){
		state->id = 0;
		state->attack = NO_ATTACK;
		state->alarm_level = 0;
		state->observables = NULL;
		state->transitions = NULL;
		state->next = NULL;
		state->loop = FALSE;
		state->reset = NULL;
		state->resetCount = 1;
	}

	command error_t ModelConfig.createState(uint8_t id, wids_attack_t att, uint8_t alarm){

		wids_state_t *state = malloc(sizeof(wids_state_t));
		wids_state_t *tmp = resetState->next;
		initState(state);

		// printf("ModelConfig.createState(%d, %s, %d)\n", id, printfAttack(att), alarm);

		state->id = id;
		state->attack = att;
		state->alarm_level = alarm;
		resetState->next = state;
		state->next = tmp;

		if(call HashMap.insert( id, state ) != SUCCESS){
			free( state );
			return FAIL;
		}
		return SUCCESS;
	}

	command error_t ModelConfig.addTransition( uint8_t idFrom, uint8_t idTo ){
		wids_state_t *from = NULL, *to = NULL;
		// printf("ModelConfig.addTransition(%d, %d)\n", idFrom, idTo);
		if( call HashMap.get(idFrom, &from) == SUCCESS &&
					call HashMap.get(idTo, &to) == SUCCESS){
			wids_state_transition_t *transition = malloc(sizeof(wids_state_transition_t));
			// printf("Transition from state->id %d to state->id %d\n", from->id, to->id);
			transition->state = to;
			transition->next = from->transitions;
			from->transitions = transition;	
			return SUCCESS;			
		} else {
			return FAIL;
		}	
	}

	command error_t ModelConfig.addObservable( uint8_t stateId, wids_observable_t obs ){
		wids_state_t *state = NULL;
		// printf("ModelConfig.addObservable(%d, %s)\n", stateId, printObservable(obs));
		if( call HashMap.get( stateId, &state ) == SUCCESS ) {
			wids_obs_list_t *obsEntry = malloc(sizeof(wids_obs_list_t));
			obsEntry->obs = obs;
			obsEntry->next = state->observables;
			state->observables = obsEntry;
			// printf("Observable %s in state %d\n", printObservable(obsEntry->obs), state->id);
			return SUCCESS;
		} else {
			return FAIL;
		}
	}

	command error_t ModelConfig.addResetObservable( uint8_t stateId, wids_observable_t obs ){
		wids_state_t *state = NULL;

		if( call HashMap.get( stateId, &state ) == SUCCESS ) {
			wids_obs_list_t *obsEntry = malloc(sizeof(wids_obs_list_t));
			obsEntry->obs = obs;
			obsEntry->next = state->reset;
			state->reset = obsEntry;
			return SUCCESS;
		} else {
			return FAIL;
		}
	}

	command error_t ModelConfig.allowLoop( uint8_t stateId, bool loop ){
		wids_state_t *state = NULL;

		if( call HashMap.get( stateId, &state ) == SUCCESS ) {
			state->loop = loop;
			return SUCCESS;
		} else {
			return FAIL;
		}
	}

	async command error_t ModelConfig.setResetCount( uint8_t stateId, uint8_t resetCount ){
		wids_state_t *state = NULL;

		if( call HashMap.get( stateId, &state ) == SUCCESS ) {
			state->resetCount = resetCount;
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
		wids_state_t *reset = resetState;
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

		if(call HashMap.get(stateId, &reset) == SUCCESS){ // here reset contains the state to remove
			call HashMap.remove(stateId);
			free(reset);
			return SUCCESS;
		} else {
			return FAIL;
		}
	}

	async command void ThreatModel.getResetState(wids_state_t **s){
		*s = resetState;
	}

	async command void ThreatModel.getState( uint8_t id, wids_state_t **s ){
		call HashMap.get( id, s );
	}

	async command wids_state_transition_t* ThreatModel.getNextStates( wids_state_t *state ) {
		return state->transitions;
	}

	async command bool ThreatModel.isLoopAllowed( wids_state_t *state ) {
		return state->loop;
	}

	async command bool ThreatModel.getResetCount( wids_state_t *state ) {
		return state->resetCount;
	}

	async command wids_obs_list_t* ThreatModel.getObservables( wids_state_t *state ){
		return state->observables;
	}

	async command wids_obs_list_t* ThreatModel.getResetObservables( wids_state_t *state ){
		return state->reset;
	}

	command uint8_t ThreatModel.getMaxAging(){
		return maxAging;
	}

	command error_t ModelConfig.sync(){
		return SUCCESS;
	}
}