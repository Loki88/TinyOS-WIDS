
#include "WIDS.h"
#include "printf.h"

module WIDSThreatModelP {

	provides interface ThreatModel;
	provides interface ModelConfig;
	provides interface Init;

	uses interface HashMap<uint8_t, wids_state_t>;
	uses interface Init as HashMapInit;

} implementation {

	norace uint8_t m_id;

	command error_t Init.init(){
		call HashMapInit.init();
		call ModelConfig.createState(0, NO_ATTACK, 0);
		return SUCCESS;
	}

	command error_t ModelConfig.createState(uint8_t id, wids_attack_t att, uint8_t alarm){

		wids_state_t *state = malloc(sizeof(wids_state_t));
		state->id = id;
		state->attack = att;
		state->alarm_level = alarm;
		state->observables = NULL;
		state->transitions = NULL;
		state->next = NULL;

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
		printf("WIDSThreatModelP -> ModelConfig.addObservable(%d, %d)\r\n", stateId, obs);

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

	}

	command error_t ModelConfig.removeState( uint8_t stateId ) {
		// TODO: first unlink state from graph, then free the memory
		// TODO: la rimozione dello stato richiede la visita dell'intero grafo per determinare quali archi sino
		// 		entranti in esso e quindi rimuovere ogni riferimento dal modello
		m_id = stateId;

		visitSubtree( call ThreatModel.getResetState() );

		free( call HashMap.get(stateId) );
	}

	async command wids_state_t* ThreatModel.getResetState(){
		return call HashMap.get(0);
	}

	async command wids_state_t* ThreatModel.getState( uint8_t id ){
		return call HashMap.get( id );
	}

	async command linked_list_t* ThreatModel.getObservedStates( wids_state_t *state, wids_observable_t observable ) {

		wids_state_transition_t *neighbour = state->transitions; // visit all the states near the current one
		linked_list_t *observedStates = NULL, *tmp;

		while( neighbour != NULL ) {
			wids_obs_list_t *observables = neighbour->state->observables;

			while( observables != NULL ) {
				if( observables->obs == observable ) {
					if( observedStates == NULL ) { // initialize first element
						observedStates = malloc( sizeof(wids_state_t) + sizeof(linked_list_t*) );
						tmp = observedStates;
					} else {
						tmp->next = malloc( sizeof(wids_state_t) + sizeof(linked_list_t*) );
						tmp = tmp->next;
					}
					
					((wids_state_trace_t*)tmp->element)->state = neighbour->state;
					break; // don't continue this while since the state has yet been added
				}
				observables = observables->next;
			}
			neighbour = neighbour -> next;
		}

		return observedStates;

	}

	default event void ModelConfig.loadDone(){

	}
}