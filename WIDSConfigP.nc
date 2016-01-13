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

module WIDSConfigP {

	provides interface ModelConfig;
	provides interface Boot as ModelReady;

	uses interface Boot;
	uses interface Init;
	uses interface ModelConfig as TMConfig;
	
	uses interface ThreatModel;

	uses interface Mount;
	uses interface ConfigStorage;
	
	uses interface Leds;
	uses interface BusyWait<TMilli, uint16_t>;

} implementation {

	uint32_t m_addr;
	uint8_t buffer[4];
	norace bool m_sync = FALSE;
	norace bool mounted = FALSE;

	uint8_t initStates[24][3] = {
		{ 0x01, CONSTANT_JAMMING, LOW_LEV_THREAT },
		{ 0x02, CONSTANT_JAMMING, HIGH_LEV_THREAT },
		{ 0x03, DECEPTIVE_JAMMING, LOW_LEV_THREAT },
		{ 0x04, DECEPTIVE_JAMMING, HIGH_LEV_THREAT },
		{ 0x05, REACTIVE_JAMMING, LOW_LEV_THREAT },
		{ 0x06, REACTIVE_JAMMING, HIGH_LEV_THREAT },
		{ 0x07, RANDOM_JAMMING, LOW_LEV_THREAT },
		{ 0x08, RANDOM_JAMMING, HIGH_LEV_THREAT },
		{ 0x09, LINKLAYER_JAMMING, LOW_LEV_THREAT },
		{ 0x0A, LINKLAYER_JAMMING, HIGH_LEV_THREAT },
		{ 0x0B, BACKOFF_MANIPULATION, LOW_LEV_THREAT },
		{ 0x0C, BACKOFF_MANIPULATION, HIGH_LEV_THREAT },
		{ 0x0D, REPLAYPROTECTION_ATTACK, LOW_LEV_THREAT },
		{ 0x0E, REPLAYPROTECTION_ATTACK, HIGH_LEV_THREAT },
		{ 0x0F, GTS_ATTACK, LOW_LEV_THREAT },
		{ 0x10, GTS_ATTACK, HIGH_LEV_THREAT },
		{ 0x11, ACK_ATTACK, LOW_LEV_THREAT },
		{ 0x12, ACK_ATTACK, HIGH_LEV_THREAT },
		{ 0x13, SELECTIVE_FORWARDING, LOW_LEV_THREAT },
		{ 0x14, SELECTIVE_FORWARDING, HIGH_LEV_THREAT },
		{ 0x15, SYBIL, LOW_LEV_THREAT },
		{ 0x16, SYBIL, HIGH_LEV_THREAT },
		{ 0x17, WORMHOLE, LOW_LEV_THREAT },
		{ 0x18, WORMHOLE, HIGH_LEV_THREAT },
	};

	uint8_t initTransitions[24][2] = {
		{ 0x00, 0x01 }, { 0x00, 0x03 }, { 0x00, 0x05 }, { 0x00, 0x07 }, { 0x00, 0x09 },
		{ 0x00, 0x0B }, { 0x00, 0x0D }, { 0x00, 0x0F }, { 0x00, 0x11 }, { 0x00, 0x13 },
		{ 0x00, 0x15 }, { 0x00, 0x17 },
		{ 0x01, 0x02 }, { 0x03, 0x04 }, { 0x05, 0x06 }, { 0x07, 0x08 }, { 0x09, 0x0A },
		{ 0x0B, 0x0C }, { 0x0D, 0x0E }, { 0x0F, 0x10 }, { 0x11, 0x12 }, { 0x13, 0x14 },
		{ 0x15, 0x16 }, { 0x17, 0x18 },
	};

	uint8_t initObservables[60][2] = {
		// CONSTANT_JAMMING
		{ 0x01, OBS_1  }, { 0x01, OBS_16 }, { 0x02, OBS_1  }, { 0x02, OBS_16 },

		// DECEPTIVE JAMMING
		{ 0x03, OBS_2  }, { 0x03, OBS_17 }, { 0x04, OBS_2  }, { 0x04, OBS_17 },

		// REACTIVE_JAMMING
		{ 0x05, OBS_3  }, { 0x05, OBS_18 }, { 0x06, OBS_3  }, { 0x06, OBS_18 },

		// RANDOM_JAMMING
		{ 0x07, OBS_4  }, { 0x07, OBS_19 }, { 0x08, OBS_4  }, { 0x08, OBS_19 },

		// LINKLAYER_JAMMING
		{ 0x09, OBS_5  }, { 0x09, OBS_20 }, { 0x0A, OBS_5  }, { 0x0A, OBS_16 },

		// BACKOFF_MANIPULATION
		{ 0x0B, OBS_6  }, { 0x0B, OBS_7  }, { 0x0B, OBS_8  }, 
		{ 0x0B, OBS_21 }, { 0x0B, OBS_22 }, { 0x0B, OBS_23  }, 
		{ 0x0C, OBS_6  }, { 0x0C, OBS_7  }, { 0x0C, OBS_8  }, 
		{ 0x0C, OBS_21 }, { 0x0C, OBS_22 }, { 0x0C, OBS_23  }, 

		// REPLAYPROTECTION_ATTACK
		{ 0x0D, OBS_9  }, { 0x0D, OBS_24  }, { 0x0E, OBS_9 }, { 0x0E, OBS_24 },

		// GTS_ATTACK
		{ 0x0F, OBS_10  }, { 0x0F, OBS_25  }, { 0x10, OBS_10  }, { 0x10, OBS_25 },
		{ 0x0F, OBS_11  }, { 0x0F, OBS_26  }, { 0x10, OBS_11  }, { 0x10, OBS_26 },

		// ACK_ATTACK
		{ 0x11, OBS_12  }, { 0x11, OBS_27  }, { 0x12, OBS_12  }, { 0x12, OBS_27 },

		// SELECTIVE_FORWARDING
		{ 0x13, OBS_13  }, { 0x13, OBS_28  }, { 0x14, OBS_13  }, { 0x14, OBS_28 },

		// SYBIL
		{ 0x15, OBS_14  }, { 0x15, OBS_29  }, { 0x16, OBS_14  }, { 0x16, OBS_29 },

		// WORMHOLE
		{ 0x17, OBS_15  }, { 0x17, OBS_30  }, { 0x18, OBS_15 }, { 0x18, OBS_30  },
	};

	enum {
	    CONFIG_ADDR		= 0,
	    CONFIG_SIZE		= 7,

	    STATE_ADDR		= 7,
	    STATE_SIZE 		= 3,
	    
	    TRANSITION_ADDR	= 96,
	    TRANSITION_SIZE	= 2,

	    OBSERVABLE_ADDR = 1057,
	    OBSERVABLE_SIZE	= 2,
	    
	    ALARM_CYCLE		= 8,
	    SYNC_DELAY		= 600, // TODO: use a timer to delay synchronization of config in flash
	};

	typedef struct config {
		uint8_t n_states;
		uint8_t n_transitions;
		uint8_t n_observables;
	} config_t;

	config_t m_configuration;


	task void loadModel();
	task void syncModel();

	void configError(){
		uint8_t i = 0;

		while( i<ALARM_CYCLE ){
			call Leds.led0Toggle();
			call Leds.led1Toggle();
			call Leds.led2Toggle();
			call BusyWait.wait(500);
			i += 1;
		}
	}

	enum {
		WL_NONE 		= 0x00,
		LOAD_CONFIG		= 0x01,
		LOAD_STATE 		= 0x02,
		LOAD_TRANS 		= 0x03,
		LOAD_OBSER 		= 0x04,
		WRITE_CONFIG	= 0x05,
		WRITE_STATE		= 0x06,
		WRITE_TRANS		= 0x07,
		WRITE_OBSER		= 0x08,
	};

	uint8_t m_loadState = WL_NONE;

	uint8_t m_count = 0;

	void startingConfig(){
		uint8_t count = 0;
		uint8_t states = 24;
		uint8_t transitions = 24;
		uint8_t observables = 60;

		m_configuration.n_states = 0;
		m_configuration.n_transitions = 0;
		m_configuration.n_observables = 0;

		while ( count < states ) {
			uint8_t id = initStates[count][0];
			uint8_t attack = initStates[count][1];
			uint8_t alarm_level = initStates[count][2];
			call ModelConfig.createState( id, attack, alarm_level );
			count += 1;
			call Leds.led0Toggle();
		}

		count = 0;

		while( count < transitions ){
			uint8_t from = initTransitions[count][0];
			uint8_t to = initTransitions[count][1];
			call ModelConfig.addTransition( from, to );
			count += 1;
			call Leds.led0Toggle();
		}

		count = 0;

		while( count < observables ){
			uint8_t id = initObservables[count][0];
			wids_observable_t obs = initObservables[count][1];
			call ModelConfig.addObservable( id, obs );
			count += 1;
			call Leds.led0Toggle();
		}

		m_loadState = WL_NONE;
		call Leds.led1Toggle();
		call BusyWait.wait(1000);
		call Leds.led1Toggle();
		signal ModelReady.booted();
	}

	task void confErrorHandling(){
		configError();
    	startingConfig();
	}

	event void Boot.booted(){
		call Init.init();
		if (call Mount.mount() != SUCCESS) {
			printf("MOUNT NOT ACCEPTED\n");
			post confErrorHandling();
	    } // else continue in Mount.mountDone()
	}

	event void Mount.mountDone(error_t error) {
		if( error == SUCCESS ){
			mounted = TRUE;
			m_addr = CONFIG_ADDR;
			m_loadState = LOAD_CONFIG;
			post loadModel();
		} else { // not mounted, load base configuration
			printf("MOUNT FAILED\n");
			startingConfig();
		}
	}

	task void loadModel(){
		switch(m_loadState){
			case LOAD_CONFIG:	
				call ConfigStorage.read(CONFIG_ADDR, &m_configuration, sizeof(m_configuration));
				break;
			case LOAD_STATE:
				m_addr = STATE_ADDR + STATE_SIZE * m_count;
				call ConfigStorage.read(m_addr, &buffer, STATE_SIZE);
				break;
			case LOAD_TRANS:	
				m_addr = STATE_ADDR + TRANSITION_SIZE * m_count;
				call ConfigStorage.read(m_addr, &buffer, TRANSITION_SIZE);
				break;
			case LOAD_OBSER: 	
				m_addr = STATE_ADDR + OBSERVABLE_SIZE * m_count;
				call ConfigStorage.read(m_addr, &buffer, OBSERVABLE_SIZE);
				break;
		}
	}

	event void ConfigStorage.readDone(storage_addr_t addr, void* buf, storage_len_t len, 
		error_t error) {
		if( error == SUCCESS ){
			switch(m_loadState) {
				case LOAD_CONFIG:
					printf("ConfigLoad: n_states %d, n_observables %d, n_transitions %d\n", m_configuration.n_states,
						m_configuration.n_observables, m_configuration.n_transitions);
					m_loadState = LOAD_STATE;
					post loadModel();
					break;
				case LOAD_STATE:
					call TMConfig.createState(*((uint8_t*)buf), *((uint8_t*)buf+1), *((uint8_t*)buf+2));
					m_count += 1;
					if ( m_count >= m_configuration.n_states ){
						m_loadState = LOAD_TRANS;
						m_count = 0;
					}
					post loadModel();
					break;
				case LOAD_TRANS:
					call TMConfig.addTransition(*((uint8_t*)buf), *((uint8_t*)buf+1));
					m_count += 1;
					if ( m_count >= m_configuration.n_transitions ){
						m_loadState = LOAD_OBSER;
						m_count = 0;
					}
					post loadModel();
					break;
				case LOAD_OBSER:
					call TMConfig.addObservable(*((uint8_t*)buf), *((uint8_t*)buf+1));
					m_count += 1;
					if ( m_count >= m_configuration.n_observables ){
						m_loadState = WL_NONE;
						m_count = 0;
					}
					post loadModel();
					break;
			}
		}
		else { // clear the model and load the default one

		}
	}

	command error_t ModelConfig.createState( uint8_t id, wids_attack_t attack, uint8_t alarm_level ){
		if( call TMConfig.createState(id, attack, alarm_level) == SUCCESS ){
			m_configuration.n_states += 1;
			m_sync = TRUE;
			return SUCCESS;
		} else {
			return FAIL;
		}
	}

	command error_t ModelConfig.addTransition( uint8_t idFrom, uint8_t idTo ) {
		if( call TMConfig.addTransition(idFrom, idTo) == SUCCESS ) {
			m_configuration.n_transitions += 1;
			m_sync = TRUE;
			return SUCCESS;
		} else {
			return FAIL;
		}
	}

	command error_t ModelConfig.addObservable( uint8_t state_id, wids_observable_t obs ){
		if( call TMConfig.addObservable(state_id, obs) == SUCCESS ) {
			m_configuration.n_observables += 1;
			m_sync = TRUE;
			return SUCCESS;
		} else {
			return FAIL;
		}
	}

	command error_t ModelConfig.removeState( uint8_t state_id ){
		if( call TMConfig.removeState( state_id ) == SUCCESS ){
			// n_states, n_transitions and n_observables are no more valid
			m_sync == TRUE;
			return SUCCESS;
		} else {
			return FAIL;
		}
	}

	void *m_buffer = NULL;
	wids_state_t *m_state;

	command error_t ModelConfig.sync( ){
		printf("M_Sync from ModelConfig.sync is %d\n", m_sync);
		if( m_sync == FALSE)
			return EALREADY;
		else if( mounted == FALSE ){ // volume is not mounted and we can't write on it
			return FAIL;
		} else {
			m_state = call ThreatModel.getResetState();
			m_buffer = m_state -> transitions; // don't store reset state, but its transitions to other states

			m_configuration.n_states = 0; // reset configuration to count the real number of object written in memory
			m_configuration.n_transitions = 0;
			m_configuration.n_observables = 0;

			m_loadState = WRITE_TRANS;
			post syncModel();
			return SUCCESS;
		}
	}

	void syncStates(){
		if(m_state != NULL){
				printf("Sync state %d\n", m_state->id);
				buffer[0] = m_state -> id;
				buffer[1] = m_state -> attack;
				buffer[2] = m_state -> alarm_level;
				m_addr = STATE_ADDR + m_configuration.n_states*STATE_SIZE;
				call ConfigStorage.write(m_addr, &buffer, STATE_SIZE);
		} else {
			m_loadState = WL_NONE;
			post syncModel();
		}
	}

	void syncTransitions(){
		printf("TRANSITION\n");
		if(((wids_state_transition_t*)m_buffer) != NULL){
			buffer[0] = m_state -> id;
			buffer[1] = ((wids_state_transition_t*)m_buffer)->state->id;
			m_addr = TRANSITION_ADDR + m_configuration.n_transitions*TRANSITION_SIZE;
			call ConfigStorage.write(m_addr, &buffer, TRANSITION_SIZE);
		} else {
			printf("FOUND NULL\n");
			m_loadState = WRITE_OBSER;
			m_buffer = m_state->observables;
			post syncModel();
		}
	}

	void syncObservables(){
		if(((wids_obs_list_t*)m_buffer) != NULL){
			buffer[0] = m_state -> id;
			buffer[1] = ((wids_obs_list_t*)m_buffer)->obs;
			m_addr = OBSERVABLE_SIZE + m_configuration.n_observables*OBSERVABLE_SIZE;
			call ConfigStorage.write(m_addr, &buffer, OBSERVABLE_SIZE);
		} else {
			m_loadState = WRITE_STATE;
			post syncModel();
		}
	}

	task void syncModel(){
		switch(m_loadState){
			case WRITE_STATE:
				m_state = m_state->next;
				syncStates();
				break;
			case WRITE_TRANS:
				syncTransitions();
				break;
			case WRITE_OBSER:
				syncObservables();
				break;
			case WL_NONE:
				call ConfigStorage.write(CONFIG_ADDR, &m_configuration, CONFIG_SIZE);
				break;
		}
	}

	event void ConfigStorage.writeDone(storage_addr_t addr, void* buf, storage_len_t len, 
		       error_t error){
		if(error == SUCCESS){
			switch(m_loadState){
				case WRITE_STATE:
					m_configuration.n_states += 1;
					m_loadState = WRITE_TRANS;
					m_buffer = m_state->transitions;
					post syncModel();
					break;
				case WRITE_TRANS:
					m_configuration.n_transitions += 1;
					m_buffer = ((wids_state_transition_t*)m_buffer)->next;
					post syncModel();
					break;
				case WRITE_OBSER:
					m_configuration.n_observables += 1;
					m_buffer = ((wids_obs_list_t*)m_buffer)->next;
					post syncModel();
					break;
				case WL_NONE:
					if(call ConfigStorage.commit() != SUCCESS){
						// handle commit failure
					}
					break;
				default:
					return;
			}
			printf("Config: states %d, transitions %d, observables %d\n", m_configuration.n_states, 
				m_configuration.n_transitions, m_configuration.n_observables);
		} else {
			// handle failure
		}
	}

	event void ConfigStorage.commitDone(error_t error){
		if(error == SUCCESS){
			printf("commitDone\n");
			m_sync = FALSE;
			signal ModelConfig.syncDone();
		} else {

		}
	}

	async event void TMConfig.syncDone(){

	}

	default async event void ModelConfig.syncDone(){}
}