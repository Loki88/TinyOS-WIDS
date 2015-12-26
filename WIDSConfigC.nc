
#include "THREATMODEL.h"
#include "WIDS.h"

module WIDSConfigC {

	provides interface ModelConfig;


	uses interface Boot;
	uses interface Init;
	uses interface ModelConfig as TMConfig;
	uses interface ConfigStorage;
	uses interface BlockWrite as StateWrite;
	uses interface BlockRead as StateRead;
	uses interface BlockWrite as TransitionWrite;
	uses interface BlockRead as TransitionRead;
	uses interface BlockWrite as ObservableWrite;
	uses interface BlockRead as ObservableRead;
	uses interface Mount;
	uses interface Leds;
	uses interface BusyWait<TMilli, uint32_t>;

} implementation {

	uint32_t m_addr;
	uint8_t state[3], transition[2], observable[2];

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

	uint8_t initObservables[][2] = {
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
		{ 0x14, OBS_14  }, { 0x13, OBS_29  }, { 0x14, OBS_14  }, { 0x14, OBS_29 },

		// WORMHOLE
		{ 0x15, OBS_15  }, { 0x15, OBS_30  }, { 0x16, OBS_15 }, { 0x16, OBS_30  },
	}

	enum {
	    CONFIG_ADDR		= 0,
	    STATE_SIZE 		= 3,
	    TRANSITION_SIZE	= 2,
	    OBSERVABLE_SIZE	= 2,
	    ALARM_CYCLE		= 30,
	};

	struct config {
		uint8_t n_states;
		uint8_t n_transitions;
		uint8_t n_observables;
	} config_t;

	config_t m_configuration;

	task void configError(){
		uint8_t i = 0;
		while( i<ALARM_CYCLE ){
			call Leds.led0Toggle();
			call Leds.led1Toggle();
			call Leds.led2Toggle();
			call BusyWait.wait(500);
		}
	}

	inline void stateInit(wids_state_t *state){

	}

	task void startingConfig(){
		wids_state_t *resetState = call ModelConfig.getResetState();
		uint8_t count = 0;
		configuration.n_states = 24;
		configuration.n_transitions = 24;
		configuration.n_observables = 60;

		while ( count < configuration.n_states ) {
			call ModelConfig.createState(initStates[count][0], initStates[count][1], initStates[count][2]);
			count += 1;
		}

		count = 0;

		while( count < configuration.n_transitions ){
			call ModelConfig.addTransition( initTransitions[count][0], initTransitions[count][1] );
			count += 1;
		}

		count = 0;

		while( count < configuration.n_observables ){
			call ModelConfig.addObservable( initObservables[count][0], initTransitions[count][1] );
			count += 1;
		}

	}

	enum {
		LOAD_NONE 		= 0x00,
		LOAD_STATE_DONE = 0x01,
		LOAD_TRANS_DONE = 0x02,
		LOAD_DONE 		= 0x03,
	};

	uint8_t m_loadState = LOAD_NONE;

	task void loadConfiguration() { // when this is called m_configuration has been set by Config.read
		void *buff; // it is the pointer for the data on volume
		m_addr = 0;
		switch(m_loadState){
			case LOAD_NONE:
				call StateRead.read(m_addr, buff, STATE_SIZE);
				break;
			case LOAD_STATE_DONE:
				call TransitionRead.read(m_addr, buff, TRANSITION_SIZE);
				break;
			case LOAD_TRANS_DONE:
				call ObservableRead.read(m_addr, buff, OBSERVABLE_SIZE);
				break;
			case LOAD_DONE:

				break
			default:
				return;
		}
	}

	inline void confErrorHandling(){
		post configError();
    	post startingConfig();
	}

	async event Boot.booted(){
		call Init.init();
		if (call Mount.mount() != SUCCESS) {
	    	// configuration storage has not been mounted
	    	confErrorHandling();
	    } // else continue in Mount.mountDone()
	}

	event void Mount.mountDone(error_t error) {
		if (error == SUCCESS) {
			if (call Config.valid() == TRUE) {
				if (call Config.read(CONFIG_ADDR, &m_configuration, sizeof(config_t)) != SUCCESS) {
					confErrorHandling(); // we were unable to load saved configuration so get the default one
				}
				else {
					post loadConfiguration();
				}
			} else { // configuration is not valid
				confErrorHandling();
			}
	    } else { // it was not possible to mount the volume
	    	confErrorHandling();
	    }
	}

	async command error_t ModelConfig.finalize(){
		return call StateWrite.sync();
	}

	async command error_t ModelConfig.addState( uint8_t id, uint8_t attack, uint8_t alarm_level ){
		if( call TMConfig.addState(id, attack, alarm_level) != SUCCESS ){
			return FAIL;
		}

		uint8_t *buff = call ThreatModel.getState( id );

		return call StateWrite.write( m_configuration.n_states * STATE_SIZE, (void *)buff, STATE_SIZE);
	}

	async command error_t ModelConfig.addTransition( uint8_t idFrom, uint8_t idTo ) {
		if( call TMConfig.addTransition() != SUCCESS ) {
			return FAIL;
		}

		uint8_t *buff = malloc( TRANSITION_SIZE );
		*buff= idFrom;
		*(buff+1) = idTo;

		return call TransitionWrite.write( m_configuration.n_transitions * TRANSITION_SIZE, buff, TRANSITION_SIZE );
	}

	async command void ModelConfig.addObservable( uint8_t state_id, uint8_t observable ){
		if( call TMConfig.addTransition() != SUCCESS ) {
			return FAIL;
		}

		uint8_t *buff = malloc( OBSERVABLE_SIZE );
		*buff= state_id;
		*(buff+1) = observable;

		return call TransitionWrite.write( m_configuration.n_observables * OBSERVABLE_SIZE, buff, OBSERVABLE_SIZE );	
	}

	async command void ModelConfig.removeState( uint8_t state_id ){

	}

	event void StateRead.readDone(storage_addr_t addr, void* buf, storage_len_t len, 
		      error_t error){
		if ( error != SUCCESS ){ // skip all because the model could be inconsistent

		}

		// load the state and read the next one
		call ModelConfig.createState( *((uint8_t*)buf), *(((uint8_t*)buf)+1), *(((uint8_t*)buf)+2) );
		if ( addr < m_configuration.n_states * STATE_SIZE )
			call StateRead.read( addr + STATE_SIZE, buf, STATE_SIZE );
		else {
			m_loadState = LOAD_STATE_DONE;
			post loadConfiguration();
		}
	}

	event void StateRead.computeCrcDone(storage_addr_t addr, storage_len_t len,
			    uint16_t crc, error_t error){

	}

	event void StateWrite.writeDone(storage_addr_t addr, void* buf, storage_len_t len,
		       error_t error){
		if(error != SUCCESS){

		} else {
			m_configuration.n_states += 1;
		}

	}

	event void StateWrite.eraseDone(error_t error){

	}

	event void StateWrite.syncDone(error_t error){
		if( error != SUCCESS ){

		} else
			call TransitionWrite.sync();
	}

	event void TransitionRead.readDone(storage_addr_t addr, void* buf, storage_len_t len, 
		      error_t error){
		if ( error != SUCCESS ){ // skip all because the model could be inconsistent

		}

		// load the state and read the next one
		call ModelConfig.addTransition( *((uint8_t*)buf), *(((uint8_t*)buf)+1) );
		if ( addr < m_configuration.n_transitions * TRANSITION_SIZE )
			call StateRead.read( addr + TRANSITION_SIZE, buf, TRANSITION_SIZE );
		else {
			m_loadState = LOAD_TRANS_DONE;
			post loadConfiguration();
		}
	}

	event void TransitionRead.computeCrcDone(storage_addr_t addr, storage_len_t len,
			    uint16_t crc, error_t error){

	}

	event void TransitionWrite.writeDone(storage_addr_t addr, void* buf, storage_len_t len,
		       error_t error){
		if(error != SUCCESS){

		} else {
			m_configuration.n_transitions += 1;
		}
	}

	event void TransitionWrite.eraseDone(error_t error){

	}

	event void TransitionWrite.syncDone(error_t error){
		if( error != SUCCESS ){

		} else
			call ObservableWrite.sync();
	}

	event void ObservableRead.readDone(storage_addr_t addr, void* buf, storage_len_t len, 
		      error_t error){
		if ( error != SUCCESS ){ // skip all because the model could be inconsistent

		}

		// load the state and read the next one
		call ModelConfig.addObservable( *((uint8_t*)buf), *(((uint8_t*)buf)+1) );
		if ( addr < m_configuration.n_observables * OBSERVABLE_SIZE )
			call StateRead.read( addr + OBSERVABLE_SIZE, buf, OBSERVABLE_SIZE );
		else {
			m_loadState = LOAD_DONE;
			post loadConfiguration();
		}
	}

	event void ObservableRead.computeCrcDone(storage_addr_t addr, storage_len_t len,
			    uint16_t crc, error_t error){

	}

	event void ObservableWrite.writeDone(storage_addr_t addr, void* buf, storage_len_t len,
		       error_t error){
		if(error != SUCCESS){

		} else {
			m_configuration.n_observables += 1;
		}
	}

	event void ObservableWrite.eraseDone(error_t error){

	}

	event void ObservableWrite.syncDone(error_t error){
		if( error != SUCCESS ){

		} else {
			// write done
			call ConfigStorage.write( CONFIG_ADDR, &m_configuration, sizeof(m_configuration) );
		}
	}

	// TODO: aggiungere l'interfaccia per il ConfigStorage, sistemare i nomi e le interfacce presenti, 
	// ricontrollare la sequenza di stati e provare la compilazione, quindi fare in modo che sia testabile
	// sui nodi: a quel punto si disporrebbe del modello e di un metodo per la sua inizializzazione e
	// customizzazione a runtime
}