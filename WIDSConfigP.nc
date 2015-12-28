

#include "WIDS.h"
#include "printf.h"

module WIDSConfigP {

	provides interface ModelConfig;

	uses interface Boot;
	uses interface Init;
	uses interface ModelConfig as TMConfig;
	
	uses interface ThreatModel;

	uses interface BlockWrite as ConfigWrite;
	uses interface BlockRead as ConfigRead;
	
	uses interface Leds;
	uses interface BusyWait<TMilli, uint16_t>;

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
		{ 0x14, OBS_14  }, { 0x13, OBS_29  }, { 0x14, OBS_14  }, { 0x14, OBS_29 },

		// WORMHOLE
		{ 0x15, OBS_15  }, { 0x15, OBS_30  }, { 0x16, OBS_15 }, { 0x16, OBS_30  },
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

	task void loadConfiguration();

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

	void *buffer;

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
			// printf("inserting state %d for attack %d, alert %d\r\n", initStates[count][0], initStates[count][1], initStates[count][2]);
			call ModelConfig.createState(initStates[count][0], initStates[count][1], initStates[count][2]);
			count += 1;
			call Leds.led0Toggle();
		}

		count = 0;

		while( count < transitions ){
			// printf("inserting transition between states %d -> %d\r\n", initTransitions[count][0], initTransitions[count][1]);
			call ModelConfig.addTransition( initTransitions[count][0], initTransitions[count][1] );
			count += 1;
			call Leds.led0Toggle();
		}

		count = 0;

		while( count < observables ){
			printf("assigning observable %d to state %d\r\n", initObservables[count][1], initObservables[count][0]);
			call ModelConfig.addObservable( initObservables[count][0], initTransitions[count][1] );
			count += 1;
			call Leds.led0Toggle();
		}

		printf("OBS_16 %d\r\n", OBS_16);

		m_loadState = WL_NONE;
		call Leds.led1Toggle();
		call BusyWait.wait(1000);
		call Leds.led1Toggle();
		signal ModelConfig.loadDone();
	}

	inline void readErr(uint32_t addr, uint32_t len ){
		call Leds.led2Toggle(); // we were unable to load the element, signal and skip it
		m_addr = addr+len;
		m_count += 1;
		post loadConfiguration();
	}

	task void loadConfiguration() { // when this is called m_configuration has been set by Config.read
		uint8_t size;
		switch (m_loadState){
			case LOAD_STATE:
				if(m_count >= m_configuration.n_states){
					m_loadState = LOAD_TRANS;
					m_addr = TRANSITION_ADDR;
					post loadConfiguration();
					return;
				} else {
					size = STATE_SIZE;
				}
				break;
			case LOAD_TRANS:
				if(m_count >= m_configuration.n_transitions){
					m_loadState = LOAD_OBSER;
					m_addr = OBSERVABLE_ADDR;
					post loadConfiguration();
					return;
				} else {
					size = TRANSITION_SIZE;
				}
				break;
			case LOAD_OBSER:
				if(m_count >= m_configuration.n_observables){
					m_loadState = WL_NONE;
					signal ModelConfig.loadDone();
					return;
				} else {
					size = OBSERVABLE_SIZE;
				}
				break;
			default:
				return;
		}

		if( call ConfigRead.computeCrc( m_addr, size, 0 ) != SUCCESS ){
			readErr(m_addr, size);
		}
	}

	task void confErrorHandling(){
		configError();
    	startingConfig();
	}

	event void Boot.booted(){
		call Init.init();
		m_addr = CONFIG_ADDR;
		if (call ConfigRead.computeCrc(m_addr, CONFIG_SIZE, 0) != SUCCESS) {
	    	// configuration could not be validated
	    	post confErrorHandling();
	    } // else continue in ConfigRead.computeCrcDone()
	}

	command error_t ModelConfig.createState( uint8_t id, wids_attack_t attack, uint8_t alarm_level ){
		if( call TMConfig.createState(id, attack, alarm_level) != SUCCESS ){
			uint8_t *buff = (uint8_t*) call ThreatModel.getState(id);

			return call ConfigWrite.write(m_configuration.n_states * STATE_SIZE, (void *)buff, STATE_SIZE);
		} else {
			return FAIL;
		}
	}

	command error_t ModelConfig.addTransition( uint8_t idFrom, uint8_t idTo ) {
		if( call TMConfig.addTransition(idFrom, idTo) == SUCCESS ) {
			uint8_t *buff = malloc( TRANSITION_SIZE );
			*buff= idFrom;
			*(buff+1) = idTo;

			return call ConfigWrite.write(m_configuration.n_transitions * TRANSITION_SIZE, buff, TRANSITION_SIZE );
		} else {
			return FAIL;
		}
	}

	command error_t ModelConfig.addObservable( uint8_t state_id, wids_observable_t obs ){
		printf("WIDSConfigP -> ModelConfig.addObservable(%d, %d)\r\n", state_id, obs);

		if( call TMConfig.addObservable(state_id, obs) == SUCCESS ) {
			uint8_t *buff = malloc( OBSERVABLE_SIZE );
			*buff= state_id;
			*(buff+1) = obs;

			return call ConfigWrite.write(m_configuration.n_observables * OBSERVABLE_SIZE, buff, OBSERVABLE_SIZE );
		} else {
			return FAIL;
		}
	}

	command error_t ModelConfig.removeState( uint8_t state_id ){

	}

	event void ConfigRead.readDone(storage_addr_t addr, void* buf, storage_len_t len, 
		      error_t error){

		if ( error != SUCCESS ){ // skip all because the model could be inconsistent
			if ( m_loadState == LOAD_CONFIG ){
				post confErrorHandling(); // configuration has not been read correctly
			}
			return;
		}

		m_addr = addr + len;

		switch (m_loadState){
			case LOAD_CONFIG:
				m_configuration.n_states = *((uint8_t*)buf);
				m_configuration.n_transitions = *(((uint8_t*)buf)+1);
				m_configuration.n_observables = *(((uint8_t*)buf)+2);
				m_loadState = LOAD_STATE;
				m_count = 0;
				break;
			case LOAD_STATE:
				call TMConfig.createState( *((uint8_t*)buf), *(((uint8_t*)buf)+1), *(((uint8_t*)buf)+2) );
				m_count += 1;
				break;
			case LOAD_TRANS:
				call TMConfig.addTransition( *((uint8_t*)buf), *(((uint8_t*)buf)+1) );
				m_count += 1;
				break;
			case LOAD_OBSER:
				call TMConfig.addObservable( *((uint8_t*)buf), *(((uint8_t*)buf)+1) );
				m_count += 1;
				break;
			default:
				return;
		}
		
		// continue the reading
		post loadConfiguration();
	}

	event void ConfigRead.computeCrcDone(storage_addr_t addr, storage_len_t len,
			    uint16_t crc, error_t error){
		if (error == SUCCESS) { // CRC has been computed
			if ( crc == 0 ) { // data is valid
				if(m_loadState == WL_NONE){
					m_loadState = LOAD_CONFIG;
					if (call ConfigRead.read(CONFIG_ADDR, buffer, CONFIG_SIZE) != SUCCESS) {
						post confErrorHandling(); // we were unable to load saved configuration so get the default one
					} else { // else continues in ConfigRead.readDone()

					}
				} else {
					if (call ConfigRead.read(addr, buffer, len) != SUCCESS) {
						readErr(addr, len);
					} else { // else continues in ConfigRead.readDone()

					}
				}
			} else { // configuration is not valid
				post confErrorHandling();
			}
	    } else { // it was not possible to mount the volume
	    	post confErrorHandling();
	    }
	}

	event void ConfigWrite.writeDone(storage_addr_t addr, void* buf, storage_len_t len,
		       error_t error){
		printf("WRITTEN");
		if(error != SUCCESS){ // data has not been written, the state could not be restored after poweroff

		} else {
			switch(m_loadState){
				case WRITE_STATE:
					m_configuration.n_states += 1;
					break;
				case WRITE_TRANS:
					m_configuration.n_transitions += 1;
					break;
				case WRITE_OBSER:
					m_configuration.n_observables += 1;
					break;
			}
			// TODO: delay timeout to sync -> on timeout write m_configuration
		}

	}

	event void ConfigWrite.eraseDone(error_t error){

	}

	event void ConfigWrite.syncDone(error_t error){
		
	}

	default event void ModelConfig.loadDone(){
		
	}

	event void TMConfig.loadDone(){} // does nothing
}