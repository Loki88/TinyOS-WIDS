
#include "THREATMODEL.h"
#include "WIDS.h"
#include "printf.h"

module TestConfC @safe() {

	uses interface Boot;

	uses interface ThreatModel;
	uses interface ModelConfig;

	uses interface Leds;
	uses interface Timer<TMilli>;
	uses interface BusyWait<TMilli, uint16_t>;

} implementation {

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

	
	event void Boot.booted(){

	}

	event void Timer.fired(){

	}

	void assertStates(){
		uint8_t i=0;
		while( i < 24 ){

			wids_state_t *state = call ThreatModel.getState( initStates[i][0] );
			wids_obs_list_t *obs = state->observables;
			wids_state_transition_t *trans = state->transitions;

			printf("Looking for state %d\r\n", initStates[i][0]);

			if ( state != NULL ){
				
				printf("State %d attack %d\n", state->id, state->attack);
				
				printf("\t Observables:\n");
				while( obs != NULL ){
					printf("\t\t- Obs id: %s \n", printObservable(obs->obs));
					obs = obs->next;
				}

				printf("\t Reachable states:\n");
				while( trans != NULL ){
					printf("\t\t - State id: %d\n", trans->state->id);
					trans = trans->next;
				}

				if ( state->id != initStates[i][0] || state->attack != initStates[i][1] 
					|| state->alarm_level != initStates[i][2]){
					printf("State Error -> not default value!\n");
				}
				printfflush();
			}
			i += 1;
		}
	}

	task void validateConfig() {
		assertStates();
	}

	event void ModelConfig.loadDone(){
		printf("LOAD DONE\n");
		post validateConfig();

		// call Timer.startPeriodic( 1000 );
	}

}