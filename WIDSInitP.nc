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

module WIDSInitP {

	provides interface Boot as ModelReady;

	uses interface Boot;
	uses interface Init;
	uses interface ModelConfig as TMConfig;
}
implementation {

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

	uint8_t initLoops[12][2] = {
		{ 0x02, TRUE },
		{ 0x04, TRUE },
		{ 0x06, TRUE },
		{ 0x08, TRUE },
		{ 0x0A, TRUE },
		{ 0x0C, TRUE },
		{ 0x0E, TRUE },
		{ 0x10, TRUE },
		{ 0x12, TRUE },
		{ 0x14, TRUE },
		{ 0x16, TRUE },
		{ 0x18, TRUE },
	};

	uint8_t initResetObservables[24][2] = {
		{ 0x01, OBS_NONE},
		{ 0x02, OBS_NONE},
		{ 0x03, OBS_NONE},
		{ 0x04, OBS_NONE},
		{ 0x05, OBS_NONE},
		{ 0x06, OBS_NONE},
		{ 0x07, OBS_NONE},
		{ 0x08, OBS_NONE},
		{ 0x09, OBS_NONE},
		{ 0x0A, OBS_NONE},
		{ 0x0B, OBS_NONE},
		{ 0x0C, OBS_NONE},
		{ 0x0D, OBS_NONE},
		{ 0x0E, OBS_NONE},
		{ 0x0F, OBS_NONE},
		{ 0x10, OBS_NONE},
		{ 0x11, OBS_NONE},
		{ 0x12, OBS_NONE},
		{ 0x13, OBS_NONE},
		{ 0x14, OBS_NONE},
		{ 0x15, OBS_NONE},
		{ 0x16, OBS_NONE},
		{ 0x17, OBS_NONE},
		{ 0x18, OBS_NONE},
	};

	uint8_t initResetCount[24][2] = {
		{ 0x01, 1 },
		{ 0x02, 2 },
		{ 0x03, 1 },
		{ 0x04, 2 },
		{ 0x05, 1 },
		{ 0x06, 2 },
		{ 0x07, 1 },
		{ 0x08, 2 },
		{ 0x09, 1 },
		{ 0x0A, 2 },
		{ 0x0B, 1 },
		{ 0x0C, 2 },
		{ 0x0D, 10 },
		{ 0x0E, 10 },
		{ 0x0F, 1 },
		{ 0x10, 2 },
		{ 0x11, 1 },
		{ 0x12, 2 },
		{ 0x13, 1 },
		{ 0x14, 2 },
		{ 0x15, 1 },
		{ 0x16, 2 },
		{ 0x17, 1 },
		{ 0x18, 2 },
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

	uint8_t m_count = 0;
	// bool m_booted = FALSE;

	task void startingConfig(){
		uint8_t count = 0;
		uint8_t states = 24;
		uint8_t transitions = 24;
		uint8_t observables = 60;
		uint8_t resets = 24;
		uint8_t loops = 12;

		while ( count < states ) {
			uint8_t id = initStates[count][0];
			uint8_t attack = initStates[count][1];
			uint8_t alarm_level = initStates[count][2];
			call TMConfig.createState( id, attack, alarm_level );
			count += 1;
		}

		count = 0;
		while( count < transitions ){
			uint8_t from = initTransitions[count][0];
			uint8_t to = initTransitions[count][1];
			call TMConfig.addTransition( from, to );
			count += 1;
		}

		count = 0;
		while( count < observables ){
			uint8_t id = initObservables[count][0];
			wids_observable_t obs = initObservables[count][1];
			call TMConfig.addObservable( id, obs );
			count += 1;
		}

		count = 0;
		while( count < resets ){
			uint8_t id = initResetObservables[count][0];
			wids_observable_t obs = initResetObservables[count][1];
			call TMConfig.addResetObservable( id, obs );
			count += 1;
		}

		count = 0;
		while( count < loops ){
			uint8_t id = initLoops[count][0];
			bool loop = initLoops[count][1];
			call TMConfig.allowLoop( id, loop );
			count += 1;
		}

		count = 0;
		while( count < loops ){
			uint8_t id = initResetCount[count][0];
			uint8_t resetCount = initResetCount[count][1];
			call TMConfig.setResetCount( id, resetCount );
			count += 1;
		}

		signal ModelReady.booted();
	}

	event void Boot.booted(){
		call Init.init();
		post startingConfig();
	}

	async event void TMConfig.syncDone(){}

}