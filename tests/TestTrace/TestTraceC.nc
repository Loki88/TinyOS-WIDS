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


#include "ThreatModel.h"
#include "TKN154.h"
#include "Wids.h"
#include "printf.h"

module TestTraceC @safe() {

	uses interface Boot;

	uses interface AlarmGeneration;
	uses interface ThreatModel;
	uses interface ModelConfig;
	provides interface ObservableNotify as Observable;

	uses interface Timer<TMilli>;

} implementation {

	uint8_t observables[] = { 
		OBS_1, OBS_1, OBS_16, OBS_16, // constant jamming should be signaled
		OBS_NONE, OBS_NONE, OBS_NONE, OBS_NONE, OBS_NONE, OBS_NONE, OBS_NONE, // verify the reset of traces
		OBS_2, OBS_5, OBS_2, OBS_2, OBS_2,
		OBS_5, OBS_5, OBS_5, OBS_5, OBS_10,
		OBS_10, OBS_10, OBS_10, OBS_10, OBS_10,
		OBS_10, OBS_10, OBS_10, OBS_10,			// all traces should be deleted leaving only the one relative to OBS_10
	};

	uint8_t index = 0;

	task void produceObservable();

	event void Boot.booted(){
		index = 0;
		call Timer.startPeriodic( 3000 );
	}

	async event void ModelConfig.syncDone(){}


	event void Timer.fired(){
		post produceObservable();
	}

	task void produceObservable(){
		wids_observable_t obs = observables[index];
		signal Observable.notify( obs );
		index += 1;
		if(index>=30)
			index = 0;
	}

	event void AlarmGeneration.traceLevelUpdate(wids_attack_t attack, uint8_t score){
		printf("Trace level update for attack %s, score %d\n", printfAttack(attack), score);
	}

	event void AlarmGeneration.newTraceFound(wids_attack_t attack, uint8_t score){
		printf("New trace created for attack %s, score %d\n", printfAttack(attack), score);
	}

}