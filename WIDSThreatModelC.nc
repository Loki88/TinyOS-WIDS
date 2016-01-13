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


#include "StorageVolumes.h"
#include "Wids.h"

configuration WIDSThreatModelC {

	provides interface Boot as ModelReady;
	provides interface ModelConfig;
	provides interface ThreatModel;

	uses interface Boot;

#ifdef WIDS_WRITE_CONFIG
	uses interface Leds;
	uses interface BusyWait<TMilli, uint16_t>;
#endif

} implementation {

#ifndef HASHMAP_STATE_SIZE
#define HASHMAP_STATE_SIZE 10
#endif

#ifdef WIDS_WRITE_CONFIG
	components WIDSConfigP as Config;
	Leds = Config.Leds;
	BusyWait = Config.BusyWait;

	// Volumes configuration
	components new ConfigStorageC(VOLUME_CONFIG) as ConfigVolume;

	Config.Mount -> ConfigVolume;
	Config.ConfigStorage -> ConfigVolume;
	ModelConfig = Config.ModelConfig;
	Config.ThreatModel -> Model;
#else
	components WIDSInitP as Config;
	ModelConfig = Model.ModelConfig;
#endif

	components WIDSThreatModelP as Model;
	components new SimpleHashMapC(wids_state_t, HASHMAP_STATE_SIZE) as States;

	Boot = Config.Boot;
	ModelReady = Config.ModelReady;
	
	ThreatModel = Model.ThreatModel;


	Model.HashMap -> States;
	Model.HashMapInit -> States;
	Config.TMConfig -> Model;
	Config.Init -> Model;

	

	
}