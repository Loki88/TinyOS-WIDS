


#include "StorageVolumes.h"
#include "THREATMODEL.h"

configuration WIDSThreatModelC {

	provides interface ModelConfig;
	provides interface ThreatModel;

	uses interface Boot;
	uses interface Leds;
	uses interface BusyWait<TMilli, uint32_t>;

} implementation {

#ifndef HASHMAP_STATE_SIZE
#define HASHMAP_STATE_SIZE 10
#endif

	components WIDSThreatModelP as Model, WIDSConfigP as Config;
	components new SimpleHashMapC(wids_state_t, HASHMAP_STATE_SIZE) as States;

	Boot = WIDSConfigP.Boot;
	Leds = WIDSConfigP.Leds;
	BusyWait = WIDSConfigP.BusyWait;

	ModelConfig = Config.ModelConfig;
	ThreatModel = Model.ThreatModel;

	Model.HashMap -> States;
	Model.HashMapInit -> States;
	Config.TMConfig -> Model;


	// Volumes configuration
	components new ConfigStorageC(VOLUME_CONFIG) as ConfigVolume;

	components new BlockStorageC(VOLUME_OBSERVABLES) as ObservablesVolume;
	components new BlockStorageC(VOLUME_STATES) as StatesVolume;
	components new BlockStorageC(VOLUME_TRANSITIONS) as TransitionsVolume;
	

	Config.ConfigStorage -> ConfigVolume;
	Config.Mount -> ConfigVolume;

	Config.StateWrite -> StatesVolume;
	Config.StateRead -> StatesVolume;
	Config.TransitionWrite -> TransitionsVolume;
	Config.TransitionRead -> TransitionsVolume;
	Config.ObservableWrite -> ObservablesVolume;
	Config.ObservableRead -> ObservablesVolume;
	
}