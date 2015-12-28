


#include "StorageVolumes.h"
#include "WIDS.h"

configuration WIDSThreatModelC {

	provides interface ModelConfig;
	provides interface ThreatModel;

	uses interface Boot;
	uses interface Leds;
	uses interface BusyWait<TMilli, uint16_t>;

} implementation {

#ifndef HASHMAP_STATE_SIZE
#define HASHMAP_STATE_SIZE 10
#endif

	components WIDSThreatModelP as Model, WIDSConfigP as Config;
	components new SimpleHashMapC(wids_state_t, HASHMAP_STATE_SIZE) as States;

	Boot = Config.Boot;
	Leds = Config.Leds;
	BusyWait = Config.BusyWait;

	ModelConfig = Config.ModelConfig;
	ThreatModel = Model.ThreatModel;


	Model.HashMap -> States;
	Model.HashMapInit -> States;
	Config.TMConfig -> Model;
	Config.ThreatModel -> Model;
	Config.Init -> Model;

	// Volumes configuration
	components new BlockStorageC(VOLUME_CONFIG) as ConfigVolume;

	Config.ConfigWrite -> ConfigVolume;
	Config.ConfigRead -> ConfigVolume;

	
}