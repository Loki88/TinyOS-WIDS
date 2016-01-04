
#include "Wids.h"

interface SystemInfo {

	async command bool cca();

	async command uint8_t getFreeRxQueueSize();

	async command uint8_t getLastDSN(message_t *msg);

	async command bool isDetectedRemotely(wids_observable_t obs);
	
}