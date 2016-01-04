
#include "Wids.h"

module RemoteDetectionDummyP {
	
	provides interface Notify<wids_observable_t>;

} implementation {

	command error_t Notify.enable(){

	}

	command error_t Notify.disable(){

	}

}