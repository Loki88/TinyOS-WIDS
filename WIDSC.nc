
#include "WIDS.h"

configuration WIDSC {
		
	provides {

	}


} implementation {

	components new HashMapC(uint8_t, wids_state_trace_t*, TRACE_NUMBERS), WIDSManagerP, WIDSThreatModelP;
	components 

}