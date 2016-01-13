
#include "printf.h"

configuration TestHashMapAppC {	
} implementation {

	components TestHashMapP as App, MainC, new SimpleHashMapC(wids_observable_t, 10);
	components SerialPrintfC;

	App.Boot -> MainC;
	App.HashMap -> SimpleHashMapC;
}