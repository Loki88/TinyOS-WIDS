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

configuration WIDSC {
		
	provides {
		interface Boot as Ready;
		interface AlarmGeneration;
		interface ThreatModel;
		interface ModelConfig;
		
	}
	uses {
		interface Boot;
		interface Leds;
		interface BusyWait<TMilli, uint16_t>;
		interface Notify<wids_observable_t> as Observable;
	}


} implementation {

	#ifndef TRACE_NUMBERS
	#define TRACE_NUMBERS 10
	#endif

	components new SimpleHashMapC(wids_state_trace_t, TRACE_NUMBERS) as HMap, WIDSManagerP;
	components new QueueC(wids_state_trace_t*, TRACE_NUMBERS);
	components new QueueC(wids_observable_t, TRACE_NUMBERS) as ObservableQueue;
	
	components WIDSThreatModelC as Model;

	AlarmGeneration = WIDSManagerP;
	ThreatModel = Model.ThreatModel;
	ModelConfig = Model.ModelConfig;
	Ready = Model;

	Boot = Model.Boot;	
	Leds = Model.Leds;
	BusyWait = Model.BusyWait;
	Observable = WIDSManagerP.Notify;


	WIDSManagerP.ThreatModel -> Model;
	WIDSManagerP.Traces -> QueueC;
	WIDSManagerP.HashMap -> HMap;
	WIDSManagerP.Observables -> ObservableQueue;
}