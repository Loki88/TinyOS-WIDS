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

interface ModelConfig {

	command error_t createState( uint8_t id, wids_attack_t attack, uint8_t alarm_level );

	command error_t addTransition( uint8_t idFrom, uint8_t idTo );

	command error_t addObservable( uint8_t state_id, wids_observable_t observable );

	command error_t removeState( uint8_t state_id );

	command error_t sync();

	async event void syncDone();

	command error_t addResetObservable( uint8_t stateId, wids_observable_t obs );

	command error_t allowLoop( uint8_t stateId, bool loop );

	async command error_t setResetCount( uint8_t stateId, uint8_t resetCount );

}