/*****************************************************************************************

    Copyright © 2015 Lorenzo Di Giuseppe <lorenzo.digiuseppe88@gmail.com>.
    All Rights Reserved.

    Redistribution and use in source and binary forms, with or without modification, 
    are permitted provided that the following conditions are met:

    1. Redistributions of source code must retain the above copyright notice, this 
    	list of conditions and the following disclaimer.

    2. Redistributions in binary form must reproduce the above copyright notice, 
    	this list of conditions and the following disclaimer in the documentation 
    	and/or other materials provided with the distribution.

    3. The name of the author may not be used to endorse or promote products derived 
    	from this software without specific prior written permission.

    THIS SOFTWARE IS PROVIDED BY Lorenzo Di Giuseppe "AS IS" AND ANY EXPRESS OR IMPLIED 
    WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY 
    AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE AUTHOR BE 
    LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL 
    DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
    LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY 
    THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
    NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN 
    IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*****************************************************************************************/

#ifndef __THREATMODEL_H
#define __THREATMODEL_H

#include "WIDS.h"

#define TRACE_NUMBERS 10
#define RESET_COUNT 5

enum model_evaluation_mode {

    TIME_SCHEDULED,     // new traces are evaluated periodically
    EVENT_BASED,        // new traces are evaluated when an observable is notified
    ON_DEMAND,          // new traces are evaluated only when required

};

typedef struct wids_obs_list {

    uint8_t obs;
    struct wids_obs_list *next;

} wids_obs_list_t;

typedef struct wids_state {

    uint8_t id;
    wids_attack_t attack;
    uint8_t alarm_level;
    struct wids_obs_list *observables;
    struct wids_state_transition *transitions;
    
    struct wids_state *next;

} wids_state_t;

typedef struct wids_state_transition {
    
    wids_state_t *state;
    struct wids_state_transition *next;

} wids_state_transition_t;

typedef struct wids_threat_model {

    wids_state_t *states;

} wids_threat_model_t;

typedef struct wids_state_trace {

    wids_state_t *state;
    uint8_t observation_count;
    uint8_t alarm_value;

} wids_state_trace_t;

#endif // __THREATMODEL_H