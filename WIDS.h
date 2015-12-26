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
#include "TKN154.h"
#include "THREATMODEL.h"

#ifndef __WIDS_H
#define __WIDS_H

enum wids_alarm_level {
    
    HIGH_LEV_THREAT =  200,
    LOW_LEV_THREAT =   50,

};

typedef enum wids_attack {

	NO_ATTACK=					0X00,

	CONSTANT_JAMMING =			0X01,
	DECEPTIVE_JAMMING =			0X02,
	REACTIVE_JAMMING =			0X03,
	RANDOM_JAMMING =			0X04,

	LINKLAYER_JAMMING =			0x10,
	BACKOFF_MANIPULATION =		0x11,
	REPLAYPROTECTION_ATTACK = 	0x12,
	GTS_ATTACK =				0x13,
	ACK_ATTACK =				0x14,

	SELECTIVE_FORWARDING =		0xA0,
	SINKHOLE =					0xA1,
	SYBIL =						0xA2,
	WORMHOLE =					0xA3,
	HELLO_FLOODING =			0xA4,

} wids_attack_t;

typedef enum wids_observable {

	NONE		= 0x00,

	OBS_1		= 0x01,	// Constant jamming detected
	OBS_2		= 0x02,	// Deceptive jamming
	OBS_3		= 0x03,	// Reactive jamming
	OBS_4		= 0x04,	// Random jamming
	OBS_5		= 0x05,	// Link-layer jamming
	OBS_6		= 0x06,	// Backoff manipulation 1
	OBS_7		= 0x07,	// Backoff manipulation 2
	OBS_8		= 0x08,	// Backoff manipulation 3
	OBS_9		= 0x09,	// Replay protection attack
	OBS_10		= 0x0A,	// GTS attack - errors on slot
	OBS_11		= 0x0B,	// GTS attack - found a different transmitting mote
	OBS_12		= 0x0C,	// ACK attack
	OBS_13		= 0x0D,	// Selective forwarding & Sinkhole
	OBS_14		= 0x0E,	// Sybil
	OBS_15		= 0x0F,	// Wormhole


	// the followings are for cooperative detection
	OBS_16		= 0x10,	// Propagation of OBS_1 from cluster member
	OBS_17		= 0x11,	// Propagation of OBS_2 from cluster member
	OBS_18		= 0x12,	// Propagation of OBS_3 from cluster member
	OBS_19		= 0x13,	// Propagation of OBS_4 from cluster member
	OBS_20		= 0x14,	// Propagation of OBS_5 from cluster member
	OBS_21		= 0x15,	// Propagation of OBS_6 from cluster member
	OBS_22		= 0x16,	// Propagation of OBS_7 from cluster member
	OBS_23		= 0x17,	// Propagation of OBS_8 from cluster member
	OBS_24		= 0x18,	// Propagation of OBS_9 from cluster member
	OBS_25		= 0x19,	// Propagation of OBS_10 from cluster member
	OBS_26		= 0x1A,	// Propagation of OBS_11 from cluster member
	OBS_27		= 0x1B,	// Propagation of OBS_12 from cluster member
	OBS_28		= 0x1C,	// Propagation of OBS_13 from cluster member
	OBS_29		= 0x1D,	// Propagation of OBS_14 from cluster member
	OBS_30		= 0x1E,	// Propagation of OBS_15 from cluster member

} wids_observable_t;

typedef struct wids_rxFrame_detail {
	
	ieee154_address_t srcAddr;

	uint8_t seqNo;

} wids_rxFrame_detail_t;




#endif // __WIDS_H