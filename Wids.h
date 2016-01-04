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
#include "TKN154.h"
#include "ThreatModel.h"

#ifndef __WIDS_H
#define __WIDS_H

#define RESET_COUNT 5


enum wids_alarm_level {
    
    HIGH_LEV_THREAT =  20,
    LOW_LEV_THREAT =   5,

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

	OBS_NONE	= 0x00,

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


// typedef enum wids_observable {

// 	NONE		= 0,

// 	OBS_1		= 1,	// Constant jamming detected
// 	OBS_2		= 2,	// Deceptive jamming
// 	OBS_3		= 3,	// Reactive jamming
// 	OBS_4		= 4,	// Random jamming
// 	OBS_5		= 5,	// Link-layer jamming
// 	OBS_6		= 6,	// Backoff manipulation 1
// 	OBS_7		= 7,	// Backoff manipulation 2
// 	OBS_8		= 8,	// Backoff manipulation 3
// 	OBS_9		= 9,	// Replay protection attack
// 	OBS_10		= 10,	// GTS attack - errors on slot
// 	OBS_11		= 11,	// GTS attack - found a different transmitting mote
// 	OBS_12		= 12,	// ACK attack
// 	OBS_13		= 13,	// Selective forwarding & Sinkhole
// 	OBS_14		= 14,	// Sybil
// 	OBS_15		= 15,	// Wormhole


// 	// the followis are for cooperative detection
// 	OBS_16		= 16,	// Propagation of OBS_1 from cluster member
// 	OBS_17		= 17,	// Propagation of OBS_2 from cluster member
// 	OBS_18		= 18,	// Propagation of OBS_3 from cluster member
// 	OBS_19		= 19,	// Propagation of OBS_4 from cluster member
// 	OBS_20		= 20,	// Propagation of OBS_5 from cluster member
// 	OBS_21		= 21,	// Propagation of OBS_6 from cluster member
// 	OBS_22		= 22,	// Propagation of OBS_7 from cluster member
// 	OBS_23		= 23,	// Propagation of OBS_8 from cluster member
// 	OBS_24		= 24,	// Propagation of OBS_9 from cluster member
// 	OBS_25		= 25,	// Propagation of OBS_10 from cluster member
// 	OBS_26		= 26,	// Propagation of OBS_11 from cluster member
// 	OBS_27		= 27,	// Propagation of OBS_12 from cluster member
// 	OBS_28		= 28,	// Propagation of OBS_13 from cluster member
// 	OBS_29		= 29,	// Propagation of OBS_14 from cluster member
// 	OBS_30		= 30,	// Propagation of OBS_15 from cluster member

// } wids_observable_t;

char* printfAttack(wids_attack_t a){
	switch(a){
		case NO_ATTACK:
			return "NO_ATTACK";
			break;
		case CONSTANT_JAMMING:
			return "CONSTANT_JAMMING";
			break;
		case DECEPTIVE_JAMMING:
			return "DECEPTIVE_JAMMING";
			break;
		case REACTIVE_JAMMING:
			return "REACTIVE_JAMMING";
			break;
		case RANDOM_JAMMING:
			return "RANDOM_JAMMING";
			break;
		case LINKLAYER_JAMMING:
			return "LINKLAYER_JAMMING";
			break;
		case BACKOFF_MANIPULATION:
			return "BACKOFF_MANIPULATION";
			break;
		case REPLAYPROTECTION_ATTACK:
			return "REPLAYPROTECTION_ATTACK";
			break;
		case GTS_ATTACK:
			return "GTS_ATTACK";
			break;
		case ACK_ATTACK:
			return "ACK_ATTACK";
			break;
		case SELECTIVE_FORWARDING:
			return "SELECTIVE_FORWARDING";
			break;
		case SINKHOLE:
			return "SINKHOLE";
			break;
		case SYBIL:
			return "SYBIL";
			break;
		case WORMHOLE:
			return "WORMHOLE";
			break;
		case HELLO_FLOODING:
			return "HELLO_FLOODING";
			break;
		default:
			return "UNKNOWN ATTACK";
	}
}

char* printObservable(wids_observable_t o){
	switch(o){
		case OBS_1: 
			return "OBS_1";
			break;
		case OBS_2: 
			return "OBS_2";
			break;
		case OBS_3: 
			return "OBS_3";
			break;
		case OBS_4: 
			return "OBS_4";
			break;
		case OBS_5: 
			return "OBS_5";
			break;
		case OBS_6: 
			return "OBS_6";
			break;
		case OBS_7: 
			return "OBS_7";
			break;
		case OBS_8: 
			return "OBS_8";
			break;
		case OBS_9: 
			return "OBS_9";
			break;
		case OBS_10: 
			return "OBS_10";
			break;
		case OBS_11: 
			return "OBS_11";
			break;
		case OBS_12: 
			return "OBS_12";
			break;
		case OBS_13: 
			return "OBS_13";
			break;
		case OBS_14: 
			return "OBS_14";
			break;
		case OBS_15: 
			return "OBS_15";
			break;
		case OBS_16: 
			return "OBS_16";
			break;
		case OBS_17: 
			return "OBS_17";
			break;
		case OBS_18: 
			return "OBS_18";
			break;
		case OBS_19: 
			return "OBS_19";
			break;
		case OBS_20: 
			return "OBS_20";
			break;
		case OBS_21: 
			return "OBS_21";
			break;
		case OBS_22: 
			return "OBS_22";
			break;
		case OBS_23: 
			return "OBS_23";
			break;
		case OBS_24: 
			return "OBS_24";
			break;
		case OBS_25: 
			return "OBS_25";
			break;
		case OBS_26: 
			return "OBS_26";
			break;
		case OBS_27: 
			return "OBS_27";
			break;
		case OBS_28: 
			return "OBS_28";
			break;
		case OBS_29: 
			return "OBS_29";
			break;
		case OBS_30: 
			return "OBS_30";
			break;
		default:
			return "";
	}
}

typedef struct wids_rxFrame_detail {
	
	ieee154_address_t srcAddr;

	uint8_t seqNo;

} wids_rxFrame_detail_t;


typedef enum wids_status {

	TX_SUCCESSFUL = 0x01,
	TX_CCA_FAILED = 0x02,
	TX_ACK_FAILED = 0x03,

	RX_SUCCESSFUL = 0x10,
	RX_CRC_FAILED = 0x11,
	RX_GTS_FAILED = 0x12,
	
	CTRL_TIMEOUT  = 0x20,

} wids_status_t;

#endif // __WIDS_H