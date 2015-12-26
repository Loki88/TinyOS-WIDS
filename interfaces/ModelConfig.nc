
#include "THREATMODEL.h"

interface ModelConfig {

	async command error_t createState( uint8_t id, wids_attack_t attack, uint8_t alarm_level );

	async command error_t addTransition( uint8_t idFrom, uint8_t idTo );

	async command error_t addObservable( uint8_t state_id, wids_observable_t observable );

	async command error_t removeState( uint8_t state_id );

}