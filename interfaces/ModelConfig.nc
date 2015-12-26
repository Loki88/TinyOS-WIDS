
#include "THREATMODEL.h"

interface ModelConfig {

	async command uint8_t addState( uint8_t attack, uint8_t alarm_level );

	async command void addObservable( uint8_t state_id, uint8_t observable );

	async command void removeState( uint8_t state_id );

}