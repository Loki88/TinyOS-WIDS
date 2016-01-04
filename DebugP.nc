
#include "Wids.h"
#include "printf.h"

module DebugP {

	uses interface AlarmGeneration;

} implementation {

	wids_attack_t threatAttack = NO_ATTACK;
	uint8_t threatScore = 0;

	event void AlarmGeneration.attackFound(wids_attack_t attack, uint8_t score) {
		printf("Notified attack %s with score %d\n", printfAttack(attack), score);

		if( score > threatScore ) {
			threatScore = score;
			threatAttack = attack;
		}

		printf("The riskful threat is %s with score %d\n", printfAttack(threatAttack), threatScore);
		printfflush();
	}

}