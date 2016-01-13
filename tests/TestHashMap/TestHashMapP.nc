
#include "Wids.h"
#include "printf.h"

module TestHashMapP {
	uses interface Boot;
	uses interface HashMap<uint8_t, wids_observable_t>;
} implementation {

	event void Boot.booted(){
		wids_observable_t o;

		call HashMap.insert(OBS_1, 1);
		call HashMap.insert(OBS_2, 2);
		call HashMap.insert(OBS_23, 3);
		call HashMap.insert(OBS_5, 11);

		if( call HashMap.get(1, &o) == SUCCESS )
			printf("Get key 1 -> %s\n", printObservable(o));
		else
			printf("For key 1 no match\n");
		if( call HashMap.get(2, &o) == SUCCESS )
			printf("Get key 2 -> %s\n", printObservable(o));
		else
			printf("For key 2 no match\n");
		if( call HashMap.get(3, &o) == SUCCESS )
			printf("Get key 3 -> %s\n", printObservable(o));
		else
			printf("For key 3 no match\n");
		if( call HashMap.get(11, &o) == SUCCESS )
			printf("Get key 11 -> %s\n", printObservable(o));
		else
			printf("For key 11 no match\n");

		call HashMap.remove(3);
		printf("Remove element with key 3\n");

		if( call HashMap.get(1, &o) == SUCCESS )
			printf("Get key 1 -> %s\n", printObservable(o));
		else
			printf("For key 1 no match\n");
		if( call HashMap.get(2, &o) == SUCCESS )
			printf("Get key 2 -> %s\n", printObservable(o));
		else
			printf("For key 2 no match\n");
		if( call HashMap.get(3, &o) == SUCCESS )
			printf("Get key 3 -> %s\n", printObservable(o));
		else
			printf("For key 3 no match\n");
		if( call HashMap.get(11, &o) == SUCCESS )
			printf("Get key 11 -> %s\n", printObservable(o));
		else
			printf("For key 11 no match\n");

		call HashMap.remove(1);
		printf("Remove element with key 3\n");

		if( call HashMap.get(1, &o) == SUCCESS )
			printf("Get key 1 -> %s\n", printObservable(o));
		else
			printf("For key 1 no match\n");
		if( call HashMap.get(2, &o) == SUCCESS )
			printf("Get key 2 -> %s\n", printObservable(o));
		else
			printf("For key 2 no match\n");
		if( call HashMap.get(3, &o) == SUCCESS )
			printf("Get key 3 -> %s\n", printObservable(o));
		else
			printf("For key 3 no match\n");
		if( call HashMap.get(11, &o) == SUCCESS )
			printf("Get key 11 -> %s\n", printObservable(o));
		else
			printf("For key 11 no match\n");
	}

}