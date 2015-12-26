
#include "UTIL.h"

interface WPriorityQueue<t, p> {

	async command t* findMax();

	async command t* deleteMax();

	async command void insert( t *element, p priority );

	async command void increaseKey( t *element, p priority );

	async command void decreaseKey( t *element, p priority );

	async command t* delete( t *element );

	async command void merge( linked_list_t* cp1, linked_list_t* cp2 );

}