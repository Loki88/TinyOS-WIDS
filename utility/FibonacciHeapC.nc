
#include "UTIL.h"
#include "HEAP.h"

generic module FibonacciHeapC(typedef elementType) {

	provides interface WPriorityQueue<elementType, uint8_t>;
	provides interface Init;

	uses interface HashMap<uint16_t, heap_node_t>;
	uses interface Init as HashMapInit;

} implementation {

	inline void heapNodeInit( heap_node_t *node ){
		node->element = NULL;
		node->key = 0;
		node->size = 0;
		node->parent = NULL;
		node->lsibling = NULL;
		node->rsibling = NULL;
		node->child = NULL;
	}

	inline void binHeapInit( binomial_heap_t *heap ){
		heap->tree = NULL;
		heap->prev = NULL;
		heap->next = NULL;
	}

	binomial_heap_t *heap;
	binomial_heap_t *heapPtr;

	heap_node_t *max = NULL;

	void ristruttura();

	command error_t Init.init() {
		call HashMapInit.init();
	}

	void setMax() {
		heap_node_t *tmp;
		heapPtr = heap;
	
		while ( heapPtr != NULL ) {
			tmp = heapPtr->tree;

			while ( tmp->rsibling != NULL ) {
				if ( max == NULL )
					max = tmp;
				else {
					if ( tmp->key > max->key ) {
						max = tmp;	
					}
				}
				tmp = tmp->rsibling;
			}

			heapPtr = heapPtr->next;
		}
	}

	async command elementType* WPriorityQueue.findMax() {
		if ( max == NULL )
			setMax();

		return (elementType*) max->element;
	}

	async command elementType* WPriorityQueue.deleteMax() {

		heap_node_t *newTrees = max->child;
		heap_node_t *tmp;

		// first remove max from the list it belongs to
		if( max->lsibling != NULL ){
			max->lsibling->rsibling = max->rsibling;
		}
		if( max->rsibling != NULL ){
			max->rsibling->lsibling = max->lsibling;
		}

		while ( newTrees != NULL ) {
			uint8_t size = newTrees->size;

			heapPtr = heap;
			while (heapPtr->tree->size > size) 
				heapPtr = heapPtr->prev;
			while (heapPtr->tree->size < size) 
				heapPtr = heapPtr->next;
			tmp = newTrees->rsibling;
			newTrees->parent = NULL;
			newTrees->rsibling = heapPtr->tree;
			heapPtr->tree = newTrees;
			newTrees = tmp;
		}

		ristruttura();

		setMax();

		return (elementType*)max->element;
	}

	void ristruttura() {
		binomial_heap_t *tmp;
		heap_node_t *tree1, *tree2;
		bool compose = TRUE;
		heapPtr = heap;

		while ( heapPtr != NULL ) {
			tmp = heapPtr->next;
			tree1 = heapPtr->tree;

			if ( tree1 == NULL ) {
				heapPtr = heapPtr->next;
				continue;
			}

			if ( tree1->rsibling != NULL ) {
				tmp = malloc(sizeof(binomial_heap_t));
				binHeapInit( tmp );
				heapPtr->next = tmp;
				tmp->prev = heapPtr;
			} else {
				heapPtr = heapPtr->next;
				continue;
			}

			while ( tree1->rsibling != NULL ) {
				tree2 = tree1->rsibling;

				if ( tree1->key < tree2->key ) { // merge the trees
					tree1->rsibling = tree2->child; // tree1->rsibling == tree2 at this point
					tree2->child = tree1;
					tree1->parent = tree2; // tree1 has been moved under tree2, it is needed to set tree2

					heapPtr->tree = tree2->rsibling; // set the forest to tree2->rsibling
					tree2->size += 1;
					tree2->rsibling = tmp->tree;
					tmp->tree = tree2; // tree2 has moved to the high order forest
				} else {
					heapPtr->tree = tree2->rsibling;
					tree2->rsibling = tree1->child;
					tree1->child = tree2;
					tree2->parent = tree1;

					tree1->size += 1;
					tree1->rsibling = tmp->tree;
					tmp->tree = tree1;
				}

				tree1 = heapPtr->tree;
			}

			heapPtr = heapPtr->next;

		}
	}


	async command void WPriorityQueue.insert( elementType *element, uint8_t priority ) {

		heap_node_t *b0;
		b0 = malloc(sizeof(heap_node_t));
		heapNodeInit( b0 );
		b0->element = element;
		b0->key = priority;
		b0->size = 0;

		// the new element is in the head of forest for B0 trees in the head of heap*
		b0->rsibling = heap->tree;
		heap->tree = b0;

		if ( max == NULL || priority > max->key )
			max = b0;

		call HashMap.insert(b0, (uint16_t) element); 	// insert the new heap_node with key its memory address
											// the hash table helps with other operations.
											// So the heap node is identified by element address in memory.

		ristruttura();
	}

	void muoviAlto( heap_node_t *node ) {

		while ( node->parent != NULL && node->parent->key < node->key ) {
			heap_node_t *tmp;
			uint8_t size = node->parent->size;

			node->parent->size = node->size;
			node->size = size;

			if ( node->child != NULL ) {
				node->child->parent = node->parent;
				node->parent->child = node->child;
			}

			tmp = node->parent->rsibling;
			node->parent->rsibling = node->rsibling;
			node->rsibling = tmp;

			tmp = node->parent->lsibling;
			node->parent->lsibling = node->lsibling;
			node->lsibling = tmp;

			tmp = node->parent->parent;
			node->parent->parent = node;
			node->parent = tmp;
		}
	}

	async command void WPriorityQueue.increaseKey( elementType *element, uint8_t priority ) {

		heap_node_t *node = call HashMap.get( (uint16_t) element );	// the node associated with element
		node -> key += priority;
		muoviAlto( node );

	}


	void muoviBasso( heap_node_t *node ) {

		bool great = FALSE;

		while( node->child != NULL && great == FALSE ) {
			heap_node_t *tmp = node->child;
			heap_node_t *tmpMax = NULL;
			while( tmp != NULL ) {
				if ( (tmpMax != NULL && tmp->key > tmpMax->key) || tmp == NULL ) {
					tmpMax = tmp;
				}

				tmp = tmp->rsibling;
			}

			if( node->key < tmpMax->key ) {
				muoviAlto( tmpMax ); // instead of move down node we move up the child with greatest key
			} else {
				great = TRUE;
			}
		}

	}

	async command void WPriorityQueue.decreaseKey( elementType *element, uint8_t priority ) {

		heap_node_t *node = call HashMap.get( (uint16_t) element );	// the node associated with element
		node -> key -= priority;
		muoviBasso( node );

	}

	async command elementType* WPriorityQueue.delete( elementType *element ) {

		call WPriorityQueue.increaseKey( element, max->key + 1 );
		return call WPriorityQueue.deleteMax();

	}

	async command void WPriorityQueue.merge( linked_list_t* cp1, linked_list_t* cp2 ) {



	}
}