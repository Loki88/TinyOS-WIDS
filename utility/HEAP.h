#ifndef __HEAP_H
#define __HEAP_H


typedef struct heap_node {

	void *element;
	uint8_t key;
	uint8_t size;
	struct heap_node *parent;
	struct heap_node *lsibling;
	struct heap_node *rsibling;
	struct heap_node *child;

} heap_node_t;

typedef struct binomial_heap {
	
	heap_node_t *tree;
	struct binomial_heap *prev;
	struct binomial_heap *next;

} binomial_heap_t;


#endif // __HEAP_H