#ifndef __UTIL_H
#define __UTIL_H


typedef struct list {

	void *element;
	void *key;
	struct list *next;

} list_t;


typedef struct linked_list {

	void *element;
	struct linked_list *next;

} linked_list_t;


#endif // __UTIL_H