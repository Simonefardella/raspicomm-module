
#ifndef RASPICOMM_QUEUE_H
#define RASPICOMM_QUEUE_H

#define QUEUE_ITEM uint8_t
#define QUEUE_SIZE 256

typedef struct
{
  QUEUE_ITEM arr[QUEUE_SIZE];
  int read, write;
} queue_t;

int queue_get_room(queue_t* queue);
int queue_enqueue(queue_t* queue, QUEUE_ITEM item);
int queue_dequeue(queue_t* queue, QUEUE_ITEM* item);
int queue_is_empty(queue_t* queue);
int queue_is_full(queue_t* queue);

#endif // RASPICOMM_QUEUE_H

