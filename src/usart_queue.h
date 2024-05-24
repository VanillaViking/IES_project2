#define BUFFER_SIZE 75

// A structure to represent a queue
typedef struct Queue {
    int head; 
    int tail;
    int size;
    int capacity;
    char array[BUFFER_SIZE];
} Queue;

Queue queue_init();
char dequeue(Queue* queue);
void enqueue(Queue* queue, char item);
void enqueue_string(Queue* queue, char* str);
int is_empty(Queue* queue);
int is_full(Queue* queue);
