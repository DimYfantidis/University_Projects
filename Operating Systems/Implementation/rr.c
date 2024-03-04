#include <stdio.h>
#include <stdlib.h>


typedef struct 
{
	int pid;
	int arrival_time;
	int burst_time;
} 
process;


// ----------- Double-ended Process Queue Implementation (START) -----------

// Building block of the following queue structure.
struct queue_node
{
    process* data;              // Reference to a process which, in the current project, resides in the stack.
    struct queue_node* next;    // The node that has priority directly after the current one.
};
typedef struct queue_node queue_node;

// Single-linked FIFO structure with head and tail nodes.
typedef struct {
    queue_node* head;
    queue_node* tail;
    int size;
} queue;

// Initializes queue's fields (Constructor)
void init_queue(queue* THIS)
{
    THIS->head = NULL;
    THIS->tail = NULL;
    THIS->size = 0;
}

// Deallocates queue's nodes from memory (Destructor)
void destroy_queue(queue* THIS)
{
    queue_node* temp;

    for (queue_node* iterator = THIS->head; iterator != NULL; iterator = temp)
    {
        temp = iterator->next;
        free(iterator);

        /*  The "data" field of a list node is supposed to be a reference to a
         *  statically allocated process and not the return value of a malloc().
         *  Thus, it is not deallocated with free. */
    }
}

// Adds a new process at the end of the queue.
void append_to_queue(queue* THIS, process* p)
{
    queue_node* old_tail = THIS->tail;

    THIS->tail = (queue_node*)malloc(sizeof(queue_node));
    THIS->tail->data = p;
    THIS->tail->next = NULL;

    THIS->size++;

    if (old_tail != NULL) {
        old_tail->next = THIS->tail;
    }
    else {
        // Special case: Queue was empty before insertion.
        THIS->head = THIS->tail;
    }
}

// Selects process from the head of the queue and removes it.
process* extract_front_of_queue(queue* THIS)
{
    // Returns NULL if empty.
    if (THIS->size == 0) {
        return NULL;
    }
    process* front = THIS->head->data;

    // Special case for list.head == list.tail
    if (THIS->size == 1)
    {
        // Removes the element from the front of the queue.
        free(THIS->head);

        // Head and tail are reset.
        THIS->head = NULL;
        THIS->tail = NULL;

        THIS->size--;   // size = 0

        return front;
    }
    queue_node* old_head = THIS->head->next;

    free(THIS->head);

    // The old head's next node comes forward and becomes the new head.
    THIS->head = old_head;

    THIS->size--;

    return front;
}

// ----------- Double-ended Process Queue Implementation (END) -----------

//  Implementation of Round Robin scheduling algorithm.
// 		@param running_processes: array of processes for scheduling
//		@param n: array size
//      @param q: time quantum of process execution
void round_robin(process* P, int n, int q)
{
    /* FIFO data structure for keeping track of the processes that
     * have arrived and are awaiting execution. */
    queue process_queue;

    /* Keeps track of elapsed time in process execution units. Used for checking
     * the arrival of processes and inserting them to the queue.
     *
     * Variable is initialized to -1 (not 0) to prevent an infinite loop,
     * since "P[0].arrival_time = 0" is allowed (P[0]'s insertion in the process
     * queue is not implemented as a special case).*/
    int elapsed_time = -1;

    /* Index for inserting the next process in the queue. Basically, given that
     * i and j are positive integers and:
     *      P[i] is currently being executed, or
     *      P[i] was the last process to be executed, or
     *      No process at all has been executed (j = 0),
     *
     * then "next_process_index" represents j where j > i.
     * This is necessary in order to perform the check
     *
     *      elapsed_time == P[j].arrival_time
     *
     * in order to update the queue by inserting the arriving process in it. */
    int next_process_index = 0;

    /* Keeps track of the number of completed processes. Used to terminate
     * the algorithm if the given n processes are executed.
     *
     * Normally scheduling algorithms run endlessly but, for the purposes of
     * this project, this variable is necessary. */
    int completed_processes = 0;


    init_queue(&process_queue);

    while (completed_processes < n)
    {
        // @See: process* extract_front_of_queue(queue*);
        process* process_next = extract_front_of_queue(&process_queue);

        // If there is no process awaiting execution, then the algorithm waits for one to arrive.
        if (process_next == NULL)
        {
            // This is implemented by moving the time frame one time unit forward...
            elapsed_time++;

            // ... and adding the processes that arrive in that time frame, if any do so.
            while ( next_process_index < n &&   // Prevents memory violation (short-circuit).
                    elapsed_time == P[next_process_index].arrival_time)
            {
                append_to_queue(&process_queue, &P[next_process_index]);
                next_process_index++;
            }
            continue;
        }

        // Loop for traversing the snapshots of the algorithm's execution time.
        for (int remaining_time = q;
            remaining_time > 0 && process_next->burst_time > 0;
            remaining_time--)
        {
            // Executes current process.
            printf("%d\n", process_next->pid);
            // Burst time is reduced.
            process_next->burst_time--;
            // Time frame moves one process execution unit forward.
            elapsed_time++;

            // Checks for arriving processes and updates queue.
            while ( next_process_index < n &&   // Prevents memory violation (short-circuit).
                    elapsed_time == P[next_process_index].arrival_time)
            {
                append_to_queue(&process_queue, &P[next_process_index]);
                next_process_index++;
            }
        }
        if (process_next->burst_time > 0) {
            /* In case the process has not been completed, it
             * is inserted again at the tail of the queue. */
            append_to_queue(&process_queue, process_next);
        }
        else {
            completed_processes++;
        }
    }
    /* Typical memory de-allocation of the queue (even though it
     * should be empty by the time SJF has finished execution) */
    destroy_queue(&process_queue);
}

int main()
{
	/* read in data - DO NOT EDIT (START) */
	int n;
	int quantum;
	scanf("%d", &n);
	scanf("%d", &quantum);
	process *arr;
	arr = malloc(n * sizeof(process));
	for (int i = 0; i < n ; ++i) {
		scanf("%d", &arr[i].pid);
		scanf("%d", &arr[i].arrival_time);
		scanf("%d", &arr[i].burst_time);
	}
	/* read in data - DO NOT EDIT (END) */

    round_robin(arr, n, quantum);

    free(arr);

	return 0; /* DO NOT EDIT THIS LINE */
}
