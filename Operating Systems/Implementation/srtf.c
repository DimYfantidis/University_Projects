#include <stdio.h>
#include <stdlib.h>

// FILE IS ALMOST IDENTICAL TO sjf.c

typedef struct
{
	int pid;
	int arrival_time;
	int burst_time;
} process;

// Process comparator function with burst time as key value.
int compare_process_burst_time(process* p1, process* p2)
{
    if (p1->burst_time < p2->burst_time) {
        return -1;
    }
    if (p1->burst_time > p2->burst_time) {
        return 1;
    }
    return 0;
}


// ----------- Double-ended Process List Implementation (START) -----------

// Building block of the following doubly linked list structure.
struct list_node
{
    process* data;              // Reference to a process, the user has obligation to manage its memory
    struct list_node* next;     // The following list element
    struct list_node* previous; // The previous list element
};
typedef struct list_node list_node;

// Doubly-linked list with head and tail nodes
typedef struct
{
    list_node* head;
    list_node* tail;
    int size;
} list;

/*  Side node: the methods for the manipulation of a list take its reference
 *  as the first parameter abbreviated as "THIS", in order to simulate an
 *  object-oriented manner of programming. */

// Initializes list's fields (Constructor)
void init_list(list* THIS)
{
    THIS->head = NULL;
    THIS->tail = NULL;
    THIS->size = 0;
}

// Deallocates list's nodes from memory (Destructor)
void destroy_list(list* THIS)
{
    list_node* temp;

    for (list_node* iterator = THIS->head; iterator != NULL; iterator = temp)
    {
        temp = iterator->next;
        free(iterator);

        /*  The "data" field of a list node is supposed to be a reference to a
         *  statically allocated process and not the return value of a malloc().
         *  Thus, it is not deallocated with free. */
    }
}

// Adds a new process at the end of the list.
void append_to_list(list* THIS, process* p)
{
    // Stores the current tail node
    list_node* temp = THIS->tail;

    // The tail becomes a new node with the given process's data.
    THIS->tail = (list_node*)malloc(sizeof(list_node));
    THIS->tail->data = p;
    THIS->tail->next = NULL;
    // The old tail becomes the new tail's previous node.
    THIS->tail->previous = temp;

    THIS->size++;

    if (temp != NULL) {
        // The new tail becomes the old tail's next node.
        temp->next = THIS->tail;
    }
    else {
        // Special case: The list was empty before the insertion.
        THIS->head = THIS->tail;
    }
}

/*      Searches linearly for the process with the minimum key value in the list.
 *  The selected process is removed from the list and returned as the result.
 *
 *      The aforementioned key value is determined by the comparator passed as the
 *  second parameter. */
process* extract_min(list* THIS, int compare_processes(process*, process*))
{
    list_node* min_node;    // Keeps track of the node with minimum element.
    process* min;           // The minimum element.

    // Returns NULL if empty.
    if (THIS->size == 0) {
        return NULL;
    }
    // Special case for list.head == list.tail
    if (THIS->size == 1)
    {
        min_node = THIS->head;

        // Head and tail are reset.
        THIS->head = NULL;
        THIS->tail = NULL;

        min = min_node->data;   // Stores the process for the return value.
        free(min_node);     // Removes the minimum element from the list.

        THIS->size--;   // size = 0

        return min;
    }
    min_node = THIS->head;

    // Iterates through the list structure to locate the minimum element.
    for (list_node* iterator = THIS->head->next;
         iterator != NULL;
         iterator = iterator->next)
    {
        // Process comparator function (declared in method's signature).
        if (compare_processes(min_node->data, iterator->data) > 0) {
            min_node = iterator;
        }
        // In the case of SRTF, the comparator examines the relation between p1's and p2's burst times.
    }
    min = min_node->data;   // Return value

    if (min_node != THIS->head) {
        min_node->previous->next = min_node->next;
    }
    else {
        // Special case: list.head is removed
        THIS->head = THIS->head->next;
        THIS->head->previous = NULL;
    }
    if (min_node != THIS->tail) {
        min_node->next->previous = min_node->previous;
    }
    else {
        // Special case: list.tail is removed
        THIS->tail->next = NULL;
        THIS->tail = THIS->tail->previous;
    }
    free(min_node);

    THIS->size--;

    return min;
}

// ----------- Double-ended Process List Implementation (END) -----------


//  Implementation of SRTF scheduling algorithm (preemptive)
// 		@param P: array of processes for scheduling
//		@param n: array size
void shortest_remaining_time_first(process* P, int n)
{
    /* Double-ended list data structure for keeping track of the processes that
     * have arrived and are awaiting execution. By SJF's definition the process
     * with the minimum burst time will be selected from the list.
     *
     * It should normally be a priority queue but taking the time to implement
     * such a structure would not be worth it, given that this isn't the purpose
     * of this project. A list is much easier to implement. */
    list process_queue;

    /* Keeps track of elapsed time in process execution units. Used for checking
     * the arrival of processes and inserting them to the queue.
     *
     * Variable is initialized to -1 (not 0) to prevent an infinite loop,
     * since the expression "P[0].arrival_time = 0" is allowed (P[0]'s insertion
     * in the process queue is not implemented as a special case).*/
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


    init_list(&process_queue);

    while (completed_processes < n)
    {
        // @See: process* extract_min(list* THIS, int cmp_func(process*, process*));
        // @See: int compare_process_burst_time(process* p1, process* p2);
        process* process_next = extract_min(&process_queue, compare_process_burst_time);

        // If there is no process awaiting execution, then the algorithm waits for one to arrive.
        if (process_next == NULL)
        {
            // This is implemented by moving the time frame one time unit forward...
            elapsed_time++;

            // ... and adding the processes that arrive in that time frame, if any do so.
            while ( next_process_index < n && // Prevents memory violation (short-circuit).
                    elapsed_time == P[next_process_index].arrival_time)
            {
                append_to_list(&process_queue, &P[next_process_index]);
                next_process_index++;
            }
            continue;
        }

        // Executes current process.
        printf("%d\n", process_next->pid);
        // Burst time is reduced.
        process_next->burst_time--;
        // Time frame moves one process execution unit forward.
        elapsed_time++;

        if (process_next->burst_time == 0) {
            completed_processes++;
        }
        else {
            /* In case the process has not finished its execution,
             * it must be re-inserted to the queue since extract_min()
             * removes it from the latter. */
            append_to_list(&process_queue, process_next);

            /* Perpetual reallocation of memory for list nodes could be avoided
             * if a get_min(list *) and remove_process(list *, process *) were
             * implemented. That way, a process would not be erased until its
             * execution had come to its end. */
        }

        // Checks for arriving processes and updates queue.
        while ( next_process_index < n && // Prevents memory violation (short-circuit).
                elapsed_time == P[next_process_index].arrival_time)
        {
            append_to_list(&process_queue, &P[next_process_index]);
            next_process_index++;
        }
    }
    /* Typical memory de-allocation of the list (even though it
     * should be empty by the time SRTF has finished execution). */
    destroy_list(&process_queue);
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

    shortest_remaining_time_first(arr, n);

    free(arr);

	return 0; /* DO NOT EDIT THIS LINE */
}
