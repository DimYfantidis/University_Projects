#include <stdio.h>
#include <stdlib.h>


typedef struct 
{
	int pid;
	int arrival_time;
	int burst_time;
}
process;


//  Implementation of FCFS scheduling algorithm
// 		@param P: array of processes for scheduling
//		@param n: array size
void first_come_first_served(process* P, int n)
{
	/*
	*		It is considered that each process, given from the standard
	*	input, arrives in the system after the previous one and before
	*	the next one. That means:
	*
	*		P[i].arrival_time < P[i+1].arrival_time
	*
	*	which discards the necessity of searching for the process with the
	 *	minimum arrival time, since the array is already sorted.
	*
	*		With all that in mind, all we need is a counter "i" that simply 
	*	increments by one in order to simulate FCFS scheduling, as P[i] has 
	*	arrived before P[i+1] and must, thus, be served first.
	*/
	for (int i = 0; i < n; ++i)
	{
		// Auxiliary inner loop for traversing the snapshots of the algorithm's execution time
		while (P[i].burst_time > 0)
		{
            // Executes current process.
			printf("%d\n", P[i].pid);
            // Burst time is reduced.
            P[i].burst_time--;
		}
	}
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

	first_come_first_served(arr, n);

    free(arr);

	return 0; /* DO NOT EDIT THIS LINE */
}
