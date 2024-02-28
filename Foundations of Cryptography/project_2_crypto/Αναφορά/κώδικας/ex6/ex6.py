from math import exp
from math import log
from timeit import default_timer as timer

import threading
import concurrent.futures
import multiprocessing


def divisors(N: int) -> list[int]:
    # An integer cannot be divided with a number greater
    # than its half, except itself
    return [x for x in range(1, (N // 2) + 1) if N % x == 0] + [N]


def sigma(m: int) -> int:
    return sum(divisors(m))


def prime_factors(n: int) -> list[tuple[int, int]]:
    if n <= 1:
        return []
    L = []
    while n % 2 == 0:
        L.append(2)
        n = n // 2
    f = 3
    while f * f <= n:
        if n % f == 0:
            L.append(f)
            n = n // f
        else:
            f += 2
    if n > 1:
        L.append(n)
    L_set = list(set(L))
    
    M = []
    for prime in L_set:
        # Prime factors and their multiplicity (exponent)
        M.append((prime, L.count(prime)))
    return M


def sigma2(m: int) -> int:
    F = prime_factors(m)
    prod = 1
    for i in range(len(F)):
        p, a = F[i]
        prod *= ((p ** (a + 1)) - 1) // (p - 1)
    return prod


# Searches for a counter argument in the interval [a, b) 
# or, more precisely, {a, a+1, ..., b-2, b-1}. 
#   Parameters:
#   - a, b: search space bounds
#   - p_label: for identifying the process in a set of processes
#   Returns:
#   - bool: True if a counter argument is found, else false
#   - int: the value of the counter argument for which the inequality doesn't hold
def find_counter_argument_in_interval(
        a: int, b: int, p_label: str = 'N/A', event: threading.Event = None) -> tuple[bool, int]:
    # Return values
    found = False
    counter_arg = -1

    # Pre Computed values (Euler's Constant)
    GAMMA = 0.57721566490153286060651209008240243
    E_GAMMA = exp(GAMMA)
    E_GAMMA_HALF = E_GAMMA / 2

    start = timer()
    for n in range(a, b, 2):
        if event is not None:
            # If the termination event is set, then some process has already 
            # found a counter argument and has returned it. Thus, every process 
            # that is still searching, is killed as their common goal has been 
            # achieved.
            if event.is_set():
                break
        # Left hand side of the inequality
        LHS = sigma2(n) / n
        # 2x Speedup
        lln = log(log(n))
        # Right hand side of the inequality
        RHS = E_GAMMA_HALF * lln + (0.74) / lln

        if LHS >= RHS:
            found = True
            counter_arg = n
            break
    stop = timer()

    # Logs statistics
    print('> Process \"%2s\" finished after %8.2f sec' % (p_label, stop - start))
    return found, counter_arg


if __name__ == '__main__':
    # Multiprocessing power
    print('Available CPU cores:', multiprocessing.cpu_count())

    error_flag = False
    
    # Splits the search process into 10 independent processes in the intervals
    # [v1=3, v2), [v2, v3), ..., [v8, v9), [v9, v10=2^20) where
    # [v1, v2) U [v2, v3) U ... U [v8, v9) U [v9, v10) = [3, 2^20)
    start = timer()
    with multiprocessing.Manager() as manager:
        # If a process finds a counter argument, the other 9 processes will
        # be notified through this variable so that they will all force stop.
        termination_event = manager.Event()
        with concurrent.futures.ProcessPoolExecutor() as executor:
            # v[0] = 3 | v[10] = 2^20
            v = [3, 209717, 335547, 443431, 545261, 629147, 723037, 837889, 943719, 1006633, 1048576]
            # Though slower, this code still works even if cpu processors < 10
            p = [executor.submit(find_counter_argument_in_interval, 
                                v[i - 1], v[i], str(i), termination_event) for i in range(1, len(v))
            ]
            # The loop is halted. It iterates when a value is returned from any process.
            for f in concurrent.futures.as_completed(p):
                # Event listener: listens for the termination 
                # of a process to fetch its result.
                found, n = f.result()
                error_flag = error_flag or found
                if error_flag:
                    # Shuts down all processes in case some haven't started
                    for future in p:
                        future.cancel()
                    # Termination flag for running processes
                    termination_event.set()
                    print('>>> Found counter argument: %d' % (n))
                    break
    stop = timer()

    # Logs results
    if not error_flag:
        print('Program finished successfully')
    else:
        print('Mathematical formula is invalid')
    print('Elapsed time: %8.2f second(s)' % (stop - start))
