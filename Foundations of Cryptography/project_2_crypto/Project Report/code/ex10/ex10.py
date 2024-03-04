from random import randint
from math import gcd
from math import isqrt
from math import log2
from math import log10
from numpy import sqrt
from numpy import cbrt
from timeit import default_timer as timer
from multiprocessing import cpu_count
import concurrent.futures


def lehman(n: int, time_threshold_sec: float) -> tuple[int, int | None, bool]:
    start = timer()
    for k in range(1, int(cbrt(float(n))) + 1):
        stop = timer()
        if stop - start > time_threshold_sec:
            return n, None, True

        _4kn = 4 * k * n
        root_4kn  = sqrt(float(_4kn))
        l = int(root_4kn) + 1
        r = int(root_4kn + cbrt(sqrt(float(n))) / (4 * sqrt(float(k))))
        for a in range(l, r + 1):
            stop = timer()
            if stop - start > time_threshold_sec:
                return n, None, True

            c = a * a - _4kn
            if c >= 0:
                b = isqrt(c)
                if b * b == c:
                    return n, gcd(a - b, n), False
    return n, None, False


if __name__ == '__main__':
    # Tsting sample
    MAX_NUMBERS_TESTED = 1000
    # Ignore
    _DIG = int(log10(MAX_NUMBERS_TESTED)) + 1
    # Number of binary digits (ignore)
    BIT_COUNT = 100
    # Number of deicmal digits (ignore)
    DEC_COUNT = int(BIT_COUNT / log2(10)) + 1
    # Number of processes
    PROCCESS_COUNT = cpu_count() - 2

    with open('results.txt', 'a') as logger:
        with concurrent.futures.ProcessPoolExecutor() as executor:
            i = 0
            success_counter = 0
            while i < MAX_NUMBERS_TESTED:
                N = [randint(2 ** (BIT_COUNT - 1), 2 ** BIT_COUNT - 1) for _ in range(PROCCESS_COUNT)]
                if MAX_NUMBERS_TESTED - i >= PROCCESS_COUNT:
                    p = [executor.submit(lehman, n, 10.0) for n in N]
                else:
                    p = [executor.submit(lehman, N[j], 10.0) for j in range(MAX_NUMBERS_TESTED - i)]
                
                for f in concurrent.futures.as_completed(p):
                    n, factor, timeout = f.result()
                    if timeout or factor in {1, n, None}:
                        logger.write((f"[%0{_DIG}d] %{DEC_COUNT}d failed\n") % (i + 1, n))
                        print((f"[%0{_DIG}d] %{DEC_COUNT}d failed") % (i + 1, n))
                    else:
                        success_counter += 1
                        logger.write((f"[%0{_DIG}d] %{DEC_COUNT}d factor found: %d\n") % (i + 1, n, factor))
                        print((f"[%0{_DIG}d] %{DEC_COUNT}d factor found: %d") % (i + 1, n, factor))
                    i += 1
            success_percentage = 100 * (success_counter / MAX_NUMBERS_TESTED)

            logger.write('Produced a factor for %7.3f%% of integers (%d / %d)\n\n' % 
                (success_percentage, success_counter, MAX_NUMBERS_TESTED))
            
            print('Produced a factor for %7.3f%% of integers (%d / %d)' % 
                (success_percentage, success_counter, MAX_NUMBERS_TESTED)
            )
