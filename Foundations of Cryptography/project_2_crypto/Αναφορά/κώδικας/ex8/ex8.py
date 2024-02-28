import math
import random
import concurrent.futures
import sys
from timeit import default_timer as timer


# Fast computation of b^e mod m
def fast_pow_mod_m(b: int, e: int, m: int) -> int:
    res = 1
    while e > 0:
        if e % 2 == 1:
            res = res * b % m
        e = e // 2
        b = (b * b) % m
    return res


# Primality test of k steps
def fermat_test(n: int) -> tuple[int, int | None, float]:
    start = timer()
    a = random.randint(2, n - 1)
    if math.gcd(a, n) > 1:
        return (n, None, timer() - start)
    if fast_pow_mod_m(a, n - 1, n) == 1:
        return (n, a, timer() - start)
    return (n, None, timer() - start)


if __name__ == '__main__':
    n1 = 835335 * (2 ** 39014) + 1
    n2 = 835335 * (2 ** 39014) - 1
    sys.set_int_max_str_digits(50000)

    
    with concurrent.futures.ProcessPoolExecutor() as executor:
        # Checks the two numbers concurrently
        p = [
            executor.submit(fermat_test, n1), 
            executor.submit(fermat_test, n2)
        ]
        for f in concurrent.futures.as_completed(p):
            n, a, time = f.result()

            s = 'n1' if n == n1 else 'n2'
            with open(s + '.txt', 'w') as f:
                if a is None:
                    f.write('%s is composite (elapsed: %.3lf sec)\n' % (s, time))
                    print('%s is composite (elapsed: %.3lf sec)' % (s, time))
                else:
                    exp = math.log2(a)
                    f.write('%s is probable prime with respect %d (elapsed: %.3lf sec)\n' % (s, a, time))
                    print('%s is probable prime with respect 2^%lf (elapsed: %.3lf sec)' % (s, exp, time))
