from math import gcd
from random import randint
from random import seed
from timeit import default_timer as timer


def pollard_rho(n: int) -> tuple[int, int]:
    # Tracks the number of execution steps needed
    steps = 0
    # The given F(X) function
    f = lambda m : (m * m + 1) % n
    x_0 = randint(2, n - 1)
    x = x_0
    y = x_0
    
    while True:
        x = f(x)
        y = f(f(y))
        d = gcd(abs(x - y), n)
        steps += 1
        if 1 < d < n:
            return d, steps


if __name__ == '__main__':
    seed(42)
    
    n = 2 ** 257 - 1

    start = timer()
    f, steps = pollard_rho(n)
    stop = timer()

    print('Found non-trivial factor: %d' % (f))
    print('Elapsed time: %.3f sec' % (stop - start))
    print('Execution steps: %d' % (steps))