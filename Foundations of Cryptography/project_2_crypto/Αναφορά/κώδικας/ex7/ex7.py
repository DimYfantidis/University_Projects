import math
import random


def unique_elements(l: list) -> list:
    lu = []
    for x in l:
        if x not in lu:
            lu.append(x)
    return lu


# Trial Division algorithm
def prime_factorization(n: int) -> tuple[list[int], list[int]]:
    factors = []
    if n <= 1:
        return [], []
    while n % 2 == 0:
        factors.append(2)
        n = n // 2
    f = 3
    while f * f <= n:
        if n % f == 0:
            factors.append(f)
            n = n // f
        else:
            f += 2
    if n != 1:
        factors.append(n)

    p = unique_elements(factors)
    a = [factors.count(prime) for prime in p]

    return p, a


def is_carmichael(n: int) -> bool:
    p, a = prime_factorization(n)
    print(p)
    for exp in a:
        if exp != 1:
            return False
    for prime in p:
        if (n - 1) % (prime - 1) != 0:
            return False
    return True


if __name__ == '__main__':
    n1 = 9999109081
    n2 = 6553130926752006031481761

    print(is_carmichael(n1))
    print(is_carmichael(n2))
