from math import isqrt
from random import randint


# Returns true if n is prime
def is_prime(n: int) -> bool:
    # 2 is prime
    if n == 2:
        return True
    # Every other even integer is not prime
    if n % 2 == 0 or n == 1:
        return False
    # Check for factors in range sqrt(n)
    for x in range(3, isqrt(n) + 1, 2):
        if n % x == 0:
            return False
    return True


# Returns a list with all the prime factors of n.
# Elements can be repeating, e.g: 24 -> [2, 2, 2, 3]
def prime_factors(n :int) -> list[int]:
    if is_prime(n):
        return [n]
    
    p_factors = []
    while n % 2 == 0:
        p_factors.append(2)
        n = n // 2

    x = 3
    finished = (n == 1)

    while not finished:
        if is_prime(x):
            while n % x == 0 and not finished:
                p_factors.append(x)
                n = n // x
                if n == 1:
                    finished = True
        x += 2
    return p_factors


# Returns a list with n's postive divisors
def divisors(n: int) -> list[int]:
    n = abs(n)
    return [x for x in range(1, n//2 + 1) if n % x == 0] + [n]


# Möbius function
def mu(n: int) -> int:
    p = prime_factors(n)

    if n == 1:
        return 1
    elif len(p) != 0:
        return (-1) ** len(p)
    return 0


# Returns the number of reduced(?) polynomials of all n class polynomials in GF(2)
def N_2(n: int) -> int:
    return int((1 / n) * sum([mu(d) * 2**(n//d) for d in divisors(n)]))


if __name__ == '__main__':
    for i in range(10):
        x = randint(1, 100)
        print(f"[{x}] info:")
        print(f'-- divisors: {divisors(x)}')
        print(f'-- prime factorization: {prime_factors(x)}')
        print(f'-- N_2({x}) = {N_2(x)}\r\n')
