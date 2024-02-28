# Trial Division algorithm
def prime_factors(n: int) -> list[int]:
    factors = []
    if n <= 1:
        return []
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
    return factors


if __name__ == '__main__':
    n1 = 2 ** 62 - 1
    n2 = 2 ** 102 - 1

    print('2^62 - 1 = ', prime_factors(n1))
    print('2^102 - 1 = ', prime_factors(n2))
