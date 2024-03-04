def fast_pow_mod_m(b: int, e: int, m: int) -> int:
    res = 1
    while e > 0:
        if e % 2 == 1:
            res = res * b % m
        e = e // 2
        b = (b * b) % m
    return res


if __name__ == '__main__':
    m = 3
    s = 301
    N, e = (899, 839)

    a = fast_pow_mod_m(s, e, N)

    if a == m:
        print('a = %d (Accepted)' % (a))
    else:
        print('a = %d (Rejected)' % (a))
