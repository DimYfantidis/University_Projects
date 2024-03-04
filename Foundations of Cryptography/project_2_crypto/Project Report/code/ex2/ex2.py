# Fast computation of b^e mod m
def fast_pow_mod_m(b: int, e: int, m: int) -> int:
    res = 1
    while e > 0:
        if e % 2 == 1:
            res = res * b % m
        e = e // 2
        b = (b * b) % m
    return res


if __name__ == '__main__':
    print('5^77 mod 19 = ', fast_pow_mod_m(5, 77, 19))
