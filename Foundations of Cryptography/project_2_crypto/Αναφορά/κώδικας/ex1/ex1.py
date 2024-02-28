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
    g = 3
    p = 101
    a = 77
    b = 91
    common_key = fast_pow_mod_m(g, a * b, p)

    print(f'Common key is: {g}^{a * b} mod {p} = ', common_key)
