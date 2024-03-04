# b1 XOR b2
def xor(b1: str, b2: str) -> str:
    l = len(b1)
    if l != len(b2):
        raise Exception('Illegal XOR arguments')
    for c1, c2 in zip(b1, b2):
        if c1 not in {'0', '1'} or c2 not in {'0', '1'}:
            raise Exception('Illegal XOR arguments')
    n1 = int(b1, 2)
    n2 = int(b2, 2)
    return '{{0:0{}b}}'.format(l).format(n1 ^ n2)


# x_1 XOR x_2 XOR ... XOR x_n
def xor_all(X: list[str]) -> str:
    l = len(X[0])
    result = l * '0'
    for x in X:
        result = xor(result, x)
    return result


# Circular shift (x << a)
def cycle_left(x: str, a: int) -> str:
    if not 0 < a < len(x):
        raise Exception('Illegal left circular shift arguments')
    for digit in x:
        if digit not in {'0', '1'}:
            raise Exception('Illegal left circular shift arguments')
    msbits = x[:a]
    lsbits = x[a:]
    return lsbits + msbits


# m XOR (m << 6) XOR (m << 10)
def encode(m: str) -> str:
    if (len(m) != 16):
        raise Exception('Illegal encode() arguemnt')
    return xor_all([m, cycle_left(m, 6), cycle_left(m, 10)])


# (c << 4) XOR (c << 2) XOR c XOR (c << 14) XOR (c << 12)
def decode(c: str) -> str:
    if (len(c) != 16):
        raise Exception('Illegal encode() arguemnt')
    return xor_all([cycle_left(c, 4), 
                    cycle_left(c, 2), 
                    c, 
                    cycle_left(c, 14), 
                    cycle_left(c, 12)])


if __name__ == '__main__':
    # All 16-digit binary sequences
    bin16 = ['{0:016b}'.format(x) for x in range(2 ** 16)]

    # Changes to True if the solution is wrong 
    error_flag = False

    for m in bin16:
        # If the solution is right, then D(E(m)) = m, for every m in {0, 1}^16
        if decode(encode(m)) != m:
            error_flag = True
            break

    if not error_flag:
        print('Problem solved successfully (decoding function is valid)')
    else:
        print('Failed to solve the problem (decoding function is invalid)')
