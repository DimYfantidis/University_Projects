from collections.abc import Callable


# The internal lookup table used for the given S-box
S_i = [
    ['0000', '0010', '0011', '0111', '1001', '1100', '1111', '0111', '0110', '1111', '1111', '0001', '0111', '0011', '0001', '0000'],
    ['0001', '0101', '0110', '1101', '0100', '0001', '0101', '1011', '0111', '1000', '0111', '0001', '0001', '0011', '0010', '1101'],
    ['0101', '0011', '0101', '1100', '1011', '0001', '0001', '0101', '1101', '0000', '1111', '0111', '0010', '0010', '1101', '0000'],
    ['0011', '1100', '0011', '1011', '0010', '0010', '0010', '0100', '0110', '0101', '0101', '0000', '0100', '0011', '0001', '0000']
]


# S-box, S:{0, 1}^6 -> {0, 1}^4
def S(x: str) -> str:
    # Outer bits give the row (base 2 string to int constructor)
    R = int(x[0] + x[5], 2)
    # Inner bits give the column (base 2 string to int constructor)
    C = int(x[1] + x[2] + x[3] + x[4], 2)
    return S_i[R][C]


# x xor y
def xor(x: str, y: str) -> str:
    # String arguments are expected to be binary sequences
    for c1, c2 in zip(x, y):
        if c1 not in {'0', '1'} or c2 not in {'0', '1'}:
            raise Exception('Binary sequences are expected as input')

    # Binary sequences must have the same length
    l = len(x)
    if l != len(y):
        raise Exception('Binary sequences must be the same length')

    # Convert to int
    n1 = int(x, 2)
    n2 = int(y, 2)

    # Returns the bitwise xor of n1 and n2 and formats them as
    # a binary sequence of the same (fixed) length as the inputs.
    return '{{0:0{}b}}'.format(l).format(n1 ^ n2)


# Computes the differential uniformity of a given S-box :
#   S_b: {0, 1}^n -> {0, 1}^m
def differential_uniformity(S_b: Callable[[str], str], n: int) -> int:
    # m doesn't need to be passed as a parameter
    m = len(S_b(n * '0'))

    binary_n = '{{0:0{}b}}'.format(n)  # Format string for n-bit sequences
    binary_m = '{{0:0{}b}}'.format(m)  # Format string for m-bit sequences

    F2_n = [binary_n.format(i) for i in range(2 ** n)]  # Set {0, 1}^n
    F2_m = [binary_m.format(i) for i in range(2 ** m)]  # Set {0, 1}^m

    # Cardinalities of sets of all n-bit sequences, z, where z meets the 
    # condition S(z xor x) xor S(z) == y for each m-bit sequence y
    cardinalities = []

    # Iterates all n-bit sequences, excluding the binary representation of 0
    for x in F2_n:
        # Excludes {0}
        if int(x, 2) == 0:
            continue

        for y in F2_m:
            Z = [z for z in F2_n if xor(S_b(xor(z, x)), S_b(z)) == y]
            cardinalities.append(len(Z))

    return max(cardinalities)


if __name__ == '__main__':
    Diff_S = differential_uniformity(S, 6)
    print('Diff(S) = {}'.format(Diff_S))
