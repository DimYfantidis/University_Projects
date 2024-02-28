from math import floor
from fractions import Fraction


# The RSA-encrypted message 
C = [
    47406263192693509,51065178201172223,30260565235128704,82385963334404268,
    8169156663927929,47406263192693509,178275977336696442,134434295894803806,
    112111571835512307,119391151761050882,30260565235128704,82385963334404268,
    134434295894803806,47406263192693509,45815320972560202,174632229312041248,
    30260565235128704,47406263192693509,119391151761050882,57208077766585306,
    134434295894803806,47406263192693509,119391151761050882,47406263192693509,
    112111571835512307,52882851026072507,119391151761050882,57208077766585306,
    119391151761050882,112111571835512307,8169156663927929,134434295894803806,
    57208077766585306,47406263192693509,185582105275050932,174632229312041248,
    134434295894803806,82385963334404268,172565386393443624,106356501893546401,
    8169156663927929,47406263192693509,10361059720610816,134434295894803806,
    119391151761050882,172565386393443624,47406263192693509,8169156663927929,
    52882851026072507,119391151761050882,8169156663927929,47406263192693509,
    45815320972560202,174632229312041248,30260565235128704,47406263192693509,
    52882851026072507,119391151761050882,111523408212481879,134434295894803806,
    47406263192693509,112111571835512307,52882851026072507,119391151761050882,
    57208077766585306,119391151761050882,112111571835512307,8169156663927929,
    134434295894803806,57208077766585306
]


def fast_pow_mod_m(b: int, e: int, m: int) -> int:
    res = 1
    while e > 0:
        if e % 2 == 1:
            res = res * b % m
        e = e // 2
        b = (b * b) % m
    return res


# Returns the floating point representation of x = [a0; a1, a2, ..., an]
def continued_fraction_expansion(coefficients: list[int]) -> float:
    a = coefficients[::-1]
    d = a[0]
    for i in range(1, len(a)):
        d = a[i] + 1 / d
    return d


# Returns the coefficients of x = [a0; a1, a2, ..., an]
def get_coefficients(x: float, precision: int) -> list[int]:
    coefficients = []
    for _ in range(precision + 1):
        coefficients.append(int(x))
        x = x - floor(x)
        x = 1 / x
    return coefficients


# Wiener's attack on RSA
def wiener(pk: tuple[int, int]) -> int | None:
    e, N = pk
    a = get_coefficients(e / N, 40)
    n = len(a)

    # print(a)

    frac = []
    for i in range(1, n):
        x_i = continued_fraction_expansion([a[j] for j in range(i)])
        frac.append(x_i)
    
    for i in range(1, n):
        (N_i, D_i) = Fraction(frac[i - 1]).limit_denominator(100000).as_integer_ratio()
        # print('[%d]: (%d, %d) = %.15f' % (i, N_i, D_i, frac[i - 1]))
        if fast_pow_mod_m(2, e * D_i, N) == 2:
            return D_i
    return None


def RSA(sk: tuple[int, int], cipher: list[int]) -> list[int]:
    d, N = sk
    message = []
    for c in cipher:
        message.append((c ** d) % N)
    return message


if __name__ == '__main__':
    e = 50736902528669041
    N = 194749497518847283

    pk = (e, N)
    d = wiener(pk)

    sk = (d, N)
    print('Probable private key: (%d, %d)' % sk)

    M = RSA(sk, C)

    print('Decypred message: %s' % (''.join([chr(m) for m in M])))
