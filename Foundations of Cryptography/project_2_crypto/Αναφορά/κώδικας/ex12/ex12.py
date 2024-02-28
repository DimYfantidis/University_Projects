# The ciphertext
C = [
    3203, 909, 3143, 5255, 5343, 
    3203, 909, 9958, 5278, 5343, 
    9958, 5278, 4674, 909, 9958, 
    792, 909, 4132, 3143, 9958, 
    3203, 5343, 792, 3143, 4443
]


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


# Calculates the totient of n
def phi(n: int) -> int:
    # Python set: removes duplicate 
    # appearances of n's prime factors
    p = list(set(prime_factors(n)))
    # Prevents conversion of data type from 'int' to 'float'
    Phi = n
    for i in range(len(p)):
        Phi_next = Phi - Phi // p[i]
        Phi = Phi_next
    return Phi


# Brute force calculation of the secret key
def get_private_key(pk: tuple[int, int]) -> tuple[int, int]:
    e, N = pk
    t = phi(N)
    d = 0
    while (e * d) % t != 1:
        d += 1
    return d, N


def RSA(sk: tuple[int, int], cipher: list[int]) -> list[int]:
    d, N = sk
    message = []
    for c in cipher:
        message.append((c ** d) % N)
    return message


if __name__ == '__main__':
    # (e, N) = (19, 11413)
    pk = (19, 11413)
    sk = get_private_key(pk)
    
    print('Public key:  (e, N) = (%d, %d)' % pk)
    print('Private key: (d, N) = (%d, %d)' % sk)

    message_ascii = RSA(sk, C)
    message_text = ''.join([chr(byte) for byte in message_ascii])

    print('Decrypted bytes:', message_ascii)
    print('Decrypted message: \"%s\"' % message_text)
