# All 5-digit binary sequences
bin5 = ['{0:05b}'.format(x) for x in range(2 ** 5)]
# Symbols given by the exercise (in the given order)
symbols = list('ABCDEFGHIJKLMNOPQRSTUVWXYZ.!?()-')
# binary encoding of each symbol as given by table 2
char_to_bytes = dict(zip(symbols, bin5))
# binary encoding of each symbol as given by table 2
bytes_to_char = dict(zip(bin5, symbols))

BIT_LENGTH = 5


# b1 XOR b2
def xor(b1: str, b2: str) -> str:
    l = len(b1)
    if l != len(b2):
        raise Exception('Illegal XOR arguments')
    result = ''
    for c1, c2 in zip(b1, b2):
        if c1 not in {'0', '1'} or c2 not in {'0', '1'}:
            raise Exception('Illegal XOR arguments')
        result += '1' if c1 != c2 else '0'
    return result


#   Classic RC4, which requires 8-bit string encoding cannot be implemented 
# as symbols are encoded to 5-bit strings.
#   Thus an identical version of RC4 is implemented with S = [0, 1, ..., 31], 
# indices (i and j) cycling every 32 iterations, etc.
def rc4(message: str, key: str) -> str:
    # Initialization
    S = ['{{0:0{}b}}'.format(BIT_LENGTH).format(x) for x in range(2 ** BIT_LENGTH)]
    # Encode the message from characters to 5-bit binary strings
    key_bytes = [char_to_bytes[c] for c in key]
    key_len = len(key_bytes)
    T = [key_bytes[i % key_len] for i in range(2 ** BIT_LENGTH)]

    # Initial permutations of S
    j = 0
    for i in range(2 ** BIT_LENGTH):
        j = (j + int(S[i], 2) + int(T[i], 2)) % (2 ** BIT_LENGTH)
        # Swap S[i] and S[j]
        S[i], S[j] = S[j], S[i]

    cipher_bytes = []
    
    # Stream Generation
    (i, j) = (0, 0)
    for c in message:
        i = (i + 1) % (2 ** BIT_LENGTH)
        j = (j + int(S[i], 2)) % (2 ** BIT_LENGTH)
        S[i], S[j] = S[j], S[i]
        t = (int(S[i], 2) + int(S[j], 2)) % (2 ** BIT_LENGTH)
        k = S[t]
        cipher_bytes.append(xor(char_to_bytes[c], k))

    # Decode the result from 5-bit binary strings to characters
    cipher = ''.join([bytes_to_char[b] for b in cipher_bytes])
    return cipher


if __name__ == '__main__':
    m = "MISTAKESAREASSERIOUSASTHERESULTSTHEYCAUSE"
    k = "HOUSE"
    c = rc4(m, k)

    # Encryption and decryption function is the same in RC4
    print('Encrypted message:', c)
    print('Decrypted message:', rc4(c, k))
