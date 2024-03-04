import random


# All 5-digit binary sequences
bin5 = ['{0:05b}'.format(x) for x in range(2 ** 5)]
# Symbols given by the exercise (in the given order)
symbols = list('ABCDEFGHIJKLMNOPQRSTUVWXYZ.!?()-')
# binary encoding of each symbol as given by table 2
char_to_bits = dict(zip(symbols, bin5))
# binary encoding of each symbol as given by table 2
bits_to_char = dict(zip(bin5, symbols))


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


# Converts text message and key to binary.
# Computes message XOR key and returns it in text form
def encrypt_otp(m: str, k: str) -> str:
    l = len(m)
    if l != len(k):
        raise Exception('Illegal OTP arguments')
    
    m_bits = ''.join([char_to_bits[c] for c in m])
    k_bits = ''.join([char_to_bits[c] for c in k])

    cipher_bits = xor(m_bits, k_bits)
    cipher = ''

    for i in range(l):
        s = cipher_bits[5 * i: 5 * (i + 1)]
        cipher += bits_to_char[s]
    return cipher


# Encryption and decryption algorithm are the same in OTP
def decrypt_otp(c: str, k: str) -> str:
    return encrypt_otp(c, k)


if __name__ == '__main__':
    with open('sample_text.txt', 'r') as f:
        text = f.read()

    # Let's ignore the technical difficulty that the key is not truly random as OTP demands.
    random.seed(42)
    # The key must have the same length as the text
    key = ''.join([random.choice(symbols) for c in text])

    cipher = encrypt_otp(text, key)

    print('\r\n>>> Random key:')
    print(key)

    print('\r\n>>> Cipher text:')
    print(cipher)

    print('\r\n>>> Deciphered text:')
    print(decrypt_otp(cipher, key))
