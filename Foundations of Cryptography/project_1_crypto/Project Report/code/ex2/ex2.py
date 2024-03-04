from numsys import IntegerRepresentation
from numpy import linspace
from timeit import default_timer as timer

import concurrent.futures
import multiprocessing


# The latin alphabet
ALPHABET = 'ABCDEFGHIJKLMNOPQRSTUVWXYZ'
# Mapping of (A, B, ..., Z) to (0, 1, ..., 25)
INT_CODE = dict(zip(ALPHABET, [i for i in range(len(ALPHABET))]))
# Mapping of (0, 1, ..., 25) to (A, B, ..., Z)
CHAR_MAP = dict(zip(INT_CODE.values(), INT_CODE.keys()))

# Vigenere function mode parameter
ENCRYPT = True
DECRYPT = False

# Appearance frequency of latin letters in the English language
FREQUENCY = {
    'A': 0.08167,
    'B': 0.01492, 
    'C': 0.02782, 
    'D': 0.04253, 
    'E': 0.12702, 
    'F': 0.02228, 
    'G': 0.02015, 
    'H': 0.06094, 
    'I': 0.06966, 
    'J': 0.00153, 
    'K': 0.00772, 
    'L': 0.04025, 
    'M': 0.02406, 
    'N': 0.06794, 
    'O': 0.07507, 
    'P': 0.01929, 
    'Q': 0.00095, 
    'R': 0.05987, 
    'S': 0.06327, 
    'T': 0.09056, 
    'U': 0.02758, 
    'V': 0.00978, 
    'W': 0.02360, 
    'X': 0.00150, 
    'Y': 0.01974, 
    'Z': 0.00074 
}


# Viginere encryption/decryption function
def vigenere(m: str, k: str, mode: bool) -> str:
    c = ''
    l = len(k)
    n = len(ALPHABET)
    for i in range(len(m)):
        u = INT_CODE[m[i]]
        if mode == ENCRYPT:
            u += INT_CODE[k[i % l]]
        else:
            u -= INT_CODE[k[i % l]]
        c += CHAR_MAP[u % n]
    return c


def index_of_coincidence(cipher: str) -> float:
    n = len(ALPHABET)
    char_appearances = dict(zip(ALPHABET, n * [0]))
    k = len(cipher)
    sum = 0.0

    for c in cipher:
        char_appearances[c] += 1

    for l in ALPHABET:
        ml = char_appearances[l]
        sum += (ml * ml) / (k * k)
    return sum


# Friedman's test for determining the possible length of key
def length_of_key(cipher: str, error: float) -> int:
    r = 2
    a = 0.0665
    k = len(cipher)

    while r < k:
        C = []
        for i in range(r):
            C.append(''.join([cipher[j] for j in range(i, k, r)]))
        for i in range(r):
            if abs(index_of_coincidence(C[i]) - a) < error:
                return r
        r += 1
    return None


# Frequencies of latin characters in text
def text_freq(text: str) -> dict[str, float]:
    n = len(ALPHABET)
    f = dict(zip(ALPHABET, n * [0]))
    for ch in text:
        f[ch] += 1
    return dict(zip(ALPHABET, [f_val / len(text) for f_val in f.values()]))


def stat_test_interval(cipher: str, a: int, b: int, length: int, p_label: str='N/A') -> None:
    # Generates permutations of the english alphabet
    alphabet = ALPHABET
    frequency = FREQUENCY.copy()
    key_generator = IntegerRepresentation(alphabet)
    n = len(alphabet)

    start = timer()
    for x in range(a, b):
        k = key_generator.get(x, pad=length)
        m = vigenere(cipher, k, mode=DECRYPT)
        
        # Letter frequencies of possible deciphered texts
        freq_text = text_freq(m)

        matches = 0
        for ch in alphabet:
            if (abs(frequency[ch] - freq_text[ch]) < 0.01):
                matches += 1
        if matches >= int(n * 0.6):
            print(f'> Process {p_label} found an intelligible text')
            print(f'> Key is {k}')
            print(f'>>> {m}')
    stop = timer()
    print('> Process %2s finished after %8.2f seconds' % (p_label, stop - start))


if __name__ == '__main__':
    n = len(ALPHABET)
    # Isolate the two ciphertexts
    with open('ex2.txt', 'r') as f:
        one = f.readline().removeprefix('[1] ').removesuffix('\n')
        two = f.readline().removeprefix('[2] ').removesuffix('\n')
    # Compute possible lengths of keys in regards to Vigenere's algorithms
    l1 = length_of_key(one, 0.01)
    l2 = length_of_key(two, 0.01)

    print('len(k1) = ', l1)
    print('len(k2) = ', l2)

    with concurrent.futures.ProcessPoolExecutor() as executor:
        v = [int(x) for x in linspace(0, n ** l1, multiprocessing.cpu_count() - 1)]
        p = [executor.submit(stat_test_interval, one, v[i - 1], v[i], l1, str(i)) for i in range(1, len(v))]

        for f in concurrent.futures.as_completed(p):
            f.result()
    