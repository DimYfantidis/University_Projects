from Crypto.Util import Padding
from Crypto.Cipher import AES
import random


# Utility dictionary for hex_to_bin()
conversion_dict = {
    '0': '0000', '1': '0001', '2': '0010', '3': '0011', 
    '4': '0100', '5': '0101', '6': '0110', '7': '0111', 
    '8': '1000', '9': '1001', 'a': '1010', 'b': '1011', 
    'c': '1100', 'd': '1101', 'e': '1110', 'f': '1111'
}

# Converts hexadecimal string to binary string
def hex_to_bin(hex: str) -> str:
    bin = ''
    for digit in hex:
        bin += conversion_dict[digit]
    return bin


# x xor y
def xor(x: str, y: str) -> str:
    if len(x) != len(y):
        # Binary sequences must have the same length
        raise Exception('Binary sequences must be the same length')
    result = ''
    for c1, c2 in zip(x, y):
        # String arguments are expected to be binary sequences
        if c1 not in {'0', '1'} or c2 not in {'0', '1'}:
            raise Exception('Binary sequences are expected as input')
        result += '1' if c1 != c2 else '0'
    return result


# Randomly selects a single byte of m and flips one randomly 
# selected bit. This is achieved by XOR-ing the byte with a 
# random power of 2 in range [1, 32], as powers of 2 contain 
# a single ace (10000000 is forbidden as there are troubles 
# with encoding to ASCII values >127 in python). 
def change_random_bit(m: bytes) -> bytes:
    M = list(m)
    i = random.randint(0, len(M) - 1)
    bit = random.choice([2 ** x for x in range(7)])
    M[i] = M[i] ^ bit
    return ''.join([chr(c) for c in M]).encode('utf-8')


if __name__ == '__main__':
    # Pairs of the original messages and their corresponding single-bit-flipped versions
    with open('messages.txt', 'rb') as f:
        original_messages = f.readlines()
    edited_messages = [change_random_bit(m) for m in original_messages]

    key = b'Sixteen byte key'

    BLOCK_SIZE = len(key)
    TOTAL_MESS_BITS = 8 * 2 * BLOCK_SIZE
    TOTAL_MESSAGES  = len(original_messages)

    aes_ecb = AES.new(key, AES.MODE_ECB)
    aes_cbc = AES.new(key, AES.MODE_CBC)

    # Pairs of the ciphertexts of the original messages and their corresponding 
    # edited versions using AES encryption in ECB mode
    ciphertext_ecb = zip(
        [aes_ecb.encrypt(Padding.pad(m1, 2 * BLOCK_SIZE)) for m1 in original_messages], 
        [aes_ecb.encrypt(Padding.pad(m2, 2 * BLOCK_SIZE)) for m2 in edited_messages]
    )
    # Same but in CBC mode
    ciphertext_cbc = zip(
        [aes_cbc.encrypt(Padding.pad(m1, 2 * BLOCK_SIZE)) for m1 in original_messages], 
        [aes_cbc.encrypt(Padding.pad(m2, 2 * BLOCK_SIZE)) for m2 in edited_messages]
    )

    # Logs the statistic test of AES's avalanche effect in ECB mode
    with open('ecb_mode_stats.txt', 'w') as f:
        f.write('Statistics for AES encryption in ECB mode:\n')
        avg_per = .0
        # Each pair of ciphertexts (c1, c2) is the encryption of 
        # (m1, m2) where m1 and m2 differ in only one bit.
        for i, (c1, c2) in enumerate(ciphertext_ecb):
            bin_cipher1 = hex_to_bin(c1.hex())
            bin_cipher2 = hex_to_bin(c2.hex())
            
            # Number of bits that differ in c1 and c2.
            bits_diff = xor(bin_cipher1, bin_cipher2).count('1')
            percentage = 100 * bits_diff / TOTAL_MESS_BITS
            avg_per += percentage
            
            # Logs the percentage of affected bits.
            f.write('Message {:2d}: {}/256 bits flipped in ciphertext ({:.5f}%)\n'
                    .format(i + 1, bits_diff, percentage))
        avg_per /= TOTAL_MESSAGES

        # Logs the average percentage of affected bits.
        f.write(f'\nAverage bits flipped: {avg_per}%\n')
        print(f'Average bits flipped in AES ciphertext in ECB mode: {avg_per}%')
    
    # Logs the statistic test of AES's avalanche effect in CBC mode
    with open('cbc_mode_stats.txt', 'w') as f:
        f.write('Statistics for AES encryption in CBC mode:\n')
        avg_per = .0
        
        # Each pair of ciphertexts (c1, c2) is the encryption of 
        # (m1, m2) where m1 and m2 differ in only one bit.
        for i, (c1, c2) in enumerate(ciphertext_cbc):
            bin_cipher1 = hex_to_bin(c1.hex())
            bin_cipher2 = hex_to_bin(c2.hex())
            # Number of bits that differ in c1 and c2.
            bits_diff = xor(bin_cipher1, bin_cipher2).count('1')
            percentage = 100 * bits_diff / TOTAL_MESS_BITS
            avg_per += percentage

            # Logs the percentage of affected bits.
            f.write('Message {:2d}: {}/256 bits flipped in ciphertext ({:.5f}%)\n'
                    .format(i + 1, bits_diff, percentage))
        avg_per /= TOTAL_MESSAGES

        # Logs the average percentage of affected bits.
        f.write(f'\nAverage bits flipped: {avg_per}%\n')
        print(f'Average bits flipped in AES ciphertext in CBC mode: {avg_per}%')
