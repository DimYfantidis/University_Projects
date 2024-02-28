if __name__ == '__main__':
    A = []
    # Fetch all lowercase symbols of the greek alphabet
    with open('greek_alphabet.txt', mode='r', encoding='utf-8') as f:
        A = list(f.readline())
    
    # Length of alphabet (=24)
    n = len(A)

    # Encryption dictionary
    E = dict(zip(A, [A[(i + 3) % n] for i in range(n)]))
    # Decryption dictionary
    D = dict(zip(E.values(), E.keys()))

    print('Enter encrypted message: ', end='')
    encrypted_message = input()
    decrypted_message = ''

    for c in encrypted_message:
        decrypted_message += D[c]
    
    print('Decrypted message: {}'.format(decrypted_message))
