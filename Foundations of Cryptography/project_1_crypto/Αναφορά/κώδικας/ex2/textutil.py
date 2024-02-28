ALPHABET = 'ABCDEFGHIJKLMNOPQRSTUVWXYZ'


if __name__ == '__main__':
    with open('text.txt', 'r') as f:
        text = f.read()

    print(text)
    
    res = []
    for ch in text.upper():
        if ch in ALPHABET:
            res.append(ch)
    plain = ''.join(res)
    
    with open('text2.txt', 'w') as f:
        f.write(plain)
    