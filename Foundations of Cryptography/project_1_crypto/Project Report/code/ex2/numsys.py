class IntegerRepresentation:
    def __init__(self, symbols: str):
        self.symbols = symbols
        self.int_code = dict(zip(symbols, [i for i in range(len(symbols))]))
        self.char_map = dict(zip(self.int_code.values(), self.int_code.keys()))
        self.l = len(self.symbols)


    def get(self, n: int, pad: int=None) -> str:
        res = ''
        if n == 0:
            return pad * self.symbols[0] if pad else self.symbols[0]
        while n != 0:
            r = n % self.l
            n = n // self.l
            res += self.char_map[r]
        if pad:
            while len(res) < pad:
                res += self.symbols[0]
        return res[::-1]
