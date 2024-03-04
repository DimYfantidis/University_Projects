import numpy as np


# Logistic Function
def logistic(x: np.ndarray) -> np.ndarray:
    return np.divide(1, (1 + np.exp(-x)))

# Logistic Function Derivative
def logistic_prime(x: np.ndarray) -> np.ndarray:
    f = logistic(x)
    return np.multiply(f, 1 - f)


# Softmax Function
def softmax(x: np.ndarray) -> np.ndarray:
    exp_x = np.exp(x - x.max())
    return exp_x / np.sum(exp_x, axis=0)

# Softmax Function Derivative
def softmax_prime(x: np.ndarray) -> np.ndarray:
    exp_x = np.exp(x-x.max())
    sum = np.sum(exp_x, axis=0)
    return exp_x / sum * (1-exp_x/sum)


# Relu Function
def relu(x: np.ndarray) -> np.ndarray:
    return np.maximum(np.zeros(np.shape(x), dtype=x.dtype), x, dtype=x.dtype) / np.linalg.norm(x, 'fro')

# Relu Function Derivative
def relu_prime(x: np.ndarray) -> np.ndarray:
    return (x > 0).astype(x.dtype)  / np.linalg.norm(x, 'fro')


# Tanh Function
def tanh(x: np.ndarray) -> np.ndarray:
    e_x = np.exp(x)
    e_neg_x = np.exp(-x)
    return (e_x - e_neg_x) / (e_x + e_neg_x)

# Tanh Function Derivative
def tanh_prime(x: np.ndarray) -> np.ndarray:
    return 1 - np.square(tanh(x))


# Identity Function
def identity(x: np.ndarray) -> np.ndarray:
    return np.copy(x)

# Identity Function Derivative
def identity_prime(x: np.ndarray) -> np.ndarray:
    return np.ones(np.shape(x), dtype=x.dtype)


activationFuncDict = {
    "softmax" : (softmax, softmax_prime), 
    "logistic" : (logistic, logistic_prime), 
    "log" : (logistic, logistic_prime), 
    "sigmoid" : (logistic, logistic_prime), 
    "sigma" : (logistic, logistic_prime), 
    "relu" : (relu, relu_prime), 
    "tanh" : (tanh, tanh_prime), 
    "id" : (identity, identity_prime), 
    "identity" : (identity, identity_prime)
}
