import numpy as np

from typing import Literal
from time import perf_counter as timer
from collections.abc import Callable
from io import StringIO
from os import linesep

from ActivationFunctions import activationFuncDict


# Implementation of Fully Connected Neural Network
# (Read the "Multi-Layer Perceptron Implementation" markdown beforehand in the proj1_2.ipynb file)
class FullyConnectedClassifier:
    def __init__(
        self, 
        num_inputs: int, 
        num_outputs: int, 
        out_func: Literal["id", "identity", "sigmoid", "sigma", "logistic", "log", "relu", "tanh", "softmax"], 
        hidden_layers: list[tuple[int, Literal["id", "identity", "sigmoid", "sigma", "logistic", "log", "relu", "tanh", "softmax"]]], 
        learning_rate: float, 
        error_func: Literal["cross-entropy", "square-error"], 
        dtype: None = np.float64
    ) -> None:
        self.dtype = dtype
        output_activation_function_name = out_func

        if error_func == "cross-entropy":
            self.compute_loss = self.cross_entropy
        elif error_func == "square-error":
            self.compute_loss = self.square_error
        else:
            raise RuntimeError('Invalid loss function')
        self.error_func_name = error_func
        
        # Initialize n, m and l
        self.n = num_inputs
        self.m = num_outputs
        self.layers = len(hidden_layers)
        
        # Intermediate list for creating the F list
        self.out_func_name = output_activation_function_name
        # None initialization of the first element achieves 1-based indexing
        self.func_name = [None]
        self.func_name += [layer[1] for layer in hidden_layers]
        self.func_name.append(output_activation_function_name)

        # The k_s variables (no. of neurons in each layer)
        self.neurons_in_layer = [num_inputs]
        self.neurons_in_layer += [layer[0] for layer in hidden_layers]
        self.neurons_in_layer.append(num_outputs)

        # Initialize F list
        self.function_in_layer = [None]
        self.function_in_layer += [activationFuncDict[layer[1]] for layer in hidden_layers]
        self.function_in_layer.append(activationFuncDict[output_activation_function_name])

        self.learning_rate = learning_rate

        # B list of bias vectors
        self.biases = []
        # W list of weight matrices
        self.weights = []
        # U list of weighted sums vectors
        self.w_sums = []
        # V list of output vectors, v = f(u + b)
        self.values = []

        L = self.neurons_in_layer

        # None initialization of the first element achieves 1-based indexing.
        self.weights.append(None)
        self.biases.append(None)
        self.w_sums.append(None)
        # 0-based indexing in of the values (V) list denotes the input vector asthe first element.
        self.values.append(np.zeros((L[0], 1)))
        # Initialize all.
        for i in range(1, self.layers+2):
            self.weights.append(np.random.randn(L[i], L[i-1]) * np.sqrt(1./L[i]))
            self.biases.append(np.ones((L[i], 1)))
            self.w_sums.append(np.zeros((L[i], 1)))
            self.values.append(np.zeros((L[i], 1)))


    # Returns a string with details for the model.
    def diagnostics(self) -> str:
        stringBuilder = StringIO()
        eta = '\u03B7'

        stringBuilder.write(f"=============================================================={linesep}")
        stringBuilder.write(f"------------------ [Classifier Diagnostics] ------------------{linesep}")
        stringBuilder.write(f"> Learning Rate: {eta} = {self.learning_rate}{linesep}")
        stringBuilder.write(f"> Activation Function (Output): {self.out_func_name}{linesep}")
        stringBuilder.write(f"> Error Function: {self.error_func_name}{linesep}")
        stringBuilder.write(f"> Input Dim.: {self.n}{linesep}")
        stringBuilder.write(f"> Output Dim.: {self.m}{linesep}")
        stringBuilder.write(f"> Hidden Layers: ({self.layers} layers){linesep}")
        for i in range(1, self.layers+1):
            stringBuilder.write(f"\t* Neurons: {self.neurons_in_layer[i]}, Activation: {self.func_name[i]}{linesep}")
        stringBuilder.write(f"> Value Dim.:{linesep}")
        for i in range(self.layers+2):
            stringBuilder.write(f"\t* v^({i}): {np.shape(self.values[i])}{linesep}")
        stringBuilder.write(f"> Weight & Bias Dim.:{linesep}")
        for i in range(1, self.layers+2):
            stringBuilder.write(f"\t* W^({i}): {np.shape(self.weights[i])}, b^({i}): {np.shape(self.biases[i])}{linesep}")
        stringBuilder.write(f"--------------------------------------------------------------{linesep}")
        stringBuilder.write(f"=============================================================={linesep}")
        return stringBuilder.getvalue()


    # Performs One-Hot encoding on the output vectors
    def one_hot(self, Y: np.ndarray) -> np.ndarray:
        assert Y.ndim == 1
        N = len(Y)
        Y_encoded = np.zeros((N, self.m), dtype=self.dtype)
        for i in range(N):
            Y_encoded[i][Y[i]] = 1.0
        return Y_encoded


    # Computes the output for a single sample
    def forward_pass(self, sample: np.ndarray) -> None:
        assert sample.ndim == 1
        assert len(sample) == self.n

        # Convenient symbols
        f = self.function_in_layer
        W = self.weights
        u = self.w_sums
        v = self.values
        
        # Initialize the first layer with the input vector.
        self.values[0] = np.reshape(sample, (self.n, 1))

        # v^(s) = f(u^(s) + b^(s)) = f(W^(s)*v^(s-1) + b^(s))$  
        for s in range(1, self.layers+2):
            u[s] = np.matmul(W[s], v[s-1])
            v[s] = f[s][0](np.add(u[s], v[s]))


    # Cross entropy loss derivative
    def cross_entropy(self, d: np.ndarray, y: np.ndarray):
        return -d / y + (1 - d) / (1 - y)


    # Instant square error loss derivative 
    def square_error(self, d: np.ndarray, y: np.ndarray):
        return y - d


    # Used directly after forward_propagation, returns the local gradient vectors for each layer.
    def back_propagation(self, target: np.ndarray) -> list[np.ndarray]:
        assert target.ndim == 1
        assert len(target) == self.m

        f = self.function_in_layer
        u = self.w_sums
        W = self.weights
        b = self.biases
        L = self.layers + 1

        # output values
        y = self.values[L]
        # List of local gradient vectors
        delta = [None for _ in range(self.layers+2)]
        # target reshaped to vector matrix
        d = np.reshape(target, (self.m, 1))

        error = self.compute_loss(d, y)
        # Output local gradient.
        delta[L] = np.multiply(error, f[L][1](np.add(u[L], b[L])))
        # Hidden local gradents, different from the output local gradient.
        for s in range(self.layers, 0, -1):
            delta[s] = np.multiply(np.matmul(W[s+1].T, delta[s+1]), f[s][1](np.add(u[s], b[s])))
        return delta


    # Used directly after back_propagation, uses the computed local gradients to change weights and biases.
    def change_weights(self, delta: list[np.ndarray]) -> None:
        W = self.weights
        v = self.values
        b = self.biases
        for s in range(1, self.layers+2):
            W[s] = np.subtract(W[s], self.learning_rate * np.matmul(delta[s], v[s-1].T))
            b[s] = np.subtract(b[s], self.learning_rate * delta[s])


    # Trains the neural network
    def fit(self, X: np.ndarray, Y: np.ndarray, test: None | tuple[np.ndarray, np.ndarray] = None, n_epochs: int = 1):
        assert X.ndim == 2
        assert Y.ndim == 1
        assert X.shape[1] == self.n
        assert X.shape[0] == len(Y)

        # Total samples
        N = len(Y)
        # One-hot encode the desired output.
        Y_train = self.one_hot(Y)

        X_train = X.astype(dtype=self.dtype)
        X_test, Y_test = None, None

        # Test set for logging accuracy after each epoch.
        # The test set doesn't affect the training.
        if test is not None:
            X_test, Y_test = test
            assert X_test.ndim == 2
            assert Y_test.ndim == 1
            assert X_test.shape[0] == len(Y_test)

        try:
            for epoch in range(1, n_epochs+1):
                start = timer()
                # For all samples in the train set
                for i in range(N):
                    # get next sample
                    x = X_train[i]
                    y = Y_train[i]

                    # perform on-line training with that sample
                    self.forward_pass(x)
                    delta = self.back_propagation(y)
                    self.change_weights(delta)
                stop = timer()
                # Log results
                print(f"Finished epoch No.{epoch}, Elapsed Time: {stop-start:.2f}sec",end='')
                if test is not None:
                    print(f", Accuracy: {self.score(X_test, Y_test) * 100:.2f}%", end='')
                print()
        except KeyboardInterrupt:
            # Press Ctrl+C to terminate training without terminating the program.
            print(f'Interrupt: Finished training during epoch No.{epoch}')
        return self


    # Returns the output of an unknown input.
    def predict(self, input: np.ndarray) -> int:
        assert np.shape(input) == (self.n,)
        self.forward_pass(input)
        return self.values[self.layers+1].ravel().argmax()


    # Returns the output values of the output neurons.
    def predict_vector(self, input: np.ndarray) -> np.ndarray:
        assert np.shape(input) == (self.n,)
        self.forward_pass(input)
        return self.values[self.layers+1].ravel()


    # Returns the prediction accuracy of the model.
    def score(self, X: np.ndarray, Y: np.ndarray) -> float:
        assert X.ndim == 2
        assert Y.ndim == 1
        assert X.shape[1] == self.n
        assert X.shape[0] == len(Y)

        N = len(Y)
        correctly_classified = 0
        for i in range(N):
            if self.predict(X[i]) == Y[i]:
                correctly_classified += 1
        return correctly_classified / N
