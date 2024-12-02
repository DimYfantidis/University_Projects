from tensorflow_model_optimization.python.core.api.quantization.keras import QuantizeConfig
from tensorflow_model_optimization.python.core.api.quantization.keras.quantizers import LastValueQuantizer
from tensorflow_model_optimization.python.core.api.quantization.keras.quantizers import MovingAverageQuantizer
from tensorflow_model_optimization.python.core.api.quantization.keras.quantizers import AllValuesQuantizer


class MyDenseQuantizeConfig(QuantizeConfig):
    def __init__(self, n_bits: int = 8):
        self.n_bits = n_bits
        super().__init__()

    def get_weights_and_quantizers(self, layer):
        return [(layer.kernel, AllValuesQuantizer(num_bits=self.n_bits, per_axis=True, symmetric=True, narrow_range=False))]

    def get_activations_and_quantizers(self, layer):
        return []

    def set_quantize_weights(self, layer, quantize_weights):
        layer.kernel = quantize_weights[0]

    def set_quantize_activations(self, layer, quantize_activations):
        pass

    def get_output_quantizers(self, layer):
        # Does not quantize output, since we return an empty list.
        return []

    def get_config(self):
        return {
            'n_bits' : self.n_bits
        }


class MyEmbeddingQuantizationConfig(QuantizeConfig):
    def __init__(self, n_bits: int = 8):
        self.n_bits = n_bits
        super().__init__()

    # Quantize the weights of the layer
    def get_weights_and_quantizers(self, layer):
        return [(layer.embeddings, AllValuesQuantizer(num_bits=self.n_bits, per_axis=True, symmetric=True, narrow_range=False))]

    # No quantization for the activations
    def get_activations_and_quantizers(self, layer):
        return []

    # Set the quantized weights back to the layer
    def set_quantize_weights(self, layer, quantize_weights):
        layer.embeddings = quantize_weights[0]

    # No quantization for activations, so this remains empty
    def set_quantize_activations(self, layer, quantize_activations):
        pass

    # No quantization for the output
    def get_output_quantizers(self, layer):
        return []

    # Return empty configuration
    def get_config(self):
        return {
            'n_bits' : self.n_bits
        }



class MyLSTMQuantizationConfig(QuantizeConfig):
    def __init__(self, n_bits: int = 8):
        self.n_bits = n_bits
        super().__init__()

    # Quantize the weights
    def get_weights_and_quantizers(self, layer):
        return [
            (layer.cell.kernel, AllValuesQuantizer(num_bits=self.n_bits, per_axis=True, symmetric=True, narrow_range=False)),
            (layer.cell.recurrent_kernel, AllValuesQuantizer(num_bits=self.n_bits, per_axis=True, symmetric=True, narrow_range=False))
        ]

    # Quantize the activations
    def get_activations_and_quantizers(self, layer):
        # return [
        #     (layer.activation, MovingAverageQuantizer(num_bits=8, symmetric=False, narrow_range=False)),
        #     (layer.recurrent_activation, MovingAverageQuantizer(num_bits=8, symmetric=False, narrow_range=False))
        # ]
        return []

    # Apply the quantized weights
    def set_quantize_weights(self, layer, quantize_weights):
        layer.cell.kernel = quantize_weights[0]
        layer.cell.recurrent_kernel = quantize_weights[1]

    # Apply the quantized activations
    def set_quantize_activations(self, layer, quantize_activations):
        # layer.activation = quantize_activations[0]
        # layer.recurrent_activation = quantize_activations[1]
        pass

    # No quantization for the output
    def get_output_quantizers(self, layer):
        return []

    # Return empty configuration
    def get_config(self):
        return {
            'n_bits' : self.n_bits
        }


class MyBatchNormalizationQuantizeConfig(QuantizeConfig):
    def __init__(self, n_bits: int = 8):
        self.n_bits = n_bits
        self.activation_quantizer = LastValueQuantizer(num_bits=self.n_bits, per_axis=False, symmetric=False, narrow_range=False)

    def get_weights_and_quantizers(self, layer):
        return []  # No weights to quantize for BatchNormalization.

    def get_activations_and_quantizers(self, layer):
        return [(layer.output, self.activation_quantizer)]

    def set_quantize_weights(self, layer, quantize_weights):
        # No weights to set for BatchNormalization.
        pass

    def set_quantize_activations(self, layer, quantize_activations):
        # Apply the quantizer to the activations.
        layer.activation = tf.keras.layers.Activation(quantize_activations[0])

    def get_output_quantizers(self, layer):
        return [self.activation_quantizer]

    def get_config(self):
        return {
            'n_bits' : self.n_bits
        }


class NoOpQuantizeConfig(QuantizeConfig):
    # Use this config for layers that you do not want to quantize
    
    def get_weights_and_quantizers(self, layer):
        return []
    
    def get_activations_and_quantizers(self, layer):
        return []
    
    def set_quantize_weights(self, layer, quantize_weights):
        # No weights are quantized.
        pass
    
    def set_quantize_activations(self, layer, quantize_activations):
        # No activations are quantized.
        pass
    
    def get_output_quantizers(self, layer):
        return []
    
    def get_config(self):
        return {}
