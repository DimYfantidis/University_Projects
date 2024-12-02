# metrics for a neural network model using sklearn
from sklearn.metrics import accuracy_score
from sklearn.metrics import precision_score
from sklearn.metrics import recall_score
from sklearn.metrics import f1_score
from sklearn.metrics import cohen_kappa_score
from sklearn.metrics import roc_auc_score
from sklearn.metrics import confusion_matrix
from sklearn.metrics import balanced_accuracy_score
from sklearn.metrics import multilabel_confusion_matrix
from sklearn.metrics import hamming_loss
from sklearn.metrics import jaccard_score
from .QuantizationAwareInterface import *
from tensorflow import keras

import tensorflow_model_optimization as tfmot
import tensorflow as tf
import numpy as np
import pandas as pd
import random
import sys
import os
import re


# The following class is meant to provide ease of use for:
#   - TFLite model creation from keras models
#   - TFLite model structural analysis
#   - TFLite model inference and evaluation
#   - Quantization Debugger tool's applications
# by encapsulating boilerplate code and repetitive code segments
class TFLiteManager:
    def __init__(self, model: str|keras.Sequential|keras.Model, logfile: str|None=None, data_path: str='./TFLiteAPI/'):
        # Data Storage for the TFLiteAPI instance (Defaults to "${CWD}/TFLiteAPI/")
        self._data_path = data_path
        if not os.path.exists(self._data_path):
            os.makedirs(self._data_path)

        if logfile is not None:
            # The entire process of TFLite operations and 
            # conversions are logged in the specified file.
            self._log_filename = self._data_path + logfile
            self._log_stream = open(self._log_filename, 'w')
        else:
            # Logging happens on terminal if no file is provided.
            self._log_filename = None
            self._log_stream = sys.stdout
    
        if isinstance(model, str):
            # CASE 1: Keras model to be converted is specified by its path
            self._model_path = model
            self._log_stream.write("Model path provided: Fetching keras model instance\n")
            # Validate that the specified model path adheres to the ".keras" format

            if not self._model_path.endswith(".keras"):
                error_message = f"\
                Model's path must be a binary file of \".keras\" format\n \
                - Path provided: {self._model_path}\n \
                - Paths existence is yet to be validated; Aborting anyway \
                "
                # Log error message
                self._log_stream.write(error_message + '\n')
                # Terminate TFLiteAPI initialization
                raise ValueError(error_message)

            # Validate the file's existence
            if not os.path.exists(self._model_path):
                error_message = f"Model path {self._model_path} does not exist"
                # Log error message
                self._log_stream.write(error_message + '\n')
                # Terminate TFLiteAPI initialization
                raise ValueError(error_message)
            # If it exists, fetch the model's content
            self._model_content = keras.models.load_model(self._model_path)
        elif isinstance(model, (keras.Sequential, keras.Model)):
            # CASE 2: Keras model to be converted is specified by its instance
            self._model_content = model
            # A ".keras" file will not be implicitly produced
            self._model_path = None
        
        # Model's directory to be exported in PB (SavedModel) format
        self._model_dir = self._data_path + self._model_content.name
        # The TFLiteConverter
        self._converter = None
        # The TFLite model's FlatBuffer format
        self._tflite_model = None
        # This will later be the file with the statistics derived by the tf.lite.experimental.QuantizationDebugger
        self._quant_debugger_results_file = self._data_path + self._model_content.name + '_qdebugger_results.csv'
        # This is True if optimizations have been applied to the generated TFLite
        self._is_tflite_quantized = False
        # This is True if quantization for input tensor is enabled to the generated TFLite
        self._is_input_quantized = False

        self._log_stream.write(f"Model content : {self._model_content}\n")
        self._log_stream.write(f"Model path (Keras format): {self._model_path}\n")
        self._log_stream.write(f"Expected Model Directory (SavedModel format): {self._model_dir}\n")

        self.model_compilation_dict = {
            'optimizer' : self._model_content.optimizer if hasattr(self._model_content, 'optimizer') else "rmsprop",
            'loss': self._model_content.loss if hasattr(self._model_content, 'loss') else None,
            'metrics': self._model_content.metrics if hasattr(self._model_content, 'metrics') else None,
            # 'loss_weights' : self._model_content.loss_weights if hasattr(self._model_content, 'loss_weights') else None,
            # 'weighted_metrics': self._model_content.weighted_metrics if hasattr(self._model_content, 'weighted_metrics') else None,
            # 'run_eagerly' : self._model_content.run_eagerly if hasattr(self._model_content, 'run_eagerly') else None,
            # 'steps_per_execution' : self._model_content.steps_per_execution if hasattr(self._model_content, 'steps_per_execution') else None,
            # 'jit_compile' : self._model_content.jit_compile if hasattr(self._model_content, 'jit_compile') else None,
            # 'pss_evaluation_shards' : self._model_content.pss_evaluation_shards if hasattr(self._model_content, 'pss_evaluation_shards') else 0,
        }
        self._log_stream.write(f"Compilation Parameters of {self._model_content.name}\n")
        for key, value in self.model_compilation_dict.items():
            self._log_stream.write(f"\t> {key} : {value}\n")
        
        # The representative dataset
        self._X_repr = None
        # The number of samples to be used (randomly) from the representative dataset
        self._n_repr_samples = None
        # The Quantization Debugger
        self.debugger = None
        # The keyword arguments of the TFLiteAPI.get_converter() 
        # method for the TFLiteConverter's specifications
        self.conv_args = None
        self.conv_kwargs = None
        
        if logfile is not None:
            self._log_stream.close()


    # Logs a message in the given logging stream
    def log(self, arg) -> 'TFLiteManager':
        if self._log_filename is not None:
            # If the stream is not STDOUT but it's a file, then it must be opened
            self._log_stream = open(self._log_filename, 'a')
        self._log_stream.write(str(arg) + '\n')
        if self._log_filename is not None:
            # Close the logging file afterwards
            self._log_stream.close()
        return self
    
    
    # Exports the provided model in pb format
    # Code segment found here: https://colab.research.google.com/github/tensorflow/tensorflow/blob/master/tensorflow/lite/examples/experimental_new_converter/Keras_LSTM_fusion_Codelab.ipynb
    def export_model(self, model_content: keras.Model|None=None, out_file: str|None=None):
        if model_content is None:
            model_content = self._model_content
            out_file = self._model_dir
        else:
            assert out_file is not None
        def run_func(x):
            return model_content(x)
        #run_model = tf.function(lambda x: model(x))
        run_model = tf.function(run_func)
        input_shape = list(model_content.input_shape)
        if input_shape[0] is None:
            input_shape[0] = 1
        concrete_func = run_model.get_concrete_function(
            tf.TensorSpec(input_shape, model_content.inputs[0].dtype))
        model_content.save(out_file, save_format="tf", signatures=concrete_func)
        self.log(f"Model \"{model_content.name}\" exported successfully at \"{out_file}\"")

    
    @staticmethod
    # Constructs a representative dataset generator with the specified dataset and sample size
    def representative_dataset(dataset: np.ndarray, n_samples: int|None=None):
        # Note that a "representative dataset" is not actually a dataset but a generator
        # that has no arguments. This nested function is the representative dataset to be 
        # provided to the TFLiteConverter and the Quantization Debugger. The outer function 
        # provides ease of use.
        # See the "representative_dataset" attribute in: https://www.tensorflow.org/api_docs/python/tf/lite/TFLiteConverter
        assert n_samples is None or n_samples >= 1
        input_dataset = tf.data.Dataset.from_tensor_slices(dataset).batch(1)
        if n_samples is not None:
            # Use a subset of the dataset's entries, created by random sampling
            input_dataset = input_dataset.take(n_samples)
        def _representative_data_gen():
            for input_value in input_dataset:
                # Model has only one input so each data point has one element.
                yield [input_value]
        return _representative_data_gen

    
    # Initializes a new TFLiteConverter based on the provided keras model. 
    # More specifically, the converter is initialized by the pb model which
    # was generated by the export_model() function.
    def _get_converter(self, model_file: str|None=None, *args, **kwargs) -> tf.lite.TFLiteConverter:
        self.conv_args = args
        self.conv_kwargs = kwargs
        if model_file is None:
            model_file = self._model_dir
        converter = tf.lite.TFLiteConverter.from_saved_model(model_file)
        self.log(f"TFLiteConverter from \"{model_file}\"")
        if 'optimizations' in kwargs and kwargs['optimizations'] is not None: 
            self.log(f"\t> Converter Optimizations: {kwargs['optimizations']}")
            self._is_tflite_quantized = bool(kwargs['optimizations'])
            converter.optimizations = kwargs['optimizations']
        if 'representative_dataset' in kwargs and kwargs['representative_dataset'] is not None:
            if not isinstance(kwargs['representative_dataset'], tuple):
                error_message = "\t> Representative dataset arguments must be provided as a tuple in the form (dataset,) or (dataset, num_samples)"
                self.log(error_message)
                raise ValueError(error_message)
            # representative_dataset argument should be the tuple (X, n_samples)
            self._X_repr, self._n_repr_samples = kwargs['representative_dataset']
            self.log(f"\t> Representative Dataset: shape={self._X_repr.shape}, samples={self._n_repr_samples}")
            converter.representative_dataset = self.representative_dataset(*kwargs['representative_dataset'])
        if 'supported_ops' in kwargs and kwargs['supported_ops'] is not None:
            self.log(f"\t> Supported Ops: {kwargs['supported_ops']}")
            converter.target_spec.supported_ops = kwargs['supported_ops']
        if 'supported_types' in kwargs and kwargs['supported_types'] is not None:
            self.log(f"\t> Supported Tensor Types: {kwargs['supported_types']}")
            converter.target_spec.supported_types = kwargs['supported_types']
        if 'inference_type' in kwargs and kwargs['inference_type'] is not None:
            self.log(f"\t> Inference Type: {kwargs['inference_type']}")
            converter.inference_type = kwargs['inference_type']
        if 'inference_input_type' in kwargs and kwargs['inference_input_type'] is not None:
            self.log(f"\t> Inference Input Type: {kwargs['inference_input_type']}")
            self._is_input_quantized = bool(kwargs['inference_input_type'])
            converter.inference_input_type = kwargs['inference_input_type']
        if 'inference_output_type' in kwargs and kwargs['inference_output_type'] is not None:
            self.log(f"\t> Inference Output Type: {kwargs['inference_output_type']}")
            converter.inference_output_type = kwargs['inference_output_type']
        if 'allow_custom_ops' in kwargs and kwargs['allow_custom_ops'] is not None:
            self.log(f"\t> Allow Custom Ops: {kwargs['allow_custom_ops']}")
            converter.allow_custom_ops = kwargs['allow_custom_ops']
        return converter


    # Generates the TFLite property based on the provided model 
    # and the TFLiteConverter specifications.
    def get_tflite_model(self, *args, **kwargs) -> bytes:
        # Firstly, the keras model must be exported to pb format
        self.export_model()
        model = self._model_content
        self.log(f"Initializing \"{model.name}\" converter")
        # The converter is derived from the specifications provided by the arguments
        converter = self._get_converter(*args, **kwargs)
        self.log(f"Converting \"{model.name}\" to TFLite format")
        # Generates the TFLite Model
        tflite_model = converter.convert()
        self._tflite_model = tflite_model
        self.log("Saved TFLite model's FlatBuffer format as instance property")
        # Saves the TFLite model's FlatBuffer format to a binary file
        self._tflite_path = self._data_path + model.name + '.tflite'
        with open(self._tflite_path, 'wb') as file:
            file.write(self._tflite_model)
        self.log(f"Saved TFLite model's FlatBuffer format in file \"{self._tflite_path}\"")
        # Saves the converter for later usage
        self._converter = converter
        self.log(f"\"{model.name}\" TFLite model size: {len(tflite_model) / 1024.0:.2f} Kilobytes")
        return tflite_model


    # Scales a numpy matrix's values to [0, 1].
    # Parameters:
    #   X (numpy.ndarray): The input matrix to be normalized.
    # Returns:
    #   numpy.ndarray: The scaled matrix with values in the range [0, 1].
    @staticmethod
    def _normalize(X: np.ndarray, axis: int|None=None):
        X_min = X.min(axis=axis)
        X_max = X.max(axis=axis)
        X_normalized = (X - X_min) / (X_max - X_min)
        return X_normalized


    # Runs inference on the given dataset using the specified classifier, if provided.
    # If the model is not provided, then the examined model of the TFLiteManager's session is used.
    # This allows for a flexible interface to run inference on other, irrelevant classifiers as well.
    def run_tflite_model(self, X: np.ndarray, model_content: bytes|None=None, is_multiclass: bool=False, is_quantized: bool=False) -> np.ndarray:
        # Standard code segment for running inference 
        # using a TFLite model using the TFLite Interpreter
        if model_content is None:
            model_content = self._tflite_model
        interpreter = tf.lite.Interpreter(model_content=model_content)
        interpreter.allocate_tensors()
        input_details = interpreter.get_input_details()[0]
        output_details = interpreter.get_output_details()[0]
        n_classes = output_details['shape'][-1]
        # Initialize the output matrix
        out_dtype = output_details['dtype']
        predictions = np.zeros(
            shape=(X.shape[0], n_classes) if is_multiclass else X.shape[0], 
            dtype=out_dtype
        )
        # It is assumed that float models produce output vectors where each of their scalars are in the range [0, 1]. Thus
        # the mid-point is arbitrarily assumed to be 1/2. In case that this isn't true (e.g.: output in [0, 10] or [-1, 1]), 
        # this method, as well as the `evaluate_tflite_model` method, must be refactored accordingly.
        mid_point = .5 if np.issubdtype(out_dtype, np.floating) else (np.iinfo(out_dtype).max + np.iinfo(out_dtype).min) // 2 + 1
        for i, sample in enumerate(X):
            # Callibrate the quantized input
            input_scale, input_zero_point = input_details["quantization"] if input_details["quantization"] != (0.0, 0) else (1.0, 0)
            sample = sample / input_scale + input_zero_point
            # Morph the initial sample in an acceptable vector for the Interpreter
            sample = np.expand_dims(sample, axis=0).astype(input_details["dtype"])
            interpreter.set_tensor(input_details["index"], sample)
            interpreter.invoke()
            output = interpreter.get_tensor(output_details["index"])[0]
            # Callibrate the quantized output
            # output_scale, output_zero_point = output_details["quantization"] if output_details["quantization"] != (0.0, 0) else (1.0, 0)
            # output = output_scale * (output - output_zero_point)
            if is_multiclass:
                # one-hot encoded output
                idx = (output >= mid_point)
                output[idx] = 1
                output[~idx] = 0
                predictions[i] = output
            else:
                # Assumes > 2 classes; Binary classification demands thresholding 
                # the single sigmoid output, not argmax-ing the softmax/logits outputs.
                predictions[i] = output.argmax()
        return predictions

    
    # Creates a modelname_analysis.log file that includes an in-depth, structural
    # model analysis. By reading the log file, the user can obtain specific details 
    # about the TFLite model's graph, layers, operations and tensors, as well as the 
    # relationships between them.
    def analyze_tflite_model(self) -> None:
        # Note: The TFLite Analyzer is an experimental feature. Thus, it's not safe to 
        # build robust, production-level code using this feature as it will be vulnerable 
        # to changes. The following method, as well as the TFLiteManager.get_tflite_tensors() 
        # may break in future versions.
        analysis_path = self._data_path + self._model_content.name + '_analysis.log'
        ORIGINAL_STDOUT = sys.stdout
        # TFLite Analyzer logs everything in STDOUT and offers no way to redirect its 
        # output. Thus, stdout is swapped and re-assigned later.
        sys.stdout = open(analysis_path, 'w')
        tf.lite.experimental.Analyzer.analyze(model_path=self._tflite_path)
        self.log(f"TFLite Analysis stored at {analysis_path}")
        sys.stdout.close()
        sys.stdout = ORIGINAL_STDOUT

    
    # Returns a list with all the TFLite model's tensor descriptions (available in the 
    # analysis log file) based on their data type. Example arguments are "INT8" or "FLOAT32"
    # to fetch the corresponding tensors. 
    def get_tflite_tensors(self, dtype: str):
        analysis_path = self._data_path + self._model_content.name + '_analysis.log'
        if not os.path.exists(analysis_path):
            error_message = "TFLite model has not yet been analyzed: call analyze_tflite_model() before calling get_tflite_tensors()"
            self.log(error_message)
            raise OSError(error_message)
        # Reads the whole file
        with open(analysis_path, 'r') as file:
            tflite_log = file.read(-1)
        # The result is determined using regular expressions
        return re.findall(f"T#[0-9]+\(.*\).*type:{dtype.upper()}.*", tflite_log)
    
     
    # Derives performance metrics from the provided (X_test, y_test) dataset.
    # See: TFLiteManager.run_tflite_model()
    def evaluate_tflite_model(
        self, X: np.ndarray, 
        y: np.ndarray, 
        results: list[str] | set[str], 
        model_content: bytes | None = None, 
        is_multiclass: bool=False, 
        is_quantized: bool|None=None
    ) -> tuple[dict, np.ndarray | None]:
        if model_content is None:
            model_content = self._tflite_model
            is_quantized = self._is_input_quantized
        elif is_quantized is None:
            error_message = "Please specify if the provided model is quantized or not with the `is_quantized` function argument."
            self.log(error_message)
            raise ValueError(error_message)
        if X.shape[0] != y.shape[0]:
            error_message = f" \
                Cannot evaluate TFLite model: Sample and label dimensions don't match \
                - Samples provided: {X.shape[0]} \
                - Labels provided:  {y.shape[0]} \
                "
            self.log(error_message)
            raise ValueError(error_message)

        # Generate the predictions to compare them with the ground-truth labels.
        predictions = self.run_tflite_model(X, model_content, is_multiclass, is_quantized)
        _results = {metric.lower() for metric in results}
        metrics_to_return = {}
        conf_matrix = None

        if 'accuracy' in _results:
            # accuracy: (tp + tn) / (p + n)
            accuracy = accuracy_score(y, predictions)
            metrics_to_return.update({'Accuracy': accuracy})
        if 'precision' in _results:
            # precision tp / (tp + fp)
            precision = precision_score(y, predictions, average='macro')
            metrics_to_return.update({'Precision': precision})
        if 'recall' in _results:
            # recall: tp / (tp + fn)
            recall = recall_score(y, predictions, average='macro')
            metrics_to_return.update({'Recall': recall})
        if 'f1' in _results:
            # f1: 2 tp / (2 tp + fp + fn)
            f1 = f1_score(y, predictions, average='macro')
            metrics_to_return.update({'F1': f1})
        if 'balanced_accuracy' in _results:
            # balanced accuracy
            balanced_accuracy = balanced_accuracy_score(y, predictions)
            metrics_to_return.update({'Balanced_Accuracy': balanced_accuracy})
        if 'cohen_kappa' in _results:
            # kappa
            kappa = cohen_kappa_score(y, predictions)
            metrics_to_return.update({'Cohens_kappa': kappa})
        if 'roc_auc' in _results:
            # ROC AUC
            auc = roc_auc_score(y, predictions, average='macro', multi_class='ovo')
            metrics_to_return.update({'ROC_AUC': auc})
        if 'binary_accuracy' in _results:
            # binary accuracy
            binary_accuracy_metric = tf.keras.metrics.BinaryAccuracy()
            binary_accuracy_metric.update_state(y, predictions)
            binary_accuracy = binary_accuracy_metric.result().numpy()
            metrics_to_return.update({'Binary_Accuracy': binary_accuracy})
        if 'hamming_loss' in _results:
            # hamming loss
            hamming = hamming_loss(y, predictions)
            metrics_to_return.update({'Hamming_Loss': hamming})
        if 'jaccard_score' in results:
            # jaccard score
            jaccard = jaccard_score(y, predictions, average='macro')
            metrics_to_return.update({'Jaccard_Score': jaccard})
        if 'confusion_matrix' in _results:
            # confusion matrix
            conf_matrix = confusion_matrix(y, predictions)
        if 'multilabel_confusion_matrix' in _results:
            # multilabel confusion matrix
            conf_matrix = multilabel_confusion_matrix(y, predictions)

        # Returns the confusion matrix separately from the other performance metrics
        return metrics_to_return, conf_matrix


    # Specifies a new Quantization Debugger.
    def set_quant_debugger(self, X_debug: np.ndarray, n_samples: int=None, debug_options: tf.lite.experimental.QuantizationDebugOptions | None=None, model_dir: str|None=None):
        converter = self._get_converter(model_dir, *self.conv_args, **self.conv_kwargs)
        if self._X_repr is not None:
            if X_debug.shape != self._X_repr.shape or n_samples != self._n_repr_samples:
                error_message = "Quantization Debugger Preparation Error: debug dataset must be the same as representative dataset"
                self.log(error_message)
                raise ValueError(error_message)
        debugger = tf.lite.experimental.QuantizationDebugger(
            converter=converter, 
            debug_dataset=self.representative_dataset(X_debug, n_samples), 
            debug_options=debug_options
        )
        self.debugger = debugger


    # Specifies a new Quantization Debugger.
    def set_quant_debugger(self, X_debug: np.ndarray, n_samples: int=None, debug_options: tf.lite.experimental.QuantizationDebugOptions | None=None, model_dir: str|None=None):
        converter = self._get_converter(model_dir, *self.conv_args, **self.conv_kwargs)
        if self._X_repr is not None:
            if X_debug.shape != self._X_repr.shape or n_samples != self._n_repr_samples:
                error_message = "Quantization Debugger Preparation Error: debug dataset must be the same as representative dataset"
                self.log(error_message)
                raise ValueError(error_message)
        debugger = tf.lite.experimental.QuantizationDebugger(
            converter=converter, 
            debug_dataset=self.representative_dataset(X_debug, n_samples), 
            debug_options=debug_options
        )
        self.debugger = debugger


    # Save the comparison metrics to a CSV file
    def export_statistics(self, out_file: str|None=None):
        if out_file is None:
            out_file = self._quant_debugger_results_file
        with open(out_file, 'w') as f:
            self.debugger.layer_statistics_dump(f)
        self.log(f"Quantization Debugger: Dumped layer metrics to \"{out_file}\"")


    # Fetch the dataframe of the comparison metrics from the CSV file
    def fetch_debug_statistics(self, in_file: str|None=None) -> pd.DataFrame:
        if in_file is None:
            in_file = self._quant_debugger_results_file
        # self.debugger.get_nondebug_quantized_model
        self.log(f"Quantization Debugger: Reading layer metrics from \"{in_file}\"")
        return pd.read_csv(in_file)


    @staticmethod
    def sample_dataset(X: np.ndarray, y: np.ndarray, percentage: float) -> tuple[np.ndarray, np.ndarray]:
        assert X.shape[0] == y.shape[0]
        # Calculate the number of elements to sample
        num_samples = int(X.shape[0] * percentage)
        # Generate random indices to sample from the input array
        idx = np.random.choice(X.shape[0], size=num_samples, replace=False)
        # Return the sampled subset
        return X[idx], y[idx]


    @staticmethod
    # Returns a function that will be used as a `clone_function` for the `keras.models.clone_model` function.
    def selective_apply_quantization_to_layer(layers_and_quantizers: dict[type, QuantizeConfig]):
        # A dictionary is provided to the outer function that matches layer types to their corresponding QuantizationConfig.
        # The inner function must take a single parameter (layer). The inner function is called by the `keras.models.clone_model`
        # for each layer in the model.
        def _selective_apply_quantization_to_layer_func(layer):
            if type(layer) in layers_and_quantizers.keys():
                # The `if` handles layers whose types are explicitly specified to be quantized under specific configuration.
                return tfmot.quantization.keras.quantize_annotate_layer(layer, quantize_config=layers_and_quantizers[type(layer)])
            # Else, they are callibrated in their default manner.
            return tfmot.quantization.keras.quantize_annotate_layer(layer)
        return _selective_apply_quantization_to_layer_func


    # Generates the Quantization Aware Version of the pipeline's model.
    # Parameters:
    #   - `X_train` and `y_train`: the model's original training dataset.
    #   - `layers_and_quantizers`: to be fed to the `TFLiteManager.selective_apply_quantization_to_layer` function (see above)
    #   - `custom_quantize_configs`: the dictionary in the form that the `tfmot.quantization.keras.quantize_scope` demands, which
    #     corresponds each custom QuantizeConfig's name to its class.
    #   - `custom_classes`: Has the same form as `custom_quantize_configs` but is related to custom layer classes. If provided, it 
    #     is appended to the `quantize_scope`.
    #   - `new_metrics`: If provided, it replaces the model's initial training metrics with new (compatible) ones.
    def generate_quant_aware(
        self, 
        X_train: np.ndarray, 
        y_train: np.ndarray, 
        layers_and_quantizers: dict[type, QuantizeConfig], 
        custom_quantize_configs: dict[str, type], 
        custom_classes: dict[str, type] | None=None,
        new_metrics: list[keras.metrics.Metric] | None=None
    ) :
        self.log(f"Quantization Aware Training: Fine-Tuning model layers with QuantizeConfigs:")
        quantize_scope = dict(**custom_quantize_configs)
        for key, value in quantize_scope.items():
            self.log(f"\t> {key} : {value}")
        if custom_classes is not None:
            # Any custom layer types are appended to the quantize scope
            quantize_scope.update(custom_classes)
        with tfmot.quantization.keras.quantize_scope(quantize_scope):
            # Load the initial model saved in pb format by `TFLiteManager.export_model` function.
            model = keras.models.load_model(self._model_dir)
            # Fetch the annotated model using the `TFLiteManager.selective_apply_quantization_to_layer` function.
            annotated_model = keras.models.clone_model(
                model,
                clone_function=self.selective_apply_quantization_to_layer(layers_and_quantizers),
            )
            self.quant_aware_model = tfmot.quantization.keras.quantize_apply(annotated_model)
            # Stores the model's initial compilation configuration (optimizer, loss function, metrics).
            model_compilation_dict = self.model_compilation_dict
            if new_metrics is not None:
                # Replace the metrics with new, compatible metrics.
                model_compilation_dict = dict(**self.model_compilation_dict)
                model_compilation_dict['metrics'] = new_metrics
            # Recompile
            self.quant_aware_model.compile(**model_compilation_dict)
            # Train Quant-Aware model on the training dataset for just one epoch.
            self.quant_aware_model.fit(X_train, y_train, epochs=1)
            out_dir: str = self._data_path + self._model_content.name + '_QAWARE/'
            # Save the Quant-Aware model on disk.
            self.export_model(self.quant_aware_model, out_dir)
            # Convert it to TFLite format.
            converter = self._get_converter(model_file=out_dir, *self.conv_args, **self.conv_kwargs)
            converter.experimental_new_quantizer = True
            converter.experimental_enable_debug_info = True
            converter.experimental_enable_resource_variables = True
            self.tflite_qaware_model = converter.convert()
            # Save TFLite model on disk.
            with open(out_dir.removesuffix('/') + '.tflite', 'wb') as fp:
                fp.write(self.tflite_qaware_model)
