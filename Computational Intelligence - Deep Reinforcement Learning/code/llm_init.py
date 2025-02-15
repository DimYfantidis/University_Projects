import re
import os

if not "HF_HOME" in os.environ.keys():
    os.environ["HF_HOME"] = '/media/data/cidl_llm/models_cache/'

from typing import Callable
from torch import bfloat16
from torch import cuda
from torch.random import manual_seed
from transformers import BitsAndBytesConfig, AutoTokenizer, AutoModelForCausalLM, Pipeline, pipeline


def llama2_init(access_token: str, system_prompt_file: str) -> tuple[Pipeline, dict, list[dict]]:
    model_name = "meta-llama/Llama-2-7b-chat-hf"

    tokenizer = AutoTokenizer.from_pretrained(model_name, token=access_token)
    bnb_config = BitsAndBytesConfig(
        load_in_4bit=True,
        bnb_4bit_quant_type='nf4',
        bnb_4bit_use_double_quant=True,
        bnb_4bit_compute_dtype=bfloat16
    )
    # Configure for 8-bit quantization
    # bnb_config = BitsAndBytesConfig(
    #     load_in_8bit=True,
    #     bnb_8bit_quant_type='int8',  # Assuming 'int8' is the desired type for 8-bit quantization
    #     bnb_8bit_compute_dtype=bfloat16  # Or you can use another appropriate dtype if needed
    # )
    model = AutoModelForCausalLM.from_pretrained(
        model_name,
        device_map={"": 0},
        quantization_config=bnb_config,
        token=access_token
    )
    model.config.use_cache = True

    generator_call_kwargs = {
        'do_sample' : True,
        'top_k' : 10,
        'num_return_sequences' : 1,
        'eos_token_id' : tokenizer.eos_token_id,
        #'max_length' : HEADER
    }
    text_generator = pipeline(
        "text-generation",
        model=model,
        tokenizer=tokenizer,
        #torch_dtype=torch.float16,
        #device_map='auto'
    )
    return text_generator, generator_call_kwargs, [dict()], lambda x: x


def llama3_init(access_token: str, system_prompt_file: str) -> tuple[Pipeline, dict, list[dict[str, str]], Callable[[str], str]]:
    model_id = "meta-llama/Meta-Llama-3-8B-Instruct"
    
    text_generator = pipeline(
        "text-generation",
        model=model_id,
        model_kwargs={"torch_dtype": bfloat16},
        device_map="auto",
        token=access_token
    )
    with open(system_prompt_file) as fp:
        system_prompt = fp.read(-1)
    messages = [
        {"role": "system", "content": system_prompt},
        {"role": "user", "content": "Greet your new user, introduce yourself and tell them about your purpose."},
    ]
    prompt = text_generator.tokenizer.apply_chat_template(
        messages, 
        tokenize=False, 
        add_generation_prompt=True
    )
    terminators = [
        text_generator.tokenizer.eos_token_id,
        text_generator.tokenizer.convert_tokens_to_ids("<|eot_id|>")
    ]
    generator_call_kwargs = {
        'max_new_tokens': 256,
        'eos_token_id': terminators,
        'do_sample': True,
        'temperature': 0.6,
        'top_p': 0.9,
        "return_full_text": False
    }
    outputs = text_generator(
        prompt,
        **generator_call_kwargs
    )
    def response_parser(_response: str) -> str:
        return _response
    
    response = outputs[0]["generated_text"]
    messages.append({"role": "assistant", "content": response})
    print('\n========================================================================================')
    print(response)
    print('==========================================================================================')

    return text_generator, generator_call_kwargs, messages, response_parser


def llama3_quant_init(access_token: str, system_prompt_file: str) -> tuple[Pipeline, dict, list[dict[str, str]], Callable[[str], str]]:
    model_id = "meta-llama/Meta-Llama-3-8B-Instruct"
    
    tokenizer = AutoTokenizer.from_pretrained(model_id, token=access_token)
    bnb_config = BitsAndBytesConfig(load_in_8bit=True)
    # Configure for 8-bit quantization
    # bnb_config = BitsAndBytesConfig(
    #     load_in_8bit=True,
    #     bnb_8bit_quant_type='int8',  # Assuming 'int8' is the desired type for 8-bit quantization
    #     bnb_8bit_compute_dtype=bfloat16  # Or you can use another appropriate dtype if needed
    # )
    model_8bit = AutoModelForCausalLM.from_pretrained(
        model_id,
        device_map="auto",
        quantization_config=bnb_config,
        token=access_token
    )
    text_generator = pipeline(
        "text-generation",
        model=model_8bit,
        tokenizer=tokenizer,
    )
    with open(system_prompt_file) as fp:
        system_prompt = fp.read(-1)
    messages = [
        {"role": "system", "content": system_prompt},
        {"role": "user", "content": "I will shortly tell you what I am seeing..."},
    ]
    prompt = text_generator.tokenizer.apply_chat_template(
        messages, 
        tokenize=False, 
        add_generation_prompt=True
    )
    terminators = [
        text_generator.tokenizer.eos_token_id,
        text_generator.tokenizer.convert_tokens_to_ids("<|eot_id|>")
    ]
    generator_call_kwargs = {
        'max_new_tokens': 256,
        'eos_token_id': terminators,
        'do_sample': True,
        'temperature': 0.6,
        'top_p': 0.9,
        "return_full_text": False
    }
    outputs = text_generator(
        prompt,
        **generator_call_kwargs
    )
    def response_parser(_response: str) -> str:
        return _response
    
    response = outputs[0]["generated_text"]
    messages.append({"role": "assistant", "content": response})
    print('\n========================================================================================')
    print(response)
    print('==========================================================================================')

    return text_generator, generator_call_kwargs, messages, response_parser


def phi3_128k_init(access_token: str, system_prompt_file: str) -> tuple[Pipeline, dict, list[dict[str, str]], Callable[[str], str]]:
    model_id = "microsoft/Phi-3-mini-128k-instruct"
    manual_seed(0)
    model = AutoModelForCausalLM.from_pretrained(
        model_id, 
        device_map="cuda", 
        torch_dtype="auto", 
        trust_remote_code=True, 
    )
    tokenizer = AutoTokenizer.from_pretrained(model_id)

    text_generator = pipeline(
        "text-generation",
        model=model,
        model_kwargs={"torch_dtype": bfloat16},
        device_map="auto",
        tokenizer=tokenizer
    )
    with open("./Large_Models_Dialogue_for_Active_Perception/llm-vqa_dialogue/system_prompts/rules_prompt.txt") as fp:
        system_prompt = fp.read(-1)
    messages = [
        {"role": "user", "content": f"/set system {system_prompt}"},
        {"role": "user", "content": "Greet your new user, introduce yourself and tell them about your purpose."},
    ]
    prompt = text_generator.tokenizer.apply_chat_template(
        messages, 
        tokenize=False, 
        add_generation_prompt=True
    )
    terminators = [
        text_generator.tokenizer.eos_token_id,
        text_generator.tokenizer.convert_tokens_to_ids("<|eot_id|>")
    ]
    generator_call_kwargs = {
        'max_new_tokens': 128,
        'eos_token_id': terminators,
        'do_sample': True,
        'temperature': 0.6,
        'top_p': 0.9,
        "return_full_text": False
    }
    outputs = text_generator(
        prompt,
        **generator_call_kwargs
    )
    def response_parser(_response: str) -> str:
        return re.findall('\".*\"', _response)[0]
    
    response = outputs[0]["generated_text"]
    messages.append({"role": "assistant", "content": response})
    print("\n==========================================================================")
    print(response)
    print("==========================================================================\n")

    return text_generator, generator_call_kwargs, messages, response_parser


def phi3_4k_init(access_token: str) -> tuple[Pipeline, dict, list[dict[str, str]], Callable[[str], str]]:
    model_id = "microsoft/Phi-3-mini-4k-instruct"
    manual_seed(0)
    model = AutoModelForCausalLM.from_pretrained(
        model_id, 
        device_map="cuda", 
        torch_dtype="auto", 
        trust_remote_code=True, 
    )
    tokenizer = AutoTokenizer.from_pretrained(model_id)

    text_generator = pipeline(
        "text-generation",
        model=model,
        # model_kwargs={"torch_dtype": bfloat16},
        # device_map="auto",
        tokenizer=tokenizer,
    )
    with open("./Large_Models_Dialogue_for_Active_Perception/llm-vqa_dialogue/system_prompts/rules_prompt.txt") as fp:
        system_prompt = fp.read(-1)
    messages = [
        {"role": "user", "content": f"/set system {system_prompt}"},
        {"role": "user", "content": "Greet your new user, introduce yourself and tell them about your purpose."},
    ]
    prompt = text_generator.tokenizer.apply_chat_template(
        messages, 
        tokenize=False, 
        add_generation_prompt=True
    )
    terminators = [
        text_generator.tokenizer.eos_token_id,
        text_generator.tokenizer.convert_tokens_to_ids("<|eot_id|>")
    ]
    generator_call_kwargs = {
        'max_new_tokens': 128,
        # 'eos_token_id': terminators,
        'do_sample': True,
        'temperature': 0.6,
        'top_p': 0.9,
        "return_full_text": False
    }
    outputs = text_generator(
        prompt,
        **generator_call_kwargs
    )
    def response_parser(_response: str) -> str:
        return re.findall('\".*\"', _response)[0]
    
    response = outputs[0]["generated_text"]
    messages.append({"role": "assistant", "content": response})
    print("\n==========================================================================")
    print(response)
    print("==========================================================================\n")

    return text_generator, generator_call_kwargs, messages, response_parser
