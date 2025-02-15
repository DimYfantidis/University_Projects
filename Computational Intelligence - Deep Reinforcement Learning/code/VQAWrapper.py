from lavis.models import load_model_and_preprocess
from lavis.common.gradcam import getAttMap
import matplotlib.pyplot as plt
from PIL import Image
import torch
import numpy as np


class VqaWrapper:
    def __init__(self):
        # initialize the vqa
        # print("Initializing vqa..\n") # debug
        self.device = torch.device("cuda:0" if torch.cuda.is_available() else "cpu")
        self.model_vqa, self.vis_processors, self.txt_processors = load_model_and_preprocess(
            name="pnp_vqa", model_type="base", is_eval=True, device=self.device)
        # print("Model loaded..\n") # debug

    def ask(self, image, question):
        # load the image
        if type(image) == Image.Image:
            pass
        elif type(image) == str:
            image = Image.open(image).convert('RGB')

        # preprocess the image and the question
        images = self.vis_processors["eval"](image).unsqueeze(0).to(self.device)
        question = self.txt_processors["eval"](question)

        samples = {"image": images, "text_input": [question]}

        # generate answer
        answer_vqa, caption_vqa, gradcam, = self.model_vqa.predict_answers(
            samples=samples,
            inference_method="generate",
            num_captions=1,
            num_patches=20,
        )

        #cap_conf = "{:.4f}".format(cap_conf.item())
        #cap_conf = float(cap_conf)
        cap_conf = 0.9
        grad = samples['gradcams']
        answer = f"Answer: {answer_vqa}"
        caption = f"Caption: {caption_vqa} {cap_conf}"

        # print(answer)
        # print(caption)
        cap_conf = "0"
        return answer, caption, cap_conf, grad

    def attention_map(self, image, gradcam):
        if type(image) == Image.Image:
            pass
        elif type(image) == str:
            image = Image.open(image).convert('RGB')

        dst_w = 720
        w, h = image.size
        scaling_factor = dst_w / w

        resized_img = image.resize((int(w * scaling_factor), int(h * scaling_factor)))
        norm_img = np.float32(resized_img) / 255
        gradcam_score = gradcam.reshape(24, 24)
        gradcam_np = gradcam_score.cpu().numpy().astype(np.float32)
        avg_gradcam = getAttMap(norm_img, gradcam_np, blur=True)

        return avg_gradcam

    def printGradCam(self, image, avg_gradcamv, question):
        if type(image) == Image.Image:
            pass
        elif type(image) == str:
            image = Image.open(image).convert('RGB')

        fig, axs = plt.subplots(1, 2, figsize=(10, 5))
        axs[0].imshow(image)
        axs[0].set_yticks([])
        axs[0].set_xticks([])
        axs[0].set_title('Original Image', fontsize=10)
        axs[1].imshow(avg_gradcamv)
        axs[1].set_yticks([])
        axs[1].set_xticks([])
        axs[1].set_title('GradCam Image', fontsize=10)
        fig.suptitle(f'Question: {question}', fontsize=12)
        plt.tight_layout(rect=[0, 0, 1, 0.10])
        plt.show()

