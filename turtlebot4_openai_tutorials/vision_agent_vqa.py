from transformers import BlipProcessor, BlipForQuestionAnswering
from PIL import Image
import torch
import os
import sys

IMAGE_PATH = os.path.expanduser('~/turtlebot4_ws/captured_images/captured_image.png')

def answer_question(image_path=IMAGE_PATH, question=None):
    if not question:
        print("[ERROR] Nessuna domanda fornita.")
        return

    print("[INFO] Loading BLIP-VQA model...")
    processor = BlipProcessor.from_pretrained("Salesforce/blip-vqa-base")
    model = BlipForQuestionAnswering.from_pretrained("Salesforce/blip-vqa-base")

    image = Image.open(image_path).convert("RGB")
    inputs = processor(image, question, return_tensors="pt")

    print(f"[INFO] Asking: {question}")
    out = model.generate(**inputs)
    answer = processor.decode(out[0], skip_special_tokens=True)
    print(f"[ANSWER] {answer}")
    return answer

if __name__ == '__main__':
    if len(sys.argv) > 1:
        question_arg = " ".join(sys.argv[1:])
        answer_question(question=question_arg)
    else:
        print("Usage: python3 vision_agent_vqa.py \"Is there a chair?\"")

