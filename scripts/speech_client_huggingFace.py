import librosa
import torch
from transformers import Wav2Vec2ForCTC, Wav2Vec2Tokenizer
import os
import speech_recognition as sr
import ffmpeg

# https://colab.research.google.com/drive/1pTkj1HE768-3aM4huTWX5og8GkUKxKRi?usp=sharing#scrollTo=scP9E_yPrGpq
# https://www.youtube.com/watch?v=dJAoK5zK36M&ab_channel=1littlecoder

# convert mp4 to wav
actual_filename = 'test.mp4'
command2mp3 = "ffmpeg -i test.mp4 speech.mp3"
command2wav = "ffmpeg -i speech.mp3 speech.wav"
os.system(command2mp3)
os.system(command2wav)
 

#load pre-trained model and tokenizer
tokenizer = Wav2Vec2Tokenizer.from_pretrained("facebook/wav2vec2-base-960h")
model = Wav2Vec2ForCTC.from_pretrained("facebook/wav2vec2-base-960h")

#load any audio file of your choice
speech, rate = librosa.load("speech.wav",sr=16000)

import IPython.display as display
display.Audio("speech.wav", autoplay=True)

input_values = tokenizer(speech, return_tensors = 'pt').input_values

print(input_values)

#Store logits (non-normalized predictions)
logits = model(input_values).logits

print(logits)

#Store predicted id's
predicted_ids = torch.argmax(logits, dim =-1)

#decode the audio to generate text
transcriptions = tokenizer.decode(predicted_ids[0])

print(transcriptions)