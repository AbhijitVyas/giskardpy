import speech_recognition as sr
from pydub import AudioSegment
import time

# code example from:
# https://www.youtube.com/watch?v=Y733Vc5hHXE&t=91s&ab_channel=buckmasterinstitute
# this speech recorder uses the microphone input and prints text, but it is not quite accurate.
# TODO: Follow this tutorial for more accurate results: 
#  https://www.google.com/search?q=python+nltk+convert+speech+from+microphone+to+text&biw=1478&bih=806&tbm=vid&sxsrf=ALiCzsZaqN3LNU-cfpbGSHxkg0xYiGB2Kg%3A1669383302236&ei=hsSAY6n_Ddy3xc8P48eimA8&ved=0ahUKEwip2uS1ucn7AhXcW_EDHeOjCPMQ4dUDCA0&uact=5&oq=python+nltk+convert+speech+from+microphone+to+text&gs_lcp=Cg1nd3Mtd2l6LXZpZGVvEAMyBQghEKABMgUIIRCgAToECCMQJzoFCAAQkQI6BAgAEEM6BQgAEIAEOgUIABCGAzoGCAAQFhAeOgcIABAeEKIEOgUIABCiBDoICCEQFhAeEB06BAghEBU6BwghEKABEAo6BAghEApQkxdYwXFg7nNoAHAAeACAAZUBiAHBJ5IBBTI4LjIzmAEAoAEBwAEB&sclient=gws-wiz-video#fpstate=ive&vld=cid:d6198483,vid:A9_0OgW1LZU
# 

recognizer = sr.Recognizer()
recognizer.energy_threshold = 300
mic = sr.Microphone()


def audio_listner():
    with mic as source:
        print('say something \n')
        audio = recognizer.listen(source)
        try:
            text = recognizer.recognize_google(audio)
            print(text)
        except sr.UnknownValueError:
            print('say again plz \n')
        except sr.RequestError:
            print('speech service down\n')

time.sleep(1)
while True:
    audio_listner()