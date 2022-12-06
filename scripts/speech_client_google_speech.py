#!/usr/bin/env python3
import os
from flask import Flask, jsonify
from flask_restful import Resource, Api, reqparse
import pandas as pd
import interactive_commands as ic
import speech_recognition as sr
from pydub import AudioSegment
import time

# code example from:
# https://www.youtube.com/watch?v=Y733Vc5hHXE&t=91s&ab_channel=buckmasterinstitute
# this speech recorder uses the microphone input and prints text, but it is not quite accurate.
# TODO: Follow this tutorial for more accurate results: 
#  https://www.google.com/search?q=python+nltk+convert+speech+from+microphone+to+text&biw=1478&bih=806&tbm=vid&sxsrf=ALiCzsZaqN3LNU-cfpbGSHxkg0xYiGB2Kg%3A1669383302236&ei=hsSAY6n_Ddy3xc8P48eimA8&ved=0ahUKEwip2uS1ucn7AhXcW_EDHeOjCPMQ4dUDCA0&uact=5&oq=python+nltk+convert+speech+from+microphone+to+text&gs_lcp=Cg1nd3Mtd2l6LXZpZGVvEAMyBQghEKABMgUIIRCgAToECCMQJzoFCAAQkQI6BAgAEEM6BQgAEIAEOgUIABCGAzoGCAAQFhAeOgcIABAeEKIEOgUIABCiBDoICCEQFhAeEB06BAghEBU6BwghEKABEAo6BAghEApQkxdYwXFg7nNoAHAAeACAAZUBiAHBJ5IBBTI4LjIzmAEAoAEBwAEB&sclient=gws-wiz-video#fpstate=ive&vld=cid:d6198483,vid:A9_0OgW1LZU
# TODO: read this tutorial: https://codelabs.developers.google.com/codelabs/cloud-speech-text-python3#9

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
            if text == 'front':
                print('moving PR2 forward')
                os.system('ls -l')
                ic.move_forward()
            elif text == 'back':
                print('moving PR2 back')
                ic.move_backward()
            elif text == 'left':
                print('moving PR2 left')
                ic.move_left()
            elif text == 'right':
                print('moving PR2 right')
                ic.move_right()
            elif text == 'reset':
                print('resetting the environment')
                ic.reset()
            elif text == 'pour':
                print('starting pouring')
                ic.perform_pouring()

        except sr.UnknownValueError:
            print('say again plz \n')
        except sr.RequestError:
            print('speech service down\n')




if __name__ == '__main__':
    print('give verble command')
    time.sleep(1)
    while True:
        audio_listner()