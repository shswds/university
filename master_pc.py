import serial
import speech_recognition as sr
from gtts import gTTS
import playsound
#import sys #-- 텍스트 저장시 사용
py_serial = serial.Serial(
    
    port='/dev/ttyACM0',
    # 보드 레이트 (통신 속도)
    baudrate=9600,
)
def speak(text):

     tts = gTTS(text=text, lang='ko')

     filename='voice.mp3'

     tts.save(filename)

     playsound.playsound(filename)

while True:
    r = sr.Recognizer()
    with sr.Microphone() as source:
        print("녹음 시작 3초")
        speech = r.record(source, duration=3)

    #sys.stdout = open('audio_output.txt', 'w') #-- 텍스트 저장시 사용

    try:
        audio = r.recognize_google(speech, language="ko-KR")
        print("Your speech thinks like\n " + audio)
        if '이름' in audio:
             commend = 'a'
             py_serial.write(commend.encode())
             speak("제이름은디에스피랩이에요")
        elif '안녕' in audio:
             commend = 'b'
             py_serial.write(commend.encode())
             speak("반가워요")
        elif '슬퍼' in audio:
             commend = 'c'
             py_serial.write(commend.encode())
             speak("저도마음이안좋아요")
        elif '바보' in audio:
             commend = 'd'
             py_serial.write(commend.encode())
             speak("그러지마세요")
        elif '멍청' in audio:
             commend = 'd'
             py_serial.write(commend.encode())
             speak("그러지마세요")
        print("Your speech thinks like\n " + audio)
    except sr.UnknownValueError:
        print("Your speech can not understand")
    except sr.RequestError as e:
        print("Request Error!; {0}".format(e))

#sys.stdout.close() #-- 텍스트 저장시 사용