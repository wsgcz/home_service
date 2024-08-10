#! /home/melodygreen/anaconda3/envs/ros/bin/python


"""语音转文本的Publisher节点，用法：向指定的话题发布消息即可或者指定命令行参数即可开始识别"""
"""识别结果将发布到你指定的话题下"""


import aip
import time
import wave
import pyaudio
import threading
from typing import *
from pathlib import Path
from colorama import Fore, Style

import rospy
from std_msgs.msg import Int16
from sentence_voice.msg import Sentence

from config import Config



class Voice2SentencePublisher(Config):
    error_reason: Dict[int, str] = {
        3300: "输入参数不正确，请检查Config", 3301:"音频质量过差，放弃本次转换", 3302:"服务器token验证失败，请更新Config", 3303:"服务器后端错误，放弃本次转换", 3304:"用户请求超限",
        3305:"日请求量超限", 3307:"服务器后端错误，放弃本次转换", 3308:"音频过长，最大支持小于60秒的语音", 3309:"服务端无法将音频转为pcm格式",
        3310:"输入的音频文件过大", 3311:"采样率rate参数不在选项里", 3312:"音频格式format参数不在选项里，格式仅支持pcm，wav或amr"}
    msg = Sentence()
    voice_save_folder = Path(__file__).resolve().parent.joinpath("video2SentenceAudioAndTxt")
    voice_server = aip.AipSpeech(appId=Config._app_id, apiKey=Config._api_key, secretKey=Config._secret_key)
    audio_recorder = pyaudio.PyAudio()
    def __init__(self, public_topic: str="audio_result", if_save:bool=False) -> None:
        super().__init__()
        self.audio_txt_id: int = 0
        self.hearing_time: int = -1
        self.if_save: bool = if_save
        self.publisher = rospy.Publisher(name=public_topic, data_class=Sentence, latch=True, queue_size=100)
        
        subscriber_thd = threading.Thread(target=self._subscriber_thd)
        subscriber_thd.setDaemon(True)
        subscriber_thd.start()

        if not self.voice_save_folder.exists():
            self.voice_save_folder.mkdir(parents=True)
        
        rospy.set_param("audio_listen_time", param_value=-1)
        rospy.loginfo(msg=f"{Fore.GREEN}语音转换节点开启，转换结果将发布至{public_topic}话题下，可以向audio_listen_time话题发布录音时间、或者指定audio_listen_time录音时间开始转换{Style.RESET_ALL}")
    
    def _hear_time_cb(self, msg: Int16):
        if msg.data > 60:
            rospy.logwarn(msg=f"{Fore.YELLOW}语音转换一次最大支持60秒，将忽略本次请求{Fore.RESET}")
        elif self.hearing_time != -1:
            rospy.logwarn(msg=f"{Fore.YELLOW}当前正在进行语音转换，忽略本次请求{Fore.RESET}")
        else:
            self.hearing_time = int(msg.data)
    
    def _record(self, duration: int) -> List[bytes]:
        stream = self.audio_recorder.open(
            rate=self.convert_sample_rate,
            channels=self.convert_channel,
            format=self.convert_bit_depth,
            frames_per_buffer=self.convert_buffer_size,
            input=True
        )
        rospy.loginfo(msg=f"{Fore.GREEN}录音开始，请开始说话，持续{Style.BRIGHT}{duration}{Style.NORMAL} 秒{Style.RESET_ALL}")
        frames: List[bytes] = []
        printed_time: List[int] = []
        for i in range(0, self.convert_sample_rate * duration // self.convert_buffer_size):
            if (time:=(i * self.convert_buffer_size // self.convert_sample_rate)) in range(1, duration+1) and time not in printed_time:
                rospy.loginfo(msg=f"录音中... {Fore.YELLOW}{time}/{duration} s{Style.RESET_ALL}")
                printed_time.append(time)
            frames.append(stream.read(self.convert_buffer_size))
        stream.stop_stream()
        stream.close()
        if self.if_save:
            wavefile: wave.Wave_write
            with wave.open(self.voice_save_folder.joinpath(f"audio_{self.audio_txt_id//2}.mp3").__str__(), "wb") as wavefile:
                wavefile.setnchannels(self.convert_channel)
                wavefile.setsampwidth(self.audio_recorder.get_sample_size(self.convert_bit_depth))
                wavefile.setframerate(self.convert_sample_rate)
                wavefile.writeframes(b"".join(frames))
        self.audio_txt_id += 1
        return frames
    
    def _wave2sentence(self, duration: int) -> Union[bool, str]:
        frame = self._record(duration=duration)
        result: Dict[int, str] = self.voice_server.asr(
            speech=b"".join(frame),
            format="pcm",
            rate=self.convert_sample_rate,
            options={ "dev_id": self.convert_dev_pid })
        if (errno:=result["err_no"]) != 0:
            rospy.logwarn(msg=f"识别失败！错误代码：{errno}，错误原因：{self.error_reason[errno]}")
            return False
        else:
            return result["result"][0]
    
    def start(self, check_hz: float=1):
        rater = rospy.Rate(check_hz)
        while not rospy.is_shutdown():
            if self.hearing_time == -1:
                self.hearing_time = rospy.get_param(param_name="audio_listen_time", default=-1)
            if self.hearing_time == -1:
                pass
            elif sentence:=self._wave2sentence(duration=self.hearing_time):
                self.msg.header.frame_id = "voice2sentenceResult"
                self.msg.header.seq = self.audio_txt_id // 2
                self.msg.header.stamp = rospy.Time.now()
                self.msg.sentence = sentence
                self.publisher.publish(self.msg)
                if self.if_save:
                    with open(self.voice_save_folder.joinpath(f"result_{self.audio_txt_id//2}.txt").__str__(), "w") as f:
                        f.writelines([str(i)+"\n" for i in [self.msg.header.frame_id, self.msg.header.seq, self.msg.header.stamp, self.msg.sentence]])
                rospy.loginfo(msg=f"{Fore.GREEN}录音结束，识别结果为：{Fore.RESET}{sentence}")
                rospy.loginfo(msg=f"{Fore.GREEN}识别结束，等待下一次识别请求：{Fore.RESET}")
                self.audio_txt_id += 1
                self.hearing_time = -1
                rospy.set_param(param_name="audio_listen_time", param_value=-1)
            rater.sleep()

    def _subscriber_thd(self):
        self.subscriber = rospy.Subscriber(name="audio_listen_time", data_class=Int16, queue_size=100,callback=self._hear_time_cb)
        rospy.spin()


if __name__ == "__main__":
    rospy.init_node(name="voice2sentence", anonymous=False)
    v2sp = Voice2SentencePublisher(public_topic="audio_result", if_save=True)
    v2sp.start()
