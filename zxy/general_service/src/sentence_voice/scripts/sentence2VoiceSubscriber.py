#! /home/melodygreen/anaconda3/envs/ros/bin/python


"""文本转语音的Subscriber节点，用法：向指定的话题发布消息即可"""
"""监听的话题是什么你自己"""

import aip
import time
import playsound
from typing import *
from pathlib import Path
from colorama import Fore, Style

import rospy
from sentence_voice.msg import Sentence

from config import Config



class Sentence2VoiceSubscriber(Config):
    error_reason: Dict[int, str] = {500: "不支持的输入", 501:"合成参数不正确，请检查Config", 502:"服务器token验证失败，请更新Config", 503:"合成后端错误，请再试一次"}
    voice_save_folder = Path(__file__).resolve().parent.joinpath("sentence2VoiceAudio")
    voice_server = aip.AipSpeech(appId=Config._app_id, apiKey=Config._api_key, secretKey=Config._secret_key)
    def __init__(self, listening_topic: str="sentence_to_speak", if_save: bool=False) -> None:
        """
        Notes:
            ROS 中通过监听指定话题进行语音播报的节点
        Arguments:
            listening_topic (str): 监听的话题
            if_save (bool): 是否将语音保存
        """
        self.mp3_id: int = 0
        self.if_save: bool = if_save
        self.subsciber: rospy.Subscriber = rospy.Subscriber(
            name=listening_topic,
            data_class=Sentence,
            queue_size=100,
            callback=self._speak_cb
        )
        if not self.voice_save_folder.exists():
            self.voice_save_folder.mkdir(parents=True)
        rospy.set_param(param_name="speaker_on", param_value=True)
        rospy.loginfo(msg=f"{Fore.GREEN}语音转换节点开启，开始监听话题：{listening_topic}，接收到该话题下的消息将自动转换为语音进行播报，此外通过指定/speaker_on参数来打开/关闭自动监听{Style.RESET_ALL}")

    def _speak_cb(self, sentence_to_speak: Sentence):
        if not rospy.get_param(param_name="speaker_on", default=True):
            rospy.logwarn(msg=f"语音播报节点监听未开启！请指定speaker_on参数开启监听，本条语音将被忽略！")
        else:
            t1 = time.time()
            rospy.loginfo(msg=f"{Fore.GREEN}语音播报节点监听到话题: {self.subsciber.name}中的消息，开始播报{Style.RESET_ALL}")
            rospy.loginfo(msg=f"\tframe_id={sentence_to_speak.header.frame_id}")
            rospy.loginfo(msg=f"\tseq_id={sentence_to_speak.header.seq}")
            rospy.loginfo(msg=f"\ttime_stamp={sentence_to_speak.header.stamp}")
            rospy.loginfo(msg=f"\tsentence={sentence_to_speak.sentence}")
            results = self._sentence2voice(sentence=sentence_to_speak.sentence)
            for i in results:
                if isinstance(i, dict):
                    rospy.logwarn(f"当前语句合成失败，错误码：{i['error_code']}，{self.error_reason[i['error_code']]}")
                    return
                else:
                    with open(path:=self.voice_save_folder.joinpath(f"result_{self.mp3_id}.mp3"), "wb") as f:
                        f.write(i)
                    playsound.playsound(path.__str__())
                    self.mp3_id += 1
                    if not self.if_save:
                        path.unlink()
            t2 = time.time()
            rospy.loginfo(msg=f"{Fore.GREEN}完成本次播报，用时：{t2-t1:>.2f} s{Style.RESET_ALL}")


    def _sentence2voice(self, sentence: str) -> List[Union[dict, bytes]]:
        all_sentence: List[str] = []
        if length:=len(sentence) > 512:
            rospy.logwarn(f"需要播报的语句过长：{length}，默认一次只能播报512个字")
            for i in range(length//500):
                all_sentence.append(sentence[i * 500, (i+i) * 500])
        else:
            all_sentence.append(sentence)
        results: List[Union[bytes, dict]] = []
        for s in all_sentence:
            result_byte: Union[dict, bytes] = self.voice_server.synthesis(
                text=s, 
                options={
                    "vol": self.synthesis_volume,
                    "pit": self.synthesis_pitch,
                    "per": self.synthesis_tone,
                    "spd": self.synthesis_speed
                })
            results.append(result_byte)
        return results



if __name__ == "__main__":
    rospy.init_node(name="sentence2voice", anonymous=False)
    s2vs = Sentence2VoiceSubscriber(listening_topic="sentence_to_speak", if_save=True)
    rospy.spin()