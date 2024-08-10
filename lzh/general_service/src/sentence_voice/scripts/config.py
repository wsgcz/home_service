import pyaudio


class Config(object):
    _app_id: str = "25175645"
    _api_key: str = "8zbGN5lmACmcYnQw05nj68w8"
    _secret_key: str = "TkHfzSRNU09ZsMfAUj14UYGXlkeUxY7D"

    # * 语音合成的配置
    # 音量，0~15，15最大
    synthesis_volume: int = 10
    # 音调，0~9，9最尖锐
    synthesis_pitch: int = 5
    # 语速，0~9，9最快
    synthesis_speed: int =5
    # 声音，0、1、3、4：无感情女声、无感情男声、有感情男声、有感情女声
    synthesis_tone: int = 0

    # * 语音转换的配置
    # 音频通道数，只支持1
    convert_channel: int = 1
    # 采样率, 16000或者8000
    convert_sample_rate: int = 16000
    # 位深度
    convert_bit_depth: int = pyaudio.paInt16
    # 缓冲区大小
    convert_buffer_size: int = 1024
    # 识别模式，1537位普通话（近场），1936为普通话（远场），1737为英语
    convert_dev_pid: int = 1537