# This is navigation and voice module written by gcz

## voice

This voice module can do ***tts***(Text to Speech), ***asr***(Audio Stream Recognition)

### useage

To configure the voice module, you need to change the dynamic link files in the ./libs and /usr/lib and the appid according to your voice SDK. And you should also modify path to the path in your computer

run "roslaunch voice voice.launch" to run the full voice module

#### Subscribes

/xf_tts: message_type std_msgs::String, function convert input string to speech

#### Publishers

/home_service_object_name_return: message_type std_msgs::String, function return the keyword of the good

### tts

file in ./src/xf_tts.cpp

### asr

file in ./src/asr_offline_record.cpp

## map_build

This map_build module can build a map and move the robot through keyboard input 

### Usage

run "roslaunch map_build mapping.launch" to construct a map
