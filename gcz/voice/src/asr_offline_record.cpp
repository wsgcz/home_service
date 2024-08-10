/*
there are some path you need to change in this file
* 语音听写(iFly Auto Transform)技术能够实时地将语音转换成对应的文字。
it can understand key word and the key word is in call.bnf,you can went to kedaxunfei to learn it 
*/
#include "ros/ros.h"
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <errno.h>
#include <unistd.h>
#include "../include/qisr.h"
#include "msp_cmn.h"
#include "msp_errors.h"
#include "speech_recognizer.h"

#include"std_msgs/String.h"
#define FRAME_LEN	640 
#define	BUFFER_SIZE	4096
#define SAMPLE_RATE_16K     (16000)
#define SAMPLE_RATE_8K      (8000)
#define MAX_GRAMMARID_LEN   (32)
#define MAX_PARAMS_LEN      (1024)
//attention :you need to change the path there
//change them to the absolute path in your computer where those files are
const char * ASR_RES_PATH        = "fo|/home/gcz/test/src/voice/bin/msc/res/asr/common.jet"; //离线语法识别资源路径
const char * GRM_BUILD_PATH      = "/home/gcz/test/src/voice/bin/msc/res/asr/GrmBuilld"; //构建离线语法识别网络生成数据保存路径
const char * GRM_FILE            = "/home/gcz/test/src/voice/bin/call.bnf"; //构建离线识别语法网络所用的语法文件
const char * LEX_NAME            = "contact"; //更新离线识别语法的contact槽（语法文件为此示例中使用的call.bnf）
int judge=1;//this variable will help to judge whether you speak or not

typedef struct _UserData {
	int     build_fini; //标识语法构建是否完成
	int     update_fini; //标识更新词典是否完成
	int     errcode; //记录语法构建或更新词典回调错误码
	char    grammar_id[MAX_GRAMMARID_LEN]; //保存语法构建返回的语法ID
}UserData;
char * f_result;

const char *get_audio_file(void); //选择进行离线语法识别的语音文件
int build_grammar(UserData *udata); //构建离线识别语法网络
int update_lexicon(UserData *udata); //更新离线识别语法词典
int run_asr(UserData *udata); //进行离线语法识别

const char* get_audio_file(void)
{
	int key = 0;
	while(key != 27) //按Esc则退出
	{
		printf("请选择音频文件：\n");
		printf("1.打电话给丁伟\n");
		printf("2.打电话给黄辣椒\n");
		scanf("%d", &key);
		//key = getc();
		//printf("key==========%c",key);
		switch(key)
		{
		case 1:
			printf("\n1.打电话给丁伟\n");
			return "wav/ddhgdw.pcm";
		case 2:
			printf("\n2.打电话给黄辣椒\n");
			return "wav/ddhghlj.pcm";
		default:
			continue;
		}
	}
	exit(0);
	return NULL;
}

int build_grm_cb(int ecode, const char *info, void *udata)
{
	UserData *grm_data = (UserData *)udata;

	if (NULL != grm_data) {
		grm_data->build_fini = 1;
		grm_data->errcode = ecode;
	}

	if (MSP_SUCCESS == ecode && NULL != info) {
		printf("构建语法成功！ 语法ID:%s\n", info);
		if (NULL != grm_data)
			snprintf(grm_data->grammar_id, MAX_GRAMMARID_LEN - 1, info);
	}
	else
		printf("构建语法失败！%d\n", ecode);

	return 0;
}

int build_grammar(UserData *udata)
{
	FILE *grm_file                           = NULL;
	char *grm_content                        = NULL;
	unsigned int grm_cnt_len                 = 0;
	char grm_build_params[MAX_PARAMS_LEN]    = {NULL};
	int ret                                  = 0;

	grm_file = fopen(GRM_FILE, "rb");	
	if(NULL == grm_file) {
		printf("打开\"%s\"文件失败！[%s]\n", GRM_FILE, strerror(errno));
		return -1; 
	}

	fseek(grm_file, 0, SEEK_END);
	grm_cnt_len = ftell(grm_file);
	fseek(grm_file, 0, SEEK_SET);

	grm_content = (char *)malloc(grm_cnt_len + 1);
	if (NULL == grm_content)
	{
		printf("内存分配失败!\n");
		fclose(grm_file);
		grm_file = NULL;
		return -1;
	}
	fread((void*)grm_content, 1, grm_cnt_len, grm_file);
	grm_content[grm_cnt_len] = '\0';
	fclose(grm_file);
	grm_file = NULL;

	snprintf(grm_build_params, MAX_PARAMS_LEN - 1, 
		"engine_type = local, \
		asr_res_path = %s, sample_rate = %d, \
		grm_build_path = %s, ",
		ASR_RES_PATH,
		SAMPLE_RATE_16K,
		GRM_BUILD_PATH
		);
	ret = QISRBuildGrammar("bnf", grm_content, grm_cnt_len, grm_build_params, build_grm_cb, udata);

	free(grm_content);
	grm_content = NULL;

	return ret;
}

int update_lex_cb(int ecode, const char *info, void *udata)
{
	UserData *lex_data = (UserData *)udata;

	if (NULL != lex_data) {
		lex_data->update_fini = 1;
		lex_data->errcode = ecode;
	}

	if (MSP_SUCCESS == ecode)
		printf("更新词典成功！\n");
	else
		printf("更新词典失败！%d\n", ecode);

	return 0;
}

int update_lexicon(UserData *udata)
{
	const char *lex_content                   = "丁伟\n黄辣椒";
	unsigned int lex_cnt_len                  = strlen(lex_content);
	char update_lex_params[MAX_PARAMS_LEN]    = {NULL}; 

	snprintf(update_lex_params, MAX_PARAMS_LEN - 1, 
		"engine_type = local, text_encoding = UTF-8, \
		asr_res_path = %s, sample_rate = %d, \
		grm_build_path = %s, grammar_list = %s, ",
		ASR_RES_PATH,
		SAMPLE_RATE_16K,
		GRM_BUILD_PATH,
		udata->grammar_id);
	return QISRUpdateLexicon(LEX_NAME, lex_content, lex_cnt_len, update_lex_params, update_lex_cb, udata);
}

static void show_result(char *string, char is_over)
{
	judge=0; //when judge is 0,it means that the record process is successful
	f_result=string;//send the result to f_result
	printf("\rResult: [ %s ]", string);
	char *shuzu=string;
	if(is_over)
		putchar('\n');
}

static char *g_result = NULL;
static unsigned int g_buffersize = BUFFER_SIZE;

void on_result(const char *result, char is_last)
{
	if (result) {
		size_t left = g_buffersize - 1 - strlen(g_result);
		size_t size = strlen(result);
		if (left < size) {
			g_result = (char*)realloc(g_result, g_buffersize + BUFFER_SIZE);
			if (g_result)
				g_buffersize += BUFFER_SIZE;
			else {
				printf("mem alloc failed\n");
				return;
			}
		}
		strncat(g_result, result, size);
		show_result(g_result, is_last);
	}
}
void on_speech_begin()
{
	if (g_result)
	{
		free(g_result);
	}
	g_result = (char*)malloc(BUFFER_SIZE);
	g_buffersize = BUFFER_SIZE;
	memset(g_result, 0, g_buffersize);

	printf("Start Listening...\n");
}
void on_speech_end(int reason)
{
	if (reason == END_REASON_VAD_DETECT)
		printf("\nSpeaking done \n");
	else
		printf("\nRecognizer error %d\n", reason);
}

/* demo recognize the audio from microphone */
static void demo_mic(const char* session_begin_params)
{
	int errcode;
	int i = 0;

	struct speech_rec iat;

	struct speech_rec_notifier recnotifier = {
		on_result,
		on_speech_begin,
		on_speech_end
	};

	errcode = sr_init(&iat, session_begin_params, SR_MIC, &recnotifier);
	if (errcode) {
		printf("speech recognizer init failed\n");
		return;
	}
	errcode = sr_start_listening(&iat);
	if (errcode) {
		printf("start listen failed %d\n", errcode);
	}
	/* demo 15 seconds recording */
	while(i++ < 4)
	 {
		ros::Duration du(1);
	 	du.sleep();	
	 }
	errcode = sr_stop_listening(&iat);
	if (errcode) {
		printf("stop listening failed %d\n", errcode);
	}
	sr_uninit(&iat);

}

int run_asr(UserData *udata)
{
	char asr_params[MAX_PARAMS_LEN]    = {NULL};
	const char *rec_rslt               = NULL;
	const char *session_id             = NULL;
	const char *asr_audiof             = NULL;
	FILE *f_pcm                        = NULL;
	char *pcm_data                     = NULL;
	long pcm_count                     = 0;
	long pcm_size                      = 0;
	int last_audio                     = 0;

	int aud_stat                       = MSP_AUDIO_SAMPLE_CONTINUE;
	int ep_status                      = MSP_EP_LOOKING_FOR_SPEECH;
	int rec_status                     = MSP_REC_STATUS_INCOMPLETE;
	int rss_status                     = MSP_REC_STATUS_INCOMPLETE;
	int errcode                        = -1;
	int aud_src                        = 0;
	//离线语法识别参数设置
	snprintf(asr_params, MAX_PARAMS_LEN - 1, 
		"engine_type = local, \
		asr_res_path = %s, sample_rate = %d, \
		grm_build_path = %s, local_grammar = %s, \
		result_type = xml, result_encoding = UTF-8, ",
		ASR_RES_PATH,
		SAMPLE_RATE_16K,
		GRM_BUILD_PATH,
		udata->grammar_id
		);
	
	//printf("音频数据在哪? \n0: 从文件读入\n1:从MIC说话\n");
	//scanf("%d", &aud_src);
		demo_mic(asr_params);
	return 0;
}


class ros_voice
{
	public:
	ros_voice(ros::NodeHandle &nh)
	{

		pub_speak=nh.advertise<std_msgs::String>("/xf_tts",10);
		pub_listen_res=nh.advertise<std_msgs::String>("/get_listen_res",10);
		pub_listen=nh.advertise<std_msgs::String>("/start_listen",10);
		sub=nh.subscribe("/start_listen",10,&ros_voice::domsg,this);
		ROS_INFO("语音接收模块启动成功！");
		ros::spin();
	}

	void domsg(const std_msgs::String::ConstPtr& msg)
	{
		ROS_INFO("i got a msg");
		if (msg->data=="startlisten")
		{	
			ROS_INFO("yes ,i have got the message");
			const char *login_config    = "appid = a34011d4"; //登录参数
			UserData asr_data; 
			int ret                     = 0 ;
			std_msgs::String shuju;
			std_msgs::String canshu;
		ret = MSPLogin(NULL, NULL, login_config); //第一个参数为用户名，第二个参数为密码，传NULL即可，第三个参数是登录参数
		if (MSP_SUCCESS != ret) {
			printf("登录失败：%d\n", ret);
			MSPLogout();
			printf("wait for the next order");
			return ;
		}

		memset(&asr_data, 0, sizeof(UserData));
		printf("构建离线识别语法网络...\n");
		ret = build_grammar(&asr_data);  //第一次使用某语法进行识别，需要先构建语法网络，获取语法ID，之后使用此语法进行识别，无需再次构建
		if (MSP_SUCCESS != ret) {
			printf("构建语法调用失败！\n");
			MSPLogout();
			printf("wait for the next order");
			return ;
		}
		while (1 != asr_data.build_fini)
			usleep(300 * 1000);
		if (MSP_SUCCESS != asr_data.errcode)
			{MSPLogout();
			printf("wait for the next order");
			return ;}
		printf("离线识别语法网络构建完成，开始识别...\n");
		std_msgs::String str;
		str.data="you can talk now"; 
		pub_speak.publish(str);
		ros::Duration duration(1.3);
		duration.sleep();
		judge=1;
		ret = run_asr(&asr_data);
		if(judge)
			f_result="1";
		canshu.data=f_result;
		ROS_INFO("%d",strlen(f_result));
		if (canshu.data=="1")
		{
			std_msgs::String redata;
			std_msgs::String msg;
			msg.data="startlisten";
			redata.data="please try again";
			pub_speak.publish(redata);
			duration.sleep();
			pub_listen.publish(msg);
		}
		else
			pub_listen_res.publish(canshu);
		if (MSP_SUCCESS != ret) {
			printf("离线语法识别出错: %d \n", ret);
			MSPLogout();
			printf("wait for the next order");
			return ;
		}
		printf("更新离线语法词典...\n");
			MSPLogout();
			printf("wait for the next order");
			return ;
		}
	}

	private:
	ros::Subscriber sub;
	ros::Publisher pub_speak;
	ros::Publisher pub_listen_res;
	ros::Publisher pub_listen;
};

int main(int argc, char* argv[])
{	
	setlocale(LC_ALL,"");
	UserData asr_data; 
	int ret                     = 0 ;
	char c;
	ros::init(argc,argv,"yuyin");
	ros::NodeHandle nh;
	ros_voice rv(nh);//it will run the class above
	ros::spin();
}