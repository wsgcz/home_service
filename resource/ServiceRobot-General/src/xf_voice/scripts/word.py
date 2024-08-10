
import rospy
from std_msgs.msg import String
cunchu=''
this_flag=0
def dothing(msg):
    global cunchu
    if this_flag==1:
        rospy.loginfo("11111111111111")
        return
    shuju=""
    shuju=str(msg.data)
    start_msg.data="ok, i know you want "
    if len(shuju)>9:
        cleft=shuju.find("<confidence>")
        cright=shuju.find("</confidence>")
        pipei=shuju[cleft+12:cright]
        rospy.loginfo("%s",pipei)
        pipei=int(pipei)
        if (pipei>=0):
            shuju=""
            shuju=str(msg.data)
            left=shuju.find("<rawtext>")
            right=shuju.find("</rawtext>")
            shuju=shuju[left+9:right]
            # if shuju in ['dining-hall' ,'livingroom']:
            #     pub.publish("zero")
            if shuju=="对" and cunchu!='':
                return_msg=String()
                return_msg.data=cunchu
                cunchu=""
                pub2.publish(return_msg)
                rospy.loginfo("%s",cunchu)
            elif (shuju=="no"):
                the_ans=String()
                the_ans.data="retry"
                pub.publish(the_ans)
            else:
                if shuju=="厨房":
                    cunchu="kitchen"
                elif shuju=="餐厅":
                    cunchu="dining"
                elif shuju=="客厅":
                    cunchu="living"
                elif shuju=="卧室":
                    cunchu="bedroom"
                elif shuju=="可乐":
                    cunchu="cola"
                elif shuju=="雪碧":
                    cunchu="sprite"
                elif shuju=="芬达":
                    cunchu="orange juice"
                elif shuju=="饼干":
                    cunchu="biscuit"
                elif shuju=="乐事薯片":
                    cunchu="lays"
                elif shuju=="薯片":
                    cunchu="chip"
                elif shuju=="面包":
                    cunchu="bread"
                elif shuju=="曲奇":
                    cunchu="cookie"    
                elif shuju=="洗手液":
                    cunchu="handwash"
                elif shuju=="洗洁精":
                    cunchu="dishsoap"        
                elif shuju=="水":
                    cunchu="water"
                elif shuju=="洗发水":
                    cunchu="shampoo"                                      
                else: 
                    cunchu=shuju
                rospy.loginfo("%s",shuju)
                pub3.publish(String("yes"))
                rospy.Rate(5).sleep()
                start_msg.data=start_msg.data+shuju+" please check it"
                pub.publish(start_msg)
                return_msg=String()
                return_msg.data=shuju
                # pub2.publish(return_msg)
def over_cb(msg:String):
    global this_flag,cunchu
    cunchu=''
    rospy.loginfo("over_pub speak")
    data=String()
    data.data="pass"
    pub2.publish(data)

    this_flag=1
def refresh_cb(msg):
    global this_flag
    this_flag=0
if __name__=="__main__":
    rospy.init_node("sd0ads")
    sub=rospy.Subscriber("general_sevice_trans",String,dothing,queue_size=10)
    pub=rospy.Publisher("/general_service_xfsaywords",String,queue_size=10)
    pub2=rospy.Publisher("general_service_object_name_return",String,queue_size=10)
    pub3=rospy.Publisher('general_service_judge_the_correct',String,queue_size=10)
    sub2=rospy.Subscriber("over_speak",String,over_cb,queue_size=10)
    sub3=rospy.Subscriber("refresh",String,refresh_cb,queue_size=10)
    start_msg=String()
    rospy.spin()