# from select import select
# import string
from std_msgs.msg import String
import rospy
count=0
class Sub:
    def __init__(self) -> None:
        self.sub=rospy.Subscriber("hh",String,self.cb,queue_size=10)
        self.count=0
    def cb(self,msg:String):

        # print(msg.data)
        self.count+=1
        print(self.count)
        if self.count%10==0:
            print("del myself")
            del self
def acb(msg:String):
    sub=Sub()
    # rospy.spin()      
if __name__=="__main__":
    rospy.init_node("hhh1")
    sub=rospy.Subscriber("ahh",String,acb,queue_size=10)
    rospy.spin()