import rospy
from tf2_msgs.msg import TFMessage
import threading
# import keyword
from pynput import keyboard
x1=None
x2=None
y1=None
y2=None
z1=None
z2=None
ww2=None
# number=0
    # print(key)
# lock=threading.Lock()
# lock2=threading.Lock()
file ='/home/linxi/Desktop/waypoints.xml'
def on_press(key):
    global x1,x2,y1,y1,z1,z2,ww2
    # rospy.loginfo(x1)
    if x1 is None:
        return

    w1="<Pos_x>"+str(x1)+"</Pos_x>"
    w2="<Pos_y>"+str(y1)+"</Pos_y>"
    w3="<Pos_z>"+str(z1)+"</Pos_z>"
    w4="<Ori_x>"+str(x2)+"</Ori_x>"
    w5="<Ori_y>"+str(y2)+"</Ori_y>"
    w6="<Ori_z>"+str(z2)+"</Ori_z>"
    w7="<Ori_w>"+str(ww2)+"</Ori_w>"
    # w0="<Name>"+str(name)+"</Name>"
    # assert w7==0 ,"this is wrong"
    if key==keyboard.Key.enter:
        name=input("please add point_name")
        rospy.loginfo("%s",name)
        rospy.loginfo("11111111111111")
        w0="<Name>"+str(name)+"</Name>"
        
        with open(file,mode='a') as f:
            f.write("<Waypoint>")
            f.write('\n')
            f.write(w0)
            f.write('\n')
            f.write(w1)
            f.write("\n")
            f.write(w2)
            f.write("\n")
            f.write(w3)
            f.write("\n")
            f.write(w4)
            f.write("\n")
            f.write(w5)
            f.write("\n")
            f.write(w6)
            f.write("\n")
            f.write(w7)
            f.write("\n")
            f.write("</Waypoint>")
            # f.close()
    elif key==keyboard.Key.alt:
        quit()


def tf_cb(msgs:TFMessage):
    global x1,x2,y1,y2,z1,z2,ww2
    
    a=TFMessage()
    msgs=msgs.transforms
    msgs=msgs[0]
    # rospy.loginfo((msgs))
    # x1=None
    # x2=None
    # y1=None
    # y2=None
    # z1=None
    # z2=None
    # ww2=None
    # rospy.loginfo("_________________")
    id=msgs.header.frame_id
    # rospy.loginfo(type(id))
    if id=="map":
        # rospy.loginfo(id)
        x1=msgs.transform.translation.x
        y1=msgs.transform.translation.y
        z1=msgs.transform.translation.z
        x2=msgs.transform.rotation.x
        y2=msgs.transform.rotation.y
        z2=msgs.transform.rotation.z
        ww2=msgs.transform.rotation.w
        # task1=threading.Thread(target=listen_keyword,args=(x1,y1,z1,x2,y2,z2,ww2,))
        # task1.start()
if __name__=="__main__":
    rospy.init_node("add_points")
    sub=rospy.Subscriber("/tf",TFMessage,tf_cb,queue_size=10)
    with keyboard.Listener(on_press=on_press) as lsn:
        lsn.join()
    rospy.spin()