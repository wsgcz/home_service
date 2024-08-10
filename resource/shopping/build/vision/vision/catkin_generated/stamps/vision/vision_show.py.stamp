import rospy
import cv2 as cv
import json
from cv_bridge import CvBridge
import numpy as np

from sensor_msgs.msg import Image

cb = CvBridge()

def put_id(image:cv.Mat, info_dict:dict) -> cv.Mat:
    try:
        xyxy_array = np.array(info_dict['coord'])
        id_array = info_dict['id array']
    except KeyError:
        return image

    center_x = (xyxy_array[:, 0] + xyxy_array[:, 2])/2
    center_y = (xyxy_array[:, 1] + xyxy_array[:, 3])/2

    for i in range(len(id_array)):
        cv.putText(
            img=image,
            text="id: " + id_array[i],
            org=(center_y[i], center_x[i]),
            fontFace=cv.FONT_HERSHEY_SIMPLEX,
            fontScale=1,
            color=(0, 0, 0),
            thickness=2
        )
    
    return image

def put_info(image:cv.Mat) -> None:

    interval = 20

    with open('/home/shopping/src/vision/vision/vision_info.json', 'r') as json_file:
        info_dict = json.load(json_file)
    
    for idx, (key, value) in enumerate(info_dict.items()):

        info = str(idx+1) + ". " + key + ": " + str(value)

        cv.putText(
            img=image,
            text=info,
            org=(10, 15 + idx * interval),
            fontFace=cv.FONT_HERSHEY_SIMPLEX,
            fontScale=0.5,
            color=(0, 0, 0),
            thickness=2
        )
    
    image = put_id(image, info_dict)
    
    return image

def image_show(image_msg:Image) -> None:

    img = cb.imgmsg_to_cv2(image_msg)
    # img = put_info(img)

    cv.imshow(
        winname='vision',
        mat=img
    )
    cv.waitKey(2)

if __name__=='__main__':

    rospy.init_node(name='vision_show')

    img_sub = rospy.Subscriber(
        name='vision_show',
        callback=image_show,
        data_class=Image,
        queue_size=10
    )

    rospy.spin()