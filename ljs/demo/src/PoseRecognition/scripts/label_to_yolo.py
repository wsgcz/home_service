import os
import json
import shutil
import random
import argparse
from tqdm import tqdm
from collections import Counter
import yaml

# 关键点名字、数量
keypoint_class = ['0','1','2','3','4','5','6','7','8','9','10','11','12','13','14','15','16']
# 物体的类名
bbox_class = {
    'stand': 0
}
 
 
def get_classes(json_dir):
    '''
    统计路径下 JSON 文件里的各类别标签数量
    '''
    names = []
    json_files = [os.path.join(json_dir, f) for f in os.listdir(json_dir) if f.endswith('.json')]
 
    for json_path in json_files:
        with open(json_path, 'r') as f:
            data = json.load(f)
            for shape in data['shapes']:
                name = shape['label']
                if name in bbox_class:
                    names.append(name)
                else:
                    continue
 
    result = Counter(names)
    return result
 
 
def process_single_json(labelme_path, save_folder):
    with open(labelme_path, 'r', encoding='utf-8') as f:
        labelme = json.load(f)
 
    img_width = labelme['imageWidth']
    img_height = labelme['imageHeight']
 
    suffix = labelme_path.split('.')[-2]
    yolo_txt_path = suffix + '.txt'
 
    with open(yolo_txt_path, 'w', encoding='utf-8') as f:
 
        for each_ann in labelme['shapes']:
 
            if each_ann['shape_type'] == 'rectangle':
 
                yolo_str = ''
 
                bbox_class_id = bbox_class[each_ann['label']]
                yolo_str += '{} '.format(bbox_class_id)
 
                bbox_top_left_x = int(min(each_ann['points'][0][0], each_ann['points'][1][0]))
                bbox_bottom_right_x = int(max(each_ann['points'][0][0], each_ann['points'][1][0]))
                bbox_top_left_y = int(min(each_ann['points'][0][1], each_ann['points'][1][1]))
                bbox_bottom_right_y = int(max(each_ann['points'][0][1], each_ann['points'][1][1]))
 
                bbox_center_x = int((bbox_top_left_x + bbox_bottom_right_x) / 2)
                bbox_center_y = int((bbox_top_left_y + bbox_bottom_right_y) / 2)
 
                bbox_width = bbox_bottom_right_x - bbox_top_left_x
 
                bbox_height = bbox_bottom_right_y - bbox_top_left_y
 
                bbox_center_x_norm = bbox_center_x / img_width
                bbox_center_y_norm = bbox_center_y / img_height
 
                bbox_width_norm = bbox_width / img_width
 
                bbox_height_norm = bbox_height / img_height
 
                yolo_str += '{:.5f} {:.5f} {:.5f} {:.5f} '.format(bbox_center_x_norm, bbox_center_y_norm,
                                                                  bbox_width_norm, bbox_height_norm)
 
                bbox_keypoints_dict = {}
                for each_ann in labelme['shapes']:
                    if each_ann['shape_type'] == 'point':
                        x = int(each_ann['points'][0][0])
                        y = int(each_ann['points'][0][1])
                        label = each_ann['label']
                        if (x > bbox_top_left_x) & (x < bbox_bottom_right_x) & (y < bbox_bottom_right_y) & (
                                y > bbox_top_left_y):
                            bbox_keypoints_dict[label] = [x, y]
 
                for each_class in keypoint_class:
                    if each_class in bbox_keypoints_dict:
                        keypoint_x_norm = bbox_keypoints_dict[each_class][0] / img_width
                        keypoint_y_norm = bbox_keypoints_dict[each_class][1] / img_height
                        yolo_str += '{:.5f} {:.5f} {} '.format(keypoint_x_norm, keypoint_y_norm, 2)
                    else:
                        yolo_str += '0 0 0 '
                f.write(yolo_str + '\n')
 
    shutil.move(yolo_txt_path, save_folder)
 
 
def json_get_class(dataset_path, output_path):
    # 统计路径下 JSON 文件里的各类别标签数量
    obj_classes = get_classes(dataset_path)
    classes = list(obj_classes.keys())
 
    # 编写yaml文件
    classes_txt = {i: classes[i] for i in range(len(classes))}  # 标签类别
    data = {
        'path': os.path.join(os.getcwd(), output_path),
        'train': "images/train",
        'val': "images/val",
        'names': classes_txt,
        'nc': len(classes)
    }
 
    with open(output_path + '/pose.yaml', 'w', encoding="utf-8") as file:
        yaml.dump(data, file)
    print("标签：", dict(obj_classes))
 
 
if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="labelme annotation to yolo dataset.")
 
    # input params
    parser.add_argument('--data_path', type=str, default=r"/home/shanhe/yolo_pose_train", help='dataset folder path to convert: ')
    parser.add_argument('--output_path', type=str, default=r"/home/shanhe/demo/src/PoseRecognition/scripts/myyolo", help='output folder path to save: ')
    args = parser.parse_args()
    dataset_path = args.data_path
    output_path = args.output_path
 
    print("dataset path: ", dataset_path)
    print("output_path path: ", output_path)
 
    if not os.path.isdir(output_path):
        os.makedirs(output_path)
    if not os.path.isdir(os.path.join(output_path, 'images', 'train')):
        os.makedirs(os.path.join(output_path, 'images', 'train'))
    if not os.path.isdir(os.path.join(output_path, 'images', 'val')):
        os.makedirs(os.path.join(output_path, 'images', 'val'))
    if not os.path.isdir(os.path.join(output_path, 'labels', 'train')):
        os.makedirs(os.path.join(output_path, 'labels', 'train'))
    if not os.path.isdir(os.path.join(output_path, 'labels', 'val')):
        os.makedirs(os.path.join(output_path, 'labels', 'val'))
 
    # select train and val dataset
    os.chdir(dataset_path)
 
    json_files = list(filter(lambda x: '.json' in x, os.listdir()))
    json_ext = json_files[0].split(".")[-1]
 
    img_files = list(filter(lambda x: '.json' not in x, os.listdir()))
    img_ext = img_files[0].split(".")[-1]
 
    files_without_ext = [file.split(".")[0] for file in json_files]
 
    frac = 0.2
    random.seed(123)
    val_num = int(len(files_without_ext) * frac)
    train_files = files_without_ext[val_num:]
    val_files = files_without_ext[:val_num]
 
    print("Total: ", len(files_without_ext))
    print("Train: ", len(train_files))
    print("Value: ", len(val_files))
 
    # copy to output file
    for train_file in tqdm(train_files):
        img_train_file = train_file + '.' + img_ext
        json_train_file = train_file + '.' + json_ext
        shutil.copy(img_train_file, os.path.join(output_path, 'images', 'train'))
        process_single_json(json_train_file, os.path.join(output_path, 'labels', 'train'))
 
    for val_file in tqdm(val_files):
        img_val_file = val_file + '.' + img_ext
        json_val_file = val_file + '.' + json_ext
        shutil.copy(img_val_file, os.path.join(output_path, 'images', 'val'))
        process_single_json(json_val_file, os.path.join(output_path, 'labels', 'val'))
 
    json_get_class(dataset_path, output_path)
 
    print("Successful!")

# 结束后再yaml末尾加入：
"""
kpt_shape: [17, 3] # number of keypoints, number of dims (2 for x,y or 3 for x,y,visible)
flip_idx: [0, 2, 1, 4, 3, 6, 5, 8, 7, 10, 9, 12, 11, 14, 13, 16, 15]
"""