import time
import os
from ultralytics import YOLO

class StartTrain:
    def __init__(self,data="", epochs=10, imgsz=640, device=[0,], workers=0, batch=4, cache=True):
        self.data = data
        self.epochs = epochs
        self.imgsz = imgsz
        self.device = device
        self.workers = workers
        self.batch = batch
        self.cache= cache
    
    def start_train(self):
        model = YOLO('yolov8n.pt')
        results = model.train(data=self.data, epochs=self.epochs, imgsz=self.imgsz, device=self.device, workers=self.workers, batch=self.batch, cache=self.cache)
        
class StartVal:
    def __init__(self,data="", imgsz=640, batch=4, conf=0.25, iou=0.6, device="0", workers=0):
        self.data = data
        self.imgsz = imgsz
        self.batch = batch
        self.conf = conf
        self.iou = iou
        self.device = device
        self.workers = workers

    def start_val(self, yolo_model):
        model = YOLO(yolo_model)
        results = model.val(data=self.data, imgsz=self.imgsz, batch=self.batch, conf=self.conf, iou=self.iou, device=self.device, workers=self.workers)

class StartPredict:
    def __init__(self):
        pass

    def start_single_predict(self, yolo_model, img_path):
        model = YOLO(yolo_model)
        results = model(img_path)
        # 获取类别名称字典
        for r in results:
            # 获取类别名称字典
            names = r.names
            print(names)
            # 可视化
            r.show()
            # 遍历每个检测到的对象
            for box in r.boxes:
                # 获取类别索引
                class_id = int(box.cls[0])
                # 获取类别名称
                class_name = names[class_id]
                # 获取置信度
                confidence = float(box.conf[0])
                # 打印结果
                print(f"检测到: {class_name}, 置信度: {confidence:.2f}")
            # 如果没有检测到任何对象
            if len(r.boxes) == 0:
                print("未检测到任何对象")
            time.sleep(3)
    
    def start_several_predict(self, yolo_model, img_folder_path):
        model = YOLO(yolo_model)  
        # 指定图片文件夹路径
        image_folder = img_folder_path
        # 获取文件夹中所有图片文件
        image_files = [f for f in os.listdir(image_folder) if f.lower().endswith(('.png', '.jpg', '.jpeg', '.gif', '.bmp'))]
        # 遍历所有图片文件
        for image_file in image_files:
            image_path = os.path.join(image_folder, image_file)
            print(f"正在处理图片: {image_file}")
            # 开始预测
            results = model(image_path)
            for r in results:
                # 获取类别名称字典
                names = r.names
                print(names)
                # 可视化
                r.show()
                # 遍历每个检测到的对象
                for box in r.boxes:
                    # 获取类别索引
                    class_id = int(box.cls[0])
                    # 获取类别名称
                    class_name = names[class_id]
                    # 获取置信度
                    confidence = float(box.conf[0])
                    # 打印结果
                    print(f"检测到: {class_name}, 置信度: {confidence:.2f}")
                # 如果没有检测到任何对象
                if len(r.boxes) == 0:
                    print("未检测到任何对象")
            print(f"完成处理图片: {image_file}")
            print("----------------------------")
            time.sleep(3)  # 每次完成后暂停3秒
        print("所有图片处理完成")

def main():
    # 修改参数
    # 训练模型的数据为'A_my_data.yaml'，轮数为100，图片大小为640，设备为本地的GPU显卡，关闭多线程的加载，图像加载的批次大小为4，开启图片缓存
    # 如果gpu占用少，可以把batch由4调到16
    data='src/RubbishRecognition/scripts/A_my_data.yaml'
    epochs=10
    imgsz=640
    batch=16
    start_train = StartTrain(data=data, epochs=epochs, imgsz=imgsz, batch=batch)
    start_val = StartVal(data=data, imgsz=imgsz, batch=batch)
    start_predict = StartPredict()
    while(True):
        print("请输入数字选择功能：")
        print("1. 训练模型")
        print("2. 验证模型")
        print("3. 单张图片预测")
        print("4. 多张图片预测")
        print("5. 退出")
        choice = input("请输入数字：")
        if choice == "1":
            start_train.start_train()
        elif choice == "2":
            yolo_model = input("请输入模型路径：")
            start_val.start_val(yolo_model)
        elif choice == "3":
            yolo_model = input("请输入模型路径：")
            img_path = input("请输入图片路径：")
            start_predict.start_single_predict(yolo_model, img_path)
        elif choice == "4":
            yolo_model = input("请输入模型路径：")
            img_folder_path = input("请输入图片文件夹路径：")
            start_predict.start_several_predict(yolo_model, img_folder_path)
        elif choice == "5":
            break
        else:
            print("无效选择，请重新输入")

if __name__ == "__main__":
    main()