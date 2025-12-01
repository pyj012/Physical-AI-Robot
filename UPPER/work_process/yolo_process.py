import os,time,cv2
import numpy as np
from ultralytics import YOLO, solutions
import traceback
from PIL import ImageFont, ImageDraw, Image
import random
from lib.realsense_lib import *
from lib.servo_control_lib import *
os.environ['YOLO_VERBOSE'] = 'True'
class YOLOframework():
    def __init__(self): 
        print("Loading YOLO Process")
        load_start= time.time()
        self.yolo_model = YOLO("yolo11n.engine")
        self.yolo_model.predict(source="0", show=False, stream=True, verbose=False, classes=[39,40,77])#  # [0, 3, 5] for multiple classes
        self.class_mapping = {
                39:'물병',
                41:'컵',
                77:'곰인형',
                # 47:'사과',
                # 60:'책상',
                # 62:'티비',
                # 63:'노트북',
                # 64:'마우스',
                # 67:'핸드폰',
                # 68:'전자레인지',
                # 72:'냉장고',
            }
        self.dected_time={}
        self.refresh_time = 1
        self.prev_refresh_time = 0
        self.yolo_classes = list(self.yolo_model.names.values())
        self.classes_ids = [self.yolo_classes.index(clas) for clas in self.yolo_classes]
        self.colors = {cls_id: random.choices(range(256), k=3) for cls_id in self.classes_ids}    
        self.realsense =RealsenseCamera()
        self.servo_motor = None
        print("Load YOLO Process Complete : ",time.time()-load_start )


    def Detect(self):
        resultes_img=None
        object_pos_dict = None
        resultes_img
        try:
            color_img ,depth_frame =self.realsense.getFrame()
            if color_img is not None and depth_frame is not None:
                resultes= self.yolo_model(color_img) 
                resultes_img,  detected_class_dict =self.ResulteDissembly(resultes)
                object_pos_dict=self.realsense.getDistance(depth_frame,  detected_class_dict)
                return (resultes_img, object_pos_dict)
            
            else:
                return (resultes_img, object_pos_dict)

        except Exception as e:
            print("detect err", e)
    
    def ResulteDissembly(self, resultes):
        # 감지된 객체들의 정보가 담긴 딕셔너리
        self.detected_class_dict={}
        result_dict = resultes[0]
        resultes_img = result_dict.plot()
        markerd_img=self.MarkClassCenter(resultes_img)
        highest_confidence_object = None
        max_confidence = -1
        # if result_dict and len(result_dict[0].boxes) > 0:
        #     for box_info in result_dict[0].boxes:
        #         current_confidence = box_info.conf.item() # tensor에서 float으로 변환
        #         if current_confidence > max_confidence:
        #             max_confidence = current_confidence
        #             highest_confidence_object = {
        #                 'box': box_info.xyxy[0].tolist(), # [x1, y1, x2, y2]
        #                 'class_id': int(box_info.cls.item()), # 클래스 ID
        #                 'confidence': current_confidence
        #             }

        # if highest_confidence_object:
        #     class_id = highest_confidence_object['class_id']
        #     if class_id in self.class_mapping: # 클래스 ID가 결과에 있는지 확인합니다
        #         result_dict.names[class_id] = self.class_mapping[class_id] # 클래스 이름을 사용자 지정 레이블로 대체합니다.
        #     #현재 객체의 이름을 가져온다.
        #     class_name = result_dict.names[class_id]
        #     #감지된 객체들에서 좌표 추출
        #     xyxylist = highest_confidence_object['box']
        #     x1 = round(xyxylist[0])
        #     y1 = round(xyxylist[1])
        #     x2 = round(xyxylist[2])
        #     y2 = round(xyxylist[3])
        #     y_offs = 1.9
        #     center_x = round(x2-((x2-x1)/2))
        #     center_y = round(y2-((y2-y1)/y_offs))
        #     self.detected_class_dict[class_name] = (center_x,center_y)
        #     self.dected_time[class_name] = time.time()

        if len(result_dict) > 0:
            for object in result_dict:
                for box_info in object.boxes:
                    current_confidence = box_info.conf.item() # tensor에서 float으로 변환
                    max_confidence = current_confidence
                    highest_confidence_object = {
                        'box': box_info.xyxy[0].tolist(), # [x1, y1, x2, y2]
                        'class_id': int(box_info.cls.item()), # 클래스 ID
                        'confidence': current_confidence
                    }
                    class_id = highest_confidence_object['class_id']
                    if class_id in self.class_mapping: # 클래스 ID가 결과에 있는지 확인합니다
                        result_dict.names[class_id] = self.class_mapping[class_id] # 클래스 이름을 사용자 지정 레이블로 대체합니다.
                    #현재 객체의 이름을 가져온다.
                    class_name = result_dict.names[class_id]
                    #감지된 객체들에서 좌표 추출
                    xyxylist = highest_confidence_object['box']
                    x1 = round(xyxylist[0])
                    y1 = round(xyxylist[1])
                    x2 = round(xyxylist[2])
                    y2 = round(xyxylist[3])
                    y_offs = 1.9
                    center_x = round(x2-((x2-x1)/2))
                    center_y = round(y2-((y2-y1)/y_offs))
                    self.detected_class_dict[class_name] = (center_x,center_y)
                    self.dected_time[class_name] = time.time()

        try:
            for name in self.dected_time.copy().keys():
                current_time = self.dected_time[name]
                if time.time()-current_time >= self.refresh_time :
                    self.dected_time.pop(name)
                    self.detected_class_dict.pop(name)

        except Exception as e:
            pass
        
        

        return markerd_img, self.detected_class_dict
    
    def MarkClassCenter(self, img):
        try:
            for classes in self.detected_class_dict:
                x,y = self.detected_class_dict[classes]
                cv2.line(img,(x,y),(x,y),(255,255,0),10)
            return img
        except Exception as e:
            print(e)
            pass
