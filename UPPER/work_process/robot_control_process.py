import time
import rclpy
import traceback
from lib.function_tools_lib import *
from lib.angle_solver_lib import *
from lib.ik_solver_lib import *
WAIT = 0
MOVEING = 1
ARRIVED = 2

class RobotControl():
    def __init__(self,  FunctionQueue, GripQueue,TargetGoalQ, MoveStateQ, scenalioQ):
        print("Loading Robot Control Process")
        load_start= time.time()
        rclpy.init()
        self.FunctionQueue =FunctionQueue
        self.GripQueue=GripQueue
        self.MoveStateQ = MoveStateQ
        self.TargetGoalQ = TargetGoalQ
        self.scenalioQ=scenalioQ
        self.function_state=FUNCTIONS_STATE.NOMAL
        self.detected_object_dict= {}
        self.target_object=""
        self.info = None
        self.location = ""
        self.grap_sequence=0
        self.place_grap_sequence=0
        self.move_sequence=0
        self.waitforgraptime= 1.5
        self.waitfornexttime=2.2
        self.prev_time = 0
        self.iksolver = GadianRRTPlanner()
        self.iksolver.max_acceleration_scaling_factor = 0.2
        self.iksolver.max_velocity_scaling_factor = 0.2
        self.anglesolver = JointSpaceMoveGroupExecutor()
        self.robot_control_dict ={}
        self.move_and_pick_time=0
        self.current_arm = ""
        self.chech_time =0
        self.PrevMoveRetryTime = 0
        self.MoveRetryTime=5
        self.PrevMoveStateCheckTime = 0
        self.MoveStateCheckTime = 1
        self.move_and_pick_seq=0
        self.move_state=WAIT
        self.WaitForCancelTime=120
        self.PrevCancelTime=0
        self.Stop_Seq=0
        self.scenalio_dict={}
        self.arm_group=None

        self.arm_info_dict={"left":[0,0,0,0,0,0], "right":[0,0,0,0,0,0], "speed":1}
        print("Load Robot Control Complete : ",time.time()-load_start )
        while True:
            if self.scenalioQ.qsize()>0:
                self.scenalio_dict.update(self.scenalioQ.get())
            if self.FunctionQueue.qsize()>0:
                try:
                    self.robot_control_dict  = self.FunctionQueue.get()
                    self.function_state=self.robot_control_dict['function_state']
                    self.detected_object_dict=self.robot_control_dict['detected_object_dict']
                    self.target_object = self.robot_control_dict['target_object']
                    self.location = self.robot_control_dict['location']
                    if "1" in self.location :
                        self.location = "table1"
                    elif "2" in self.location:
                        self.location = "table2"
                    elif "초기" in self.location or "시작" in self.location or "베이스" in self.location:
                        self.location = "base"

                    
                except:
                    pass
                # robot_control_dict={
                #     'target_object':self.gemini.target_object,
                #     'detected_object_dict':self.gemini.detected_object_dict,
                #     'function_state' :self.gemini.function_state,
                #     'location' : self.gemini.location
                # }
            if self.MoveStateQ.qsize()>0:
                receiveMessage =self.MoveStateQ.get() 
                # print(receiveMessage)
                try:
                    self.move_state=receiveMessage['move_state']
                except Exception as e:
                    print(e)
                    pass

            try:        
                if not self.function_state==FUNCTIONS_STATE.NOMAL:
                    if self.function_state== FUNCTIONS_STATE.GREETING:
                        self.greeting()
                        self.function_state=FUNCTIONS_STATE.NOMAL

                    elif self.function_state== FUNCTIONS_STATE.ARM_UP:
                        print("ARM_UP")
                        # self.iksolver.move_to_pose(0.186, -0.120, 1.288)
                        self.function_state=FUNCTIONS_STATE.NOMAL

                    elif self.function_state== FUNCTIONS_STATE.ATTENTION:
                        self.attention()
                        self.function_state=FUNCTIONS_STATE.NOMAL

                    elif self.function_state== FUNCTIONS_STATE.STANDBY:
                        self.standby()
                        self.function_state=FUNCTIONS_STATE.NOMAL

                    elif self.function_state== FUNCTIONS_STATE.HEART:
                        self.heart()
                        self.function_state=FUNCTIONS_STATE.NOMAL

                    elif self.function_state== FUNCTIONS_STATE.THINKINGSTATE:
                        self.thinkingstate()
                        self.function_state=FUNCTIONS_STATE.NOMAL
                        
                    elif self.function_state== FUNCTIONS_STATE.RELEASE_HAND:
                        self.release_hand()
                        self.function_state=FUNCTIONS_STATE.NOMAL
                    elif self.function_state== FUNCTIONS_STATE.HOLD_HAND:
                        self.hold_hand()
                        self.function_state=FUNCTIONS_STATE.NOMAL

                    elif self.function_state == FUNCTIONS_STATE.BOXING:
                        result =self.boxing()
                        if result is not None:
                            self.function_state=FUNCTIONS_STATE.NOMAL  
                    
                    elif self.function_state == FUNCTIONS_STATE.HAND_SHAKE:
                        result =self.hand_shake()
                        if result is not None:
                            self.function_state=FUNCTIONS_STATE.NOMAL  

                    elif self.function_state== FUNCTIONS_STATE.GRAP:
                        result =self.grap_object()
                        if result is not None:
                            self.function_state=FUNCTIONS_STATE.NOMAL

                    elif self.function_state == FUNCTIONS_STATE.MOVE_LOCATION:
                        result =self.move_location()
                        if result is not None:
                            self.function_state=FUNCTIONS_STATE.NOMAL

                    elif self.function_state == FUNCTIONS_STATE.MOVE_AND_PICK:
                        if self.move_and_pick_seq == 0:
                            goal_result =self.move_location()
                            if goal_result is not None:
                                self.move_and_pick_seq+=1
                                self.move_and_pick_time=time.time()

                        elif self.move_and_pick_seq == 1:
                            if time.time()- self.move_and_pick_time>=1:
                                grap_result = self.grap_object(scenario=True)
                                if grap_result is not None:
                                    self.move_and_pick_seq+=1
                                    self.location="table2"

                        elif self.move_and_pick_seq == 2:
                            return_result =self.move_location()
                            if return_result is not None:
                                self.move_and_pick_seq+=1
                                self.move_and_pick_time=time.time()
                        elif self.move_and_pick_seq == 3:
                            if time.time()- self.move_and_pick_time>=1:
                                return_result =self.place_object()
                                if return_result is not None:
                                    self.move_and_pick_seq=0
                                    self.function_state=FUNCTIONS_STATE.NOMAL
                        

                    elif self.function_state==FUNCTIONS_STATE.STOP_MOVE:
                        if self.Stop_Seq==0:
                            self.location= "cancel"
                            self.TargetGoalQ.put(self.location)
                            print("!!!!SEND STOP!!!!!")
                            self.move_sequence=0
                            self.Stop_Seq+=1
                            self.PrevCancelTime= time.time()

                        elif self.Stop_Seq==1:
                            if time.time()-self.PrevCancelTime>=3:
                                self.location= ""             
                                self.Stop_Seq=0
                                self.TargetGoalQ.put(self.location)

                    elif self.function_state== FUNCTIONS_STATE.PLACE:
                        return_result = self.place_object()
                        if return_result is not None:
                            self.function_state=FUNCTIONS_STATE.NOMAL      
                        

                else:
                    self.grap_sequence =0
                    self.move_sequence =0
                    self.place_grap_sequence =0
                    self.move_and_pick_seq=0
                time.sleep(0.001)
            except :
                print("RobotControl ERR :")
                print(traceback.format_exc())
                rclpy.shutdown()

    def place_object(self):
        if self.place_grap_sequence == 0:
            self.place(self.current_arm)
            self.place_grap_sequence+=1
            self.prev_time = time.time()

        elif self.place_grap_sequence == 1:
            if time.time()-self.prev_time >= 4:
                self.GripQueue.put(('L',False))
                self.GripQueue.put(('R',False)) 
                time.sleep(1)
                self.standby() 
                self.place_grap_sequence=0    
                return True
            
    def grap_object(self, scenario=False):
        # print("######GRAP PROCESS :", self.grap_sequence)
        try:
            if self.grap_sequence == 0:
                    print("###Ready GRAPW###")
                    self.move_to_standby()
                    self.GripQueue.put(('L',False))
                    self.GripQueue.put(('R',False))
                    self.grap_sequence+=1
                    self.prev_time = time.time()

            elif self.grap_sequence == 1:
                try:
                    if time.time()-self.prev_time >= 5:
                        self.iksolver.max_acceleration_scaling_factor = 0.15
                        self.iksolver.max_velocity_scaling_factor = 0.15
                        if scenario:
                            print("SCENARIO !!")
                            x,y,z = self.scenalio_dict[self.target_object]
                            result, self.arm_group= self.iksolver.plan_and_execute(x+0.0, y+0.0, z+0.0)
                        else:   
                            print("NOMAL !!")
                            x,y,z = self.detected_object_dict[self.target_object]
                            result, self.arm_group= self.iksolver.plan_and_execute(x+0.0, y+0.0, z+0.0)
                        print("### ARM GRUP== ", self.arm_group)
                        self.current_arm=self.arm_group["group"]
                        print("#### COMPLET MOTION PLANNING###",self.current_arm)
                        if self.current_arm == "left_arm":
                            self.GripQueue.put(('L',False))
                        else : 
                            self.GripQueue.put(('R',False))
                        self.chech_time= time.time()
                        self.prev_time = time.time()
                        self.grap_sequence+=1
                except Exception as e:
                    print("CURRENT ARM ERR", e)
                    pass

            elif self.grap_sequence == 2:
                if time.time()-self.prev_time >= self.waitforgraptime:
                    print("###GRAP NOW###")
                    if self.current_arm == "left_arm":
                        self.GripQueue.put(('L',True))
                    else : 
                        self.GripQueue.put(('R',True))
                    print("걸린시간",time.time()-self.chech_time)
                    self.prev_time = time.time()
                    self.grap_sequence +=1
                    
            elif self.grap_sequence ==3:
                if time.time()-self.prev_time >= self.waitfornexttime:
                    # print("###Ready GRAPW###")
                    self.standby()
                    # self.GripQueue.put(('L',False))
                    # self.GripQueue.put(('R',False))
                    self.grap_sequence=0
                    return True

        except Exception as e:
            print("grap err",e)
            pass

    def move_location(self):
        if time.time()- self.PrevMoveStateCheckTime >= self.MoveStateCheckTime:
            self.PrevMoveStateCheckTime= time.time()
            print("!!! CURREMT MOVE STATE :", self.move_state)
            print("move_sequence : ", self.move_sequence)
            print("location : ", self.location)
            print("Left Time To Cancel...", round(self.WaitForCancelTime-(time.time()-self.PrevCancelTime)))

        if self.move_sequence==0:
            if time.time()-self.PrevMoveRetryTime >=self.MoveRetryTime:
                if self.move_state == WAIT:
                    print("SEND GOAL NOW")
                    self.TargetGoalQ.put(self.location) 
                    self.PrevMoveRetryTime=time.time()
                    self.PrevCancelTime=time.time()
                    self.move_sequence+=1
                else:
                    print("LOWER is Busy, Try Later")
                    self.PrevMoveRetryTime=time.time()

        elif self.move_sequence==1:
            if time.time()-self.PrevCancelTime>=self.WaitForCancelTime:
                self.PrevCancelTime= time.time()
                self.location= "cancel"
                self.TargetGoalQ.put(self.location)
                self.location= ""
                self.move_sequence=0
                return True

            if self.move_state == ARRIVED:
                print("TAGETGOAL ARRIVED!!")
                self.move_sequence=0
                self.location= ""
                return True

    def greeting(self):
        ok=False
        self.arm_info_dict["speed"]=0.3
        left_arm=[[20, 10, -40, 90, 0, 20],[-130,30,30,55,-50,35],[-130,40,-30,55,-90,-15],[-130,30,30,55,-50,35],[-130,40,-30,55,-90,-15],[20, 10, -40, 90, 0, 20],[0,0,0,0,0,0]]
        for sequence in range(0,len(left_arm)):
            # if sequence >1 and sequence < len(left_arm)-1:
            #     self.arm_info_dict["speed"]=0.5
            #     time.sleep(1.2)

            # else:
            #     self.arm_info_dict["speed"]=0.8
                

            self.arm_info_dict["left"]=left_arm[sequence]
            ok=self.anglesolver.move_with_angle(self.arm_info_dict)     
            # time.sleep(5)  
        return ok
    
    def release_hand(self):
        ok=False
        self.GripQueue.put(('L',False))
        self.GripQueue.put(('R',False))
        return ok
    
    def hold_hand(self):
        ok=False
        self.GripQueue.put(('L',True))
        self.GripQueue.put(('R',True))
        return ok
    
    def standby(self):
        ok=False
        left_arm=[[10, 10, 0, 120, 0, 15]]
        right_arm=[[10, -10, 0, -120, 0, 15]]
        self.arm_info_dict["speed"]=1.0
        for sequence in range(0,len(right_arm)):
            self.arm_info_dict["left"]=left_arm[sequence]
            self.arm_info_dict["right"]=right_arm[sequence]
            ok=self.anglesolver.move_with_angle(self.arm_info_dict)
            time.sleep(0.1)
        return ok
    
    def move_to_standby(self):
        ok=False
        left_arm=[[20, 10, -55, 110, 0, 25],[20, 10, 0, 120, 0, 10]]
        right_arm=[[20, -10, 55, -110, 0, 25],[20, -10, 0, -120, 0, 10]]
        self.arm_info_dict["speed"]=1.0
        for sequence in range(0,len(right_arm)):
            self.arm_info_dict["left"]=left_arm[sequence]
            self.arm_info_dict["right"]=right_arm[sequence]
            ok=self.anglesolver.move_with_angle(self.arm_info_dict)
            time.sleep(0.1)
        return ok
    
    def attention(self):
        ok=False
        left_arm=[[20, 10, -40, 90, 0, 20],[0,0,0,0,0,0]]
        right_arm=[[20, -10, 40, -90, 0, 20],[0,0,0,0,0,0]]
        self.arm_info_dict["speed"]=0.7
        for sequence in range(0,len(right_arm)):
            self.arm_info_dict["left"]=left_arm[sequence]
            self.arm_info_dict["right"]=right_arm[sequence]
            ok=self.anglesolver.move_with_angle(self.arm_info_dict)     
        return ok
    
    def heart(self):
        ok=False
        left_arm=[[20, 10, -40, 90, 0, 20],[-17, 138, -60, 68, -1, 90]]
        right_arm=[[20, -10, 40, -90, 0, 20],[-17, -138, 60, -68, 0, 90]]
        self.arm_info_dict["speed"]=0.3
        for sequence in range(0,len(right_arm)):
            self.arm_info_dict["left"]=left_arm[sequence]
            self.arm_info_dict["right"]=right_arm[sequence]
            ok=self.anglesolver.move_with_angle(self.arm_info_dict)
        return ok
    
    def thinkingstate(self):
        ok=False
        self.arm_info_dict["left"]=[-70, 0, -20, 120, -60, 60]
        self.arm_info_dict["right"]=[-40, 15, -60, -55, 70, 15]
        self.arm_info_dict["speed"]=0.4
        ok=self.anglesolver.move_with_angle(self.arm_info_dict)
        return ok
    
    def place(self, hand):
        ok=False
        if hand == "left_arm":
            self.arm_info_dict["left"]=[-10, -2, 20, 92, 5, 5]
        else:
            self.arm_info_dict["right"]=[-10, 15, -16, -100, 5, -2]
        self.arm_info_dict["speed"]=0.3
        ok=self.anglesolver.move_with_angle(self.arm_info_dict)
        return ok
    
    def boxing(self):
        ok=False
        left_arm=[[20, 10, -40, 90, 0, 20],[-40, 20, 15, 120, 0, 20],[-85, 5, 10, 20, 0, 0],[-40, 20, 15, 120, 0, 20],[-40, 20, 15, 120, 0, 20],[50, 5, -40, 100, 0, 0],[20, 10, -40, 90, 0, 20],[0,0,0,0,0,0]]
        right_arm=[[20, -10, 40, -90, 0, 20],[-40, -20, -15, -120, 0, 20],[-80, -20, -15, -120, 0, 20],[-85, -5, -10, -20, 0, 0],[-40, -20, -15, -120, 0, 20],[50, -5, 40, -100, 0, 0],[20, -10, 40, -90, 0, 20],[0,0,0,0,0,0]]
        self.arm_info_dict["speed"]=0.5
        for sequence in range(0,len(right_arm)):
            self.arm_info_dict["left"]=left_arm[sequence]
            self.arm_info_dict["right"]=right_arm[sequence]
            ok=self.anglesolver.move_with_angle(self.arm_info_dict)
            if sequence < 2:
                time.sleep(0.5)
            if sequence >2 and sequence < 5 :
                time.sleep(4)
            else:
                time.sleep(1)
        return ok
    
    def hand_shake(self):
        ok=False
        left_arm=[[20, 10, -40, 90, 0, 20],[-80, 85, -5, 90, -10, 50],[-80, 85, -5, 90, -10, -50],[-80, 85, -5, 90, -10, 50],[-80, 85, -5, 90, -10, -50],[20, 10, -40, 90, 0, 20],[0,0,0,0,0,0]]
        right_arm=[[20, -10, 40, -90, 0, 20],[-80,- 85, 5, -90, 10, 50],[-80, -85, 5, -90, 10, -50],[-80, -85, 5, -90, -10, 50],[-80, -85, 5, -90, -10, -50],[20, -10, -40, 90, 0, 20],[0,0,0,0,0,0]]
        self.arm_info_dict["speed"]=0.5
        for sequence in range(0,len(right_arm)):
            self.arm_info_dict["left"]=left_arm[sequence]
            self.arm_info_dict["right"]=right_arm[sequence]
            ok=self.anglesolver.move_with_angle(self.arm_info_dict)
            if sequence < 2:
                time.sleep(0.5)
            else:
                time.sleep(1.8)
        return ok