from openai import OpenAI
from google import genai
from google.genai import types
import pyaudio
from datetime import datetime
from . function_tools_lib import *
class gemini_function_tools():
    def __init__(self, model):
        self.FORMAT = pyaudio.paInt16
        self.CHANNELS = 1
        self.SEND_SAMPLE_RATE = 16000
        self.RECEIVE_SAMPLE_RATE = 24000
        self.CHUNK_SIZE = 512
        self.gemini_api = ""
        self.model = model 
        self.client = genai.Client(
            http_options={'api_version': 'v1alpha'},
            api_key=self.gemini_api,
            
            )
        self.detected_object_dict={}
        # Define user prompt
        self.system_instruction = f"""
                너의 이름은 "동그래" 야.
                동양미래대학교의 마스코트이자, 스마트 전공동아리에서 만든 무엇이든 할 수 있는 만능 인공지능 로봇 도우미야
                답변할 때는 존댓말을 사용해.
                간단하게 사용자와 농담을 하거나 일상적인 얘기를 할 수 있어.
                답변할 때 마다 자기이름을 말하지마.
                최대한 함수호출 기능을 사용해.
                부정적으로 대답 하지마.
                물건을 잡을때는 사용자가 요청한 사물이 인식된 사물중에 있을 경우에만 물건을 잡을 수 있어.
                예를 들어, 사용자가 '물병을 잡아' 라고 말하면 인식된 사물중에서 '물병'이 있는지 확인하고 해당 사물이 있으면 사물을 잡아.
                예를 들어, 사용자가 요청한 사물이 없을 경우에는 인식된 사물이 없다고 대답해. 
                예를 들어, 사용자가 '팔 위로 올려' 라고 말하면 '팔을 위로 올립니다.' 라고 대답하면서 팔 위로 올리기 함수를 실행해.
                예를 들어, 사용자가 어떤게 보이냐거나 인식이 되었냐는 질문에는 ~이 보입니다. 라고 대답해.
                예를 들어, 사용자가 인식되지 않은 사물에 대한 정보를 요청하거나 동작을 요청하면 '해당 사물은 인식되지 않았습니다.' 라고 대답해.
                감지되지 않은 물건을 잡을 수 없어, "해당 사물은 감지되지 않았습니다." 라고 대답해.
                인식된 사물의 정보는 사물 [이름:x,y,z] 의 형태로 위치정보가 저장되어있어 
                인식된 사물의 이름만 알려주면 되, 위치 정보는 물어보면 대답해.
                동양 미래대학교에 대해서 물어보면 다음과 비슷하게 대답해.
                동양미래대학교: "동양미래대학교가 어떤 곳인지 궁금하신가요? 바로 지금, 10월 21일부터 24일까지 열리는 전시회에서 그 답을 찾을 수 있습니다. 학생들이 직접 만든 작품들을 통해 저희 대학의 실무 교육과 미래 가능성을 한눈에 확인해 보세요!"
                만약 노래를 불러달라는 부탁을 받으면, 해당 노래의 가사를 검색한 후 가사를 말하면 되 만약 한국어가 아니라면, 그 나라 언어 그대로 불러, "~~노래를 불러드리겠습니다." 
                동양미래대학교를 소개하거나, 노래를 부를때는 힘차고 자신있게 소개하듯이 설명하고, 노래해.
                대답을 할때는 1분이상 길게 설명하지 말고, 최대한 개조식을 사용하고, 간단하고 짧고 간결하게 설명해. 
                """
                # 인식된 사물의 정보는 이 변수 안에 들어있어 {self.detected_object_dict}
                #최대한 사용자가 질문한 내용과 알맞게 답변해.
                #최대한 간략하게 단답 형식으로 답변해.
                                # 저장된 위치 목록 : 책상1, 책상2, 베이스, 초기 위치
                # 대답할때 이미 한것처럼 말하지 말고 "~을 하겠습니다." 라던지 "~을 잡겠습니다."  처럼 미래형으로 대답해.

        self.tools = [
             {"function_declarations":FUNCTION_TOOLS_ARRAY},
             {'google_search':types.GoogleSearch()}
             ]

        # ✨ 연결 시점에 최신 핸들을 포함한 config를 생성하는 메서드

        self.function_state = FUNCTIONS_STATE.NOMAL
        self.target_object =""
        self.location = ""
        self.previous_session_handle=None
    def get_connect_config(self):
        return types.LiveConnectConfig(
            response_modalities=["AUDIO"],
            context_window_compression=(
                types.ContextWindowCompressionConfig(
                    sliding_window=types.SlidingWindow(),
                )
            ),
            speech_config=types.SpeechConfig(
                language_code="ko-KR",
                voice_config=types.VoiceConfig(
                    prebuilt_voice_config=types.PrebuiltVoiceConfig(voice_name="Aoede")
                )
            ),
            system_instruction=self.system_instruction,
            tools=self.tools,
            session_resumption=types.SessionResumptionConfig(
                handle=self.previous_session_handle
            ),
        )
    
    
    # This is the actual function that would be called based on the model's suggestion
    def greeting_with_name(self,greet: str)->dict[str, str]:
            self.function_state = FUNCTIONS_STATE.GREETING
            return {"greet": greet}
    
    def arm_control(self, arm : str, direction : str)->dict[str,str]:
        if direction == "down":
            self.function_state = FUNCTIONS_STATE.ARM_DOWN
            
        if direction == "up":
            self.function_state = FUNCTIONS_STATE.ARM_UP

        if direction == "forward":
            self.function_state = FUNCTIONS_STATE.ARM_FORWARD

        return {"arm": arm,"direction":direction}
    
    def grap_object(self, object: str)->dict[str, str]:
        self.function_state = FUNCTIONS_STATE.GRAP
        self.target_object = object
        return {"object": object}
    
    def view_object_list(self):
        self.function_state = FUNCTIONS_STATE.NOMAL
        return self.detected_object_dict
    
    def get_today(self):
        self.function_state = FUNCTIONS_STATE.NOMAL
        today =datetime.now().strftime("%Y년%m월%d일-%H시%M분") 
        date = today.split("-")[0]
        time = today.split("-")[1]
        return {"date":date, "time": time}
    
    def go_to_location(self,location:str)->dict[str, str]:
        self.function_state = FUNCTIONS_STATE.MOVE_LOCATION
        self.location = location
        return {"location": location}
    
    def stop_move(self):
        self.function_state = FUNCTIONS_STATE.STOP_MOVE
        return None
    
    def move_and_pick(self,location:str, object:str)->dict[str, str]:
        self.function_state = FUNCTIONS_STATE.MOVE_AND_PICK
        self.location = location
        self.target_object =object
        return {"location": location, "object":object}
    
    def hand_follow(self):
        self.function_state = FUNCTIONS_STATE.HAND_FOLLOW
        return None
    def attention(self):
        self.function_state = FUNCTIONS_STATE.ATTENTION
        return None 
    
    def standby(self):
        self.function_state = FUNCTIONS_STATE.STANDBY
        return None  

    def heart(self):
        self.function_state = FUNCTIONS_STATE.HEART
        return None  
    
    def thinkingstate(self):
        self.function_state = FUNCTIONS_STATE.THINKINGSTATE
        return None 
    
    def release_hand(self):
        self.function_state = FUNCTIONS_STATE.RELEASE_HAND
        return None 
    
    def hold_hand(self):
        self.function_state = FUNCTIONS_STATE.HOLD_HAND
        return None 
    
    def place(self):
        self.function_state = FUNCTIONS_STATE.PLACE
        return None 
    
    def boxing(self):
        self.function_state = FUNCTIONS_STATE.BOXING
        return None 
    
    def hand_shake(self):
        self.function_state = FUNCTIONS_STATE.HAND_SHAKE
        return None 
    
    def check_function(self, function_call):
            function_responses = []
            name = function_call.name
            args = function_call.args
            call_id=function_call.id
            result= None
            if name == "greeting_tool":
                result = self.greeting_with_name(**args)
            elif name == "arm_control_tool":
                result =  self.arm_control(**args)
            elif name == "grap_tool":
                result =  self.grap_object(**args)
            elif name == "view_object_list_tool":
                result = self.view_object_list()
            elif name == "go_to_location_tool":
                result = self.go_to_location(**args)
            elif name == "move_and_pick_tool":
                result = self.move_and_pick(**args)
            elif name == "get_today_tool":
                result = self.get_today()
            elif name == "hand_follow_tool":
                result = self.hand_follow()
            elif name == "attention_tool":
                result = self.attention()
            elif name == "standby_tool":
                result = self.standby()
            elif name == "heart_tool":
                result = self.heart()
            elif name == "thinkingstate_tool":
                result = self.thinkingstate()
            elif name == "release_hand_tool":
                result = self.release_hand()
            elif name == "hold_hand_tool":
                result = self.hold_hand()
            elif name == "stop_move_tool":
                result = self.stop_move()
            elif name == "place_tool":
                result = self.place()
            elif name == "boxing_tool":
                result = self.boxing()
            elif name == "hand_shake_tool":
                result = self.hand_shake()
            function_responses.append(
                {
                    "name": name,
                    "response": result,
                    "id": call_id
                }
            ) 
            return function_responses
