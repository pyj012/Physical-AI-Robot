from enum import Enum
from google.genai import types

VIEW_OBJEECT_LIST_TOOL={
    "name": "view_object_list_tool",
    "description": "현재 감지된 사물 정보들을 사용자에게 알려준다. 결과는 'object' 키를 가진 JSON 객체로 반환된다.",
}

GREETING_WITH_NAME_TOOL= {
    "name": "greeting_tool",
    "description": "사용자와 인사한다.",
    "parameters": {
        "type": "object",
        "properties": {
            "greet": 
            {   
                "type": "string",
                "description": "사용자가 인사한다.",
            },
        },
        "required": ["greet"],
    },
}
ARM_CONTROL_TOOL={
    "name": "arm_control_tool",
    "description": "사용자의 명령에 따라 팔을 앞, 위, 아래로 움진인다.",
    "parameters": {
        "type": "object",
        "properties": {
            "arm": 
            {   
                "type": "string",
                "description": "움직일 팔을 결정한다 'left' or 'right' .",
            },
            "direction": 
            {   
                "type": "string",
                "description": "움직일 팔의 방향을 결정한다 'up' or 'down' or 'forward' .",
            },
        },
        "required": ["arm","direction"],
    },
}
GRAP_TOOL={
    "name": "grap_tool",
    "description": "사물을 향에 팔을 뻗어 물건을 잡는다.",
    "parameters": {
        "type": "object",
        "properties": {
            "object": 
            {   
                "type": "string",
                "description": "수집할 사물이다",
            },
        },
        "required": ["object"],
    },
}
GO_TO_LOCATION={
    "name": "go_to_location_tool",
    "description": "사용자가 지정한 위치로 이동한다.",
    "parameters": {
        "type": "object",
        "properties": {
            "location": 
            {   
                "type": "string",
                "description": "이동할 장소.",
            },
        },
        "required": ["location"],
    },
}

MOVE_AND_PICK_TOOL={
    "name": "move_and_pick_tool",
    "description": "사용자가 지정한 위치로 이동한 뒤 물건을 잡는다.",
    "parameters": {
        "type": "object",
        "properties": {
            "location": 
            {   
                "type": "string",
                "description": "이동할 장소.",
            },
            "object": 
            {   
                "type": "string",
                "description": "수집할 사물이다",
            },
        },
        "required": ["location","object"],
    },
}
STOP_MOVE_TOOL ={
    "name": "stop_move_tool",
    "description": "이동을 멈춘다.",
}
GET_TODAY ={
    "name": "get_today_tool",
    "description": "현재 날짜와 시간을 가져온다. 결과는 'date'와 'time' 키를 가진 JSON 객체로 반환된다.",
}
HAND_FOLLOW_TOOL ={
    "name": "hand_follow_tool",
    "description": "팔을 뻗어 사용자의 손을 따라한다,",
}
ATTENTION_TOOL ={
    "name": "attention_tool",
    "description": "양 팔을 아래로 내린다.",
}
STANDBY_TOOL ={
    "name": "standby_tool",
    "description": "양 팔을 준비자세로 만든다.",
}
HEART_TOOL ={
    "name": "heart_tool",
    "description": "양 팔을 하트자세로 만든다.",
}
THINKINGSTATE_TOOL ={
    "name": "thinkingstate_tool",
    "description": "양 팔을 고민하는자세로 만든다.",
}
RELEASE_HAND_TOOL ={
    "name": "release_hand_tool",
    "description": "양 손을 펼펴 손바닥을 보인다.",
}
HOLD_HAND_TOOL ={
    "name": "hold_hand_tool",
    "description": "양 손을 쥐어 주먹을 만든다.",
}
PLACE_TOOL ={
    "name": "place_tool",
    "description": "물건을 책상위에 내려놓는다.",
}
BOXING_TOOL ={
    "name": "boxing_tool",
    "description": "양 팔을 복싱자세로 만든다.",
}
HAND_SHAKE_TOOL ={
    "name": "hand_shake_tool",
    "description": "양 손을 흔드는 자세로 만든다.",
}
FUNCTION_TOOLS_ARRAY=[
    GREETING_WITH_NAME_TOOL,
    ARM_CONTROL_TOOL,
    GRAP_TOOL,
    VIEW_OBJEECT_LIST_TOOL,
    GO_TO_LOCATION,
    MOVE_AND_PICK_TOOL,
    GET_TODAY,
    ATTENTION_TOOL,
    STANDBY_TOOL,
    HEART_TOOL,
    THINKINGSTATE_TOOL,
    RELEASE_HAND_TOOL,
    HOLD_HAND_TOOL,
    STOP_MOVE_TOOL,
    # HAND_FOLLOW_TOOL,
    PLACE_TOOL,
    BOXING_TOOL,
    HAND_SHAKE_TOOL
    ]
class FUNCTIONS_STATE(Enum):
    NOMAL = 0 
    GREETING = 1
    ARM_UP=2
    ARM_FORWARD=3
    ARM_DOWN = 4
    GRAP =5
    MOVE_LOCATION = 6
    MOVE_AND_PICK = 7
    HAND_FOLLOW= 8
    ATTENTION=9
    STANDBY=10
    HEART=11
    THINKINGSTATE=12
    RELEASE_HAND=13
    HOLD_HAND=14
    STOP_MOVE=15
    PLACE=16
    BOXING=17
    HAND_SHAKE=18
