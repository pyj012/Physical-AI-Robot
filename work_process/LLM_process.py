import sounddevice
import playsound
import asyncio
import pyaudio
import traceback
import os, time, cv2
from threading import Thread
import multiprocessing 
from lib.gemini_tools_lib import *
import numpy as np
class LLM_MULTIMODAL_LIVE():
    def __init__(self,InfoQueue,FunctionQueue, emotionQueue, ActivateQueue):
        load_time=time.time()
        print("Loading LLM PROCESS")
        self.gemini= gemini_function_tools("gemini-live-2.5-flash-preview")
        self.InfoQueue=InfoQueue
        self.FunctionQueue= FunctionQueue
        self.emotionQueue = emotionQueue
        self.ActivateQueue = ActivateQueue
        self.function_responses=None
        self.Activate_State= True
        self.emotion = True
        self.stt_text = ""
        self.idle_time = time.time()
        self.SILENCE_THRESHOLD = 1500
        self.SILENCE_DURATION_S = 0.2
        self.current_imotion = ""
        self.set_idle_time= 5
        print("LOADED LLM PROCESS : ", time.time()-load_time)

    def start(self):
        asyncio.run(self.audio_loop())

    async def audio_loop(self):
        audio_queue = asyncio.Queue()
        model_speaking = False
        session = None
        chunks_per_second = self.gemini.SEND_SAMPLE_RATE / self.gemini.CHUNK_SIZE
        # 필요한 침묵 청크 수
        silence_chunks_needed = (self.SILENCE_DURATION_S * chunks_per_second)
        pya = pyaudio.PyAudio()
        mic_info = pya.get_default_input_device_info()
        while True:
            try:
                async with (
                        self.gemini.client.aio.live.connect(
                            model=self.gemini.model, 
                            config=self.gemini.configs
                            ) as session,     
                    ):
                    input_stream = await asyncio.to_thread(
                        pya.open,
                        format=self.gemini.FORMAT,
                        channels=self.gemini.CHANNELS,
                        rate=self.gemini.SEND_SAMPLE_RATE,
                        input=True,
                        input_device_index=mic_info["index"],
                        frames_per_buffer=self.gemini.CHUNK_SIZE,
                    )
                    output_stream = await asyncio.to_thread(
                        pya.open, format=self.gemini.FORMAT, channels=self.gemini.CHANNELS, rate=self.gemini.RECEIVE_SAMPLE_RATE, output=True
                    )

                    async def listen_and_send():
                        count= 0
                        nonlocal model_speaking
                        silent_chunks_count = 0

                        while True:
                            if not model_speaking:
                                try:
                                    if self.InfoQueue.qsize()>0:
                                        self.gemini.detected_object_dict = self.InfoQueue.get()
                                    # if self.ActivateQueue.qsize()>0:
                                    #     self.Activate_State= self.ActivateQueue.get()
                                    #     if self.Activate_State:
                                    #         silent_chunks_count = 0

                                    if self.Activate_State:
                                        data = await asyncio.to_thread(input_stream.read, self.gemini.CHUNK_SIZE, exception_on_overflow=False)
                                        # 1. 오디오 데이터를 numpy 배열로 변환
                                        audio_data = np.frombuffer(data, dtype=np.int16)
                                        # 2. RMS(볼륨) 계산
                                        rms = np.sqrt(np.mean(audio_data.astype(np.float32)**2))
                                        # 3. 볼륨이 임계값보다 낮은지 확인
                                        if rms < self.SILENCE_THRESHOLD:
                                            silent_chunks_count =0

                                        else:
                                            print("사용자가 말하는중")
                                            # 소리가 감지되면 카운터 초기화
                                            silent_chunks_count += 1
                                        
                                        # 4. 침묵이 충분히 지속되었는지 확인
                                        # print(rms, silent_chunks_count, silence_chunks_needed)
                                        if silent_chunks_count > silence_chunks_needed:
                                            self.current_imotion="THINKING"
                                            self.emotionQueue.put("THINKING")
                                            self.idle_time=time.time()

                                        if self.current_imotion != "IDLE":
                                            if time.time() - self.idle_time >= self.set_idle_time:
                                                self.current_imotion= "IDLE"
                                                self.emotionQueue.put("IDLE")

                                        await session.send_realtime_input(
                                            audio=types.Blob(data=data, mime_type="audio/pcm")
                                        )

                                except Exception as e:
                                    print(f"Audio input error: {e}")
                                    await asyncio.sleep(0.1)
                                    return False
                            else:
                                await asyncio.sleep(0.1)

                    async def receive_and_play():
                        nonlocal model_speaking
                        while True:
                            async for response in session.receive():
                                if response.server_content:
                                    pass

                                elif response.tool_call:
                                    function_calls = response.tool_call.function_calls
                                    for function_call in function_calls:
                                        self.function_responses=self.gemini.check_function(function_call)
                                        robot_control_dict={
                                            'target_object':self.gemini.target_object,
                                            'detected_object_dict':self.gemini.detected_object_dict,
                                            'function_state' :self.gemini.function_state,
                                            'location' : self.gemini.location
                                        }
                                        self.FunctionQueue.put(robot_control_dict)
                                        print("function send:",  self.function_responses)
                                    await session.send_tool_response(function_responses=self.function_responses)
                                    continue
                                    # await session.send(input=self.function_responses,e end_of_turn=True)

                                if response.server_content and response.server_content.model_turn:
                                    model_speaking = True
                                    self.emotionQueue.put("SMILE")
                                    for part in response.server_content.model_turn.parts:
                                        if part.inline_data:
                                            await asyncio.to_thread(output_stream.write, part.inline_data.data)

                                elif response.server_content and response.server_content.turn_complete:
                                    model_speaking = False
                    await asyncio.gather(
                        asyncio.create_task(listen_and_send()),
                        asyncio.create_task(receive_and_play()),
                    )

            except Exception as e:
                traceback.print_exception(None, e, e.__traceback__)
                continue

            