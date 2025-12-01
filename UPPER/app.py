# -*- coding: utf8 -*-
import sys
import traceback
import multiprocessing 
import time
# Original imports remain the same
from ultralytics import YOLO, solutions
import playsound
import asyncio
import pyaudio
import os, cv2
from threading import Thread
from PyQt5.QtWidgets import *
from PyQt5.QtCore import *
from PyQt5.QtGui import *
from GUI.pages import *
from work_process import *
from lib import *

class LLM_MULTIMODAL_LIVE():
    def __init__(self, InfoQueue, FunctionQueue, emotionQueue, ActivateQueue):
        self.gemini = gemini_function_tools("gemini-live-2.5-flash-preview")
        # self.gemini = gemini_function_tools("gemini-2.5-flash-native-audio-preview-09-2025")
        self.InfoQueue = InfoQueue
        self.FunctionQueue = FunctionQueue
        self.emotionQueue = emotionQueue
        self.ActivateQueue = ActivateQueue
        self.function_responses = None
        self.Activate_State = True
        self.stt_text = ""
        self.idle_time = time.time()
        self.SILENCE_THRESHOLD = 500
        self.SILENCE_DURATION_S = 0.1
        self.current_imotion = ""
        self.set_idle_time = 5
        self.user_input_flag = 0

    def start(self):
        asyncio.run(self.audio_loop())

    async def audio_loop(self):
        # ✨ 재연결을 위한 외부 루프
        while True:
            # ✨ PyAudio 객체를 루프 시작 시점에 새로 생성하여 오디오 드라이버 안정성 확보
            pya = pyaudio.PyAudio()
            mic_info = pya.get_default_input_device_info()

            print(mic_info)
            input_stream = None
            output_stream = None

            # ✨ GoAway 신호를 감지하고 부드러운 재연결을 시작하기 위한 Event 객체
            shutdown_event = asyncio.Event()

            try:
                config = self.gemini.get_connect_config()
                if self.gemini.previous_session_handle:
                    print(f"\n세션 핸들로 재연결을 시도합니다: {self.gemini.previous_session_handle[:10]}...")

                async with self.gemini.client.aio.live.connect(
                    model=self.gemini.model,
                    config=config
                ) as session:
                    print("✅ 세션 연결 성공!")

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

                    # --- 내부 함수 listen_and_send ---
                    async def listen_and_send():
                        silent_chunks_count = 0
                        chunks_per_second = self.gemini.SEND_SAMPLE_RATE / self.gemini.CHUNK_SIZE
                        silence_chunks_needed = (self.SILENCE_DURATION_S * chunks_per_second)
                        last_send_time = time.time()
                        keep_warm_interval = 10
                        silent_chunk = b'\x00' * self.gemini.CHUNK_SIZE

                        while True:
                            try:
                                if self.InfoQueue.qsize() > 0:
                                    self.gemini.detected_object_dict = self.InfoQueue.get()

                                if self.ActivateQueue.qsize() > 0:
                                    self.Activate_State = self.ActivateQueue.get()

                                if self.Activate_State:
                                    data = await asyncio.to_thread(input_stream.read, self.gemini.CHUNK_SIZE, exception_on_overflow=False)
                                    audio_data = np.frombuffer(data, dtype=np.int16)
                                    rms = np.sqrt(np.mean(audio_data.astype(np.float32)**2))
                                    if rms < self.SILENCE_THRESHOLD:
                                        silent_chunks_count = 0
                                    else:
                                        silent_chunks_count += 1

                                    if silent_chunks_count > silence_chunks_needed:
                                        self.idle_time = time.time()
                                        self.current_imotion = "THINKING"
                                        self.emotionQueue.put("THINKING")
                                        self.user_input_flag = True
                                    else:
                                        if self.current_imotion != "IDLE":
                                            if time.time() - self.idle_time >= self.set_idle_time:
                                                self.current_imotion = "IDLE"
                                                self.emotionQueue.put("IDLE")

                                    await session.send_realtime_input(
                                        audio=types.Blob(data=data, mime_type="audio/pcm")
                                    )
                                    
                                    last_send_time = time.time()

                                if time.time() - last_send_time > keep_warm_interval:
                                    print(f"[System] {keep_warm_interval}초 이상 활동 없어 연결 유지 신호 전송...")
                                    await session.send_realtime_input(
                                        audio=types.Blob(data=silent_chunk, mime_type="audio/pcm")
                                    )
                                    last_send_time = time.time()

                            except OSError as e:
                                print(f"Audio input error: {e}")
                                await asyncio.sleep(0.1)
                                continue

                    # --- 내부 함수 receive_and_play ---
                    async def receive_and_play(shutdown_event: asyncio.Event):
                        model_speaking = False
                        while True:
                            async for response in session.receive():
                                if response.go_away is not None:
                                    print(f"🚨 서버로부터 연결 종료 예고 수신. 남은 시간: {response.go_away.time_left.total_seconds()}초")
                                    print("부드러운 재연결을 준비합니다...")
                                    shutdown_event.set() # 재연결 신호 발생

                                if response.session_resumption_update:
                                    update = response.session_resumption_update
                                    if update.resumable and update.new_handle:
                                        print(f"🔄 새 세션 핸들 수신 및 저장: {update.new_handle[:10]}...")
                                        self.gemini.previous_session_handle = update.new_handle

                                if response.tool_call:
                                    function_calls = response.tool_call.function_calls
                                    for function_call in function_calls:
                                        self.function_responses = self.gemini.check_function(function_call)
                                        robot_control_dict = {
                                            'target_object': self.gemini.target_object,
                                            'detected_object_dict': self.gemini.detected_object_dict,
                                            'function_state': self.gemini.function_state,
                                            'location': self.gemini.location
                                        }
                                        self.FunctionQueue.put(robot_control_dict)
                                        print("function send:", self.function_responses)
                                    await session.send_tool_response(function_responses=self.function_responses)
                                    continue

                                if response.server_content and response.server_content.model_turn:
                                    model_speaking = True
                                    self.emotionQueue.put("SMILE")
                                    for part in response.server_content.model_turn.parts:
                                        if part.inline_data:
                                            await asyncio.to_thread(output_stream.write, part.inline_data.data)

                                if response.server_content and response.server_content.turn_complete:
                                    model_speaking = False
                                    self.emotionQueue.put("IDLE")

                    listen_task = asyncio.create_task(listen_and_send())
                    receive_task = asyncio.create_task(receive_and_play(shutdown_event))

                    # ✨ 작업이 모두 끝나거나, 재연결 신호(shutdown_event)가 발생할 때까지 기다림
                    done, pending = await asyncio.wait(
                        [listen_task, receive_task, shutdown_event.wait()],
                        return_when=asyncio.FIRST_COMPLETED
                    )

                    # ✨ 재연결 신호로 인해 종료된 경우, 나머지 진행중인 작업들을 정리
                    if shutdown_event.is_set():
                        print("재연결을 위해 현재 태스크를 정리합니다...")
                        for task in pending:
                            task.cancel()
                        # async with 블록을 정상적으로 빠져나가 finally를 호출하고 재연결 시작
                        break 

            except Exception as e:
                print(f"🚨 세션 오류 발생: {type(e).__name__}. 5초 후 재연결합니다.")
                traceback.print_exception(None, e, e.__traceback__)
                await asyncio.sleep(5)

            finally:
                # ✨ 연결이 어떤 방식으로든 종료될 때 모든 오디오 리소스를 확실하게 정리
                print("오디오 스트림 및 PyAudio 인스턴스를 정리합니다...")
                if input_stream and input_stream.is_active():
                    input_stream.stop_stream()
                    input_stream.close()
                if output_stream and output_stream.is_active():
                    output_stream.stop_stream()
                    output_stream.close()
                pya.terminate()

# ==================== Step 1: Create a Worker Class ====================
# This class will handle all background processing in the separate thread.
class Worker(QObject):
    """
    Runs long-running tasks in a separate thread to prevent the GUI from freezing.
    It emits a signal with the processed results.
    """
    # Signal that will send a dictionary of results back to the main thread
    update_signal = pyqtSignal(dict)

    def __init__(self, yolo_model, EmotionQueue):
        super().__init__()
        self.yolo_model = yolo_model
        self.EmotionQueue = EmotionQueue
        self._is_running = False

    @pyqtSlot()
    def run(self):
        """Main processing loop for the worker thread."""
        self._is_running = True
        while self._is_running:
            try:
                # --- Heavy, blocking task (YOLO detection) ---
                ODimg, ODinfo = self.yolo_model.Detect()

                # --- Fast queue checks ---

                
                emotion_state = None
                if self.EmotionQueue.qsize() > 0:
                    emotion_state = self.EmotionQueue.get()

                # Bundle all results into a dictionary
                results = {
                    'ODimg': ODimg,
                    'ODinfo': ODinfo,
                    'emotion_state': emotion_state
                }
                # Emit the signal to send the results to the main thread
                self.update_signal.emit(results)
                time.sleep(0.0001)

            except Exception as e:
                print(f"Error in worker thread: {e}")
                traceback.print_exc()

    def stop(self):
        """Signals the processing loop to stop."""
        self._is_running = False


class MainProgram(QWidget):
    def __init__(self):
        super().__init__()        
        self.EmotionQueue = multiprocessing.Queue()
        self.LLMMultiModalInfoQueue=multiprocessing.Queue()
        self.LLMMultiModalFunctionQueue=multiprocessing.Queue()
        self.LLMActivateQueue=multiprocessing.Queue()
        self.GripQueue = multiprocessing.Queue()
        self.MoveStateQ = multiprocessing.Queue()
        self.TargetGoalQ = multiprocessing.Queue()
        self.Moveit_State_Process_ResultQ =multiprocessing.Queue()
        self.Motor_Control_Process_CommandQ = multiprocessing.Queue()
        self.object_detection_resultQ=multiprocessing.Queue()
        self.scenario_Q=multiprocessing.Queue()

        self.LLMMultiModal = LLM_MULTIMODAL_LIVE(self.LLMMultiModalInfoQueue, self.LLMMultiModalFunctionQueue, self.EmotionQueue, self.LLMActivateQueue)
        self.LLMMultiModalThread = Thread(target=self.LLMMultiModal.start, args=())
        self.LLMMultiModalThread.daemon = True
        self.LLMMultiModalThread.start()

        self.Motor_Control_Process = multiprocessing.Process(target=SmartMotorController, args = (self.Moveit_State_Process_ResultQ,self.GripQueue))
        self.Motor_Control_Process.daemon = True
        self.Motor_Control_Process.start()

        self.RobotControlProcess = multiprocessing.Process(target=RobotControl, args = (self.LLMMultiModalFunctionQueue, self.GripQueue, self.TargetGoalQ, self.MoveStateQ,self.scenario_Q))
        self.RobotControlProcess.daemon = True
        self.RobotControlProcess.start()

        self.Moveit_State_Process = multiprocessing.Process(target=Moveit_Joint_Class, args = (self.Moveit_State_Process_ResultQ,))
        self.Moveit_State_Process.daemon = True
        self.Moveit_State_Process.start()

        # self.OrderToLowerProcess = multiprocessing.Process(target=TCPSocektServer, args = (self.TargetGoalQ, self.MoveStateQ))
        # self.OrderToLowerProcess.daemon = True
        # self.OrderToLowerProcess.start()

        self.prev_moveit_state=None
        self.moveit_state =None
        self.object_pos_dict={}
        self.objectIMG=None
        self.prevTime= time.time()
        self.stack = QStackedWidget(self)
        self.stack.setGeometry(0,0,1080,1080)
        self.pagelist = ApplicationPages(self.stack)
        self.currentUiState="SLEEP"
        self.prevUiState="SLEEP"
        self.page_state = False
        self.touch_cnt = 0
        self.sleep_time = 60
        self.yolo_model = YOLOframework()

        self.initUI()
        
        # ================= Step 2: Replace QTimer with QThread =================
        # self.timer = QTimer(self)                   # REMOVED
        # self.timer.setInterval(100)                 # REMOVED
        # self.timer.timeout.connect(self.timeout)    # REMOVED
        # self.timer.start()                          # REMOVED
        self.setup_thread()

    def setup_thread(self):
        """Initializes and starts the background worker thread."""
        self.thread = QThread()
        self.worker = Worker(
            self.yolo_model, 
            self.EmotionQueue
        )
        self.worker.moveToThread(self.thread)

        # Connect signals and slots for thread communication
        self.thread.started.connect(self.worker.run)
        self.worker.update_signal.connect(self.update_ui)
        
        self.thread.start()
        print("✅ Background worker thread started.")

    def initUI(self):
        self.setWindowTitle('My First Application')
        self.move(1080, 0)
        self.resize(1080, 1080)
        self.stack.setCurrentWidget(self.pagelist.mainpage.MainPage)
        self.setWindowFlags(Qt.FramelessWindowHint)   
        self.showFullScreen()
        # self.show()

    def closeEvent(self, event):
        # =========== Step 3: Update closeEvent for clean thread shutdown ===========
        print("Application closing...")

        # Stop the worker thread first
        if hasattr(self, 'thread') and self.thread.isRunning():
            print("Stopping worker thread...")
            self.worker.stop()  # Signal the worker's loop to finish
            self.thread.quit()    # End the thread's event loop
            self.thread.wait(5000) # Wait up to 5s for the thread to finish gracefully
            if self.thread.isRunning():
                print("⚠️ Thread did not stop gracefully, terminating.")
                self.thread.terminate()
            else:
                print("✅ Worker thread stopped.")
        
        print("Terminating child processes...")
        if hasattr(self, 'RobotControlProcess') and self.RobotControlProcess.is_alive():
            self.RobotControlProcess.terminate()

        if hasattr(self, 'Moveit_State_Process') and self.Moveit_State_Process.is_alive():
            self.Moveit_State_Process.terminate()

        if hasattr(self, 'Motor_Control_Process') and self.Motor_Control_Process.is_alive():
            self.Motor_Control_Process.terminate() # Corrected a bug from original code

        if hasattr(self, 'OrderToLowerProcess') and self.OrderToLowerProcess.is_alive():
            self.OrderToLowerProcess.terminate() # Corrected a bug from original code

        print("All termination signals sent. Closing window.")
        event.accept() 
        # rclpy.shutdown() # This should ideally be handled within its own process

    def mousePressEvent(self, event):
        # This method remains unchanged
        if event.buttons() & Qt.LeftButton:
            self.prevTime =time.time()
            self.page_state= not self.page_state      
            self.touch_cnt+=1
            if self.touch_cnt == 1:
                self.currentUiState = "IDLE"
            elif self.touch_cnt == 2:           
                self.currentUiState = "SMILING" 
            elif self.touch_cnt == 3:
                self.currentUiState = "FUNNY"
            elif self.touch_cnt == 4:
                self.currentUiState = "ANGRY"
            elif self.touch_cnt == 5:
                self.currentUiState = "THINKING"
            elif self.touch_cnt == 6:
                re = QMessageBox.question(self, "종료 확인", "종료 하시겠습니까?",
                        QMessageBox.StandardButton.Yes | QMessageBox.StandardButton.No)

                if re == QMessageBox.StandardButton.Yes:
                    self.close()
                self.touch_cnt=0

            if self.page_state :
                self.pagelist.mainpage.stopEMOJI()
                self.stack.setCurrentWidget(self.pagelist.camerapage.CameraPage)
            else:
                self.stack.setCurrentWidget(self.pagelist.mainpage.MainPage)

    # The 'timeout' method is no longer needed and has been removed.

    # =========== Step 4: Create a new slot to handle UI updates ===========
    @pyqtSlot(dict)
    def update_ui(self, results):
        """
        This slot receives data from the worker thread and updates the GUI.
        It runs on the main GUI thread, so it is safe to modify widgets here.
        """
        try:
            # Unpack the data bundle from the worker
            ODimg = results['ODimg']
            ODinfo = results['ODinfo']
            new_emotion_state = results['emotion_state']
            if self.Moveit_State_Process_ResultQ.qsize() > 0:
                self.moveit_state = self.Moveit_State_Process_ResultQ.get()


            # if self.moveit_state !=self.prev_moveit_state :
            #     self.prev_moveit_state=self.moveit_state
            #     print(self.prev_moveit_state)

            # if self.moveit_state is not None:
            #     self.Motor_Control_Process_CommandQ.put(self.moveit_state)

            self.currentWidget = self.stack.currentWidget()
            self.LLMMultiModalInfoQueue.put(ODinfo) # Can be done in worker or here
            self.scenario_Q.put(ODinfo)

            if self.currentUiState != "SLEEP":
                if time.time() - self.prevTime >= self.sleep_time:
                    self.currentUiState = "SLEEP"
                    print("잠들기")

            if self.currentWidget == self.pagelist.mainpage.MainPage:
                if new_emotion_state is not None:
                    self.currentUiState = new_emotion_state
                    self.prevTime = time.time()

                if self.currentUiState != self.prevUiState:
                    if self.currentUiState == "SLEEP":
                        self.pagelist.mainpage.setSleepEMOJI()
                    elif self.currentUiState == "IDLE":
                        self.pagelist.mainpage.setIdleEMOJI()
                    elif self.currentUiState == "SMILE":
                        self.pagelist.mainpage.setSmileEMOJI()
                    elif self.currentUiState == "ANGRY":
                        self.pagelist.mainpage.setAngryEMOJI()
                    elif self.currentUiState == "THINKING":
                        self.pagelist.mainpage.setThinkingEMOJI()
                    elif self.currentUiState == "SMILING":
                        self.pagelist.mainpage.setSmilingEMOJI()
                    elif self.currentUiState == "FUNNY":
                        self.pagelist.mainpage.setFunnyEMOJI()
                    elif self.currentUiState == "VERY_ANGRY":
                        self.pagelist.mainpage.setVeryAngryEMOJI()
                    elif self.currentUiState == "SEARCH":
                        self.pagelist.mainpage.setSearchEMOJI()
                    self.prevUiState = self.currentUiState

            elif self.currentWidget == self.pagelist.camerapage.CameraPage:
                if self.pagelist.camerapage.CameraPage.isVisible():
                    if ODimg is not None:
                        self.pagelist.camerapage.setImgLabel(ODimg)

        except Exception as e:
            print(f"Error updating UI: {e}")
            traceback.print_exc()

            
if __name__ == '__main__':
   app = QApplication(sys.argv)
   ex = MainProgram()
   sys.exit(app.exec_())
