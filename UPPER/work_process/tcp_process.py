import socket
import time, json
class TCPSocektServer():
    def __init__(self, TargetGoalQ, MoveStateQ):
        # 서버 설정
        self.TargetGoalQ = TargetGoalQ
        self.MoveStateQ = MoveStateQ
        self.location = ""
        self.host = "172.20.10.2"  # 서버의 IP 주소 또는 도메인 이름
        self.port = 7777       # 포트 번호
        # 서버 소켓 생성
        self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.server_socket.bind((self.host, self.port))
        self.server_socket.listen(5)
        self.send_delay = 1
        self.prev_send_time = time.time()
        location_dict = {"location":self.location}
        self.send_message = json.dumps(location_dict)
        # 클라이언트 연결 대기
        while True:
            try:
                print(f"서버가 {self.host}:{self.port}에서 대기 중입니다...")
                client_socket, client_address = self.server_socket.accept()
                client_socket.setblocking(False)
                print(f"클라이언트 {client_address}가 연결되었습니다.")
            except Exception as e:
                pass

            while True:
                try:
                    if self.TargetGoalQ.qsize()>0:
                        self.location=self.TargetGoalQ.get()
                        location_dict = {"location":self.location}
                        self.send_message = json.dumps(location_dict)

                    if time.time() - self.prev_send_time >= self.send_delay:
                        self.prev_send_time=time.time()
                        client_socket.send(self.send_message.encode("utf-8"))

                except (BrokenPipeError, ConnectionResetError) as e:
                    print(f"Connection error: {e}")
                    client_socket.close()    
                    time.sleep(5)
                    break
                    # 클라이언트로부터 요청 받기
                try:
                    receivemessage = client_socket.recv(1024).decode(("utf-8"))
                    if len(receivemessage)>0:
                        try:
                            unpack_data = json.loads(receivemessage)
                            self.MoveStateQ.put(unpack_data)
                        except Exception as e:
                            print("JSON ERR",e)
                            pass
                except Exception as e:
                    pass
                    



# class TCPSocektClient():
#     def __init__(self):
#         #LLM 사용시 인터넷 연결 필요
#         self.server_address = "0.0.0.0"  # 서버의 실제 IP 주소 또는 도메인 이름
#         self.server_port = 7777# 서버 포트 번호
#         self.receive_message=None
#         # 서버에 연결
#         while True:
#             # 클라이언트 연결 대기  
#             try:
#                 client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
#                 client_socket.connect((self.server_address, self.server_port))
#                 client_socket.setblocking(False)
#                 print("server Connect")
#                 while True:
#                     try:
#                         self.receive_message = client_socket.recv(1024).decode("utf-8")
#                         if self.receive_message is not None:
#                             print("receive:",self.receive_message)
#                     except Exception as e:
#                         pass

#                     try:
#                         test_dict = self.receive_message
#                         self.receive_message=None
#                         client_socket.send(test_dict.encode("utf-8"))
#                     except Exception as e:
#                         pass

#             except Exception as e:
#                 print(e)
#                 pass

#             # finally:
#             #     # 클라이언트 소켓 닫기
#             #     print("연결종료")
#             #     client_socket.close()

if __name__ == "__main__":
    Server = TCPSocektServer()
    
