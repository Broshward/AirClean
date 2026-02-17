#!/usr/bin/python
import socket
import time
from datetime import datetime

def start_server():
    # Храним время последнего полученного пакета
    last_received_time = 0

    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        s.bind(('0.0.0.0', 8283))
        s.listen(1)
        print(f"[{datetime.now().strftime('%H:%M:%S')}] Сервер запущен. Ожидание интервала > 60 сек.")

        while True:
            conn, addr = s.accept()
            with conn:
                data = conn.recv(4096).decode('utf-8', errors='ignore')
                if data:
                    current_time = time.time()
                    time_stamp = datetime.now().strftime('%H:%M:%S')
                    
                    # Проверяем разницу во времени
                    interval = current_time - last_received_time
                    
                    if last_received_time != 0 and interval < 60:
                        # Если прошло меньше 60 секунд
                        error_msg = f"INTERVAL {int(interval)}s < 1m"
                        print(f"[{time_stamp}] Ошибка от {addr}: {error_msg}")
                        conn.sendall(error_msg.encode())
                    else:
                        # Если всё в порядке
                        print(f"[{time_stamp}] Принято от {addr}: {data.strip()}")
                        conn.sendall(b"OK")
                        last_received_time = current_time

if __name__ == "__main__":
    start_server()
