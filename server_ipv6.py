#!/usr/bin/python
import socket
import time
from datetime import datetime

def start_server():
    last_received_time = 0
    port = 8283

    # Создаем сокет для IPv6
    s = socket.socket(socket.AF_INET6, socket.SOCK_STREAM)
    
    # Опция, позволяющая сокету принимать и IPv4, и IPv6
    try:
        s.setsockopt(socket.IPPROTO_IPV6, socket.IPV6_V6ONLY, 0)
    except (AttributeError, OSError):
        pass # На некоторых системах это значение по умолчанию
        
    s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    
    # Привязываемся к "::" (слушать везде)
    s.bind(('::', port))
    s.listen(5)
    
    print(f"[{datetime.now().strftime('%H:%M:%S')}] Сервер запущен (Dual Stack: IPv4/IPv6)")
    print(f"Слушаю порт {port}...")

    while True:
        conn, addr = s.accept()
        with conn:
            data = conn.recv(4096).decode('utf-8', errors='ignore')
            if data:
                current_time = time.time()
                time_stamp = datetime.now().strftime('%H:%M:%S')
                interval = current_time - last_received_time
                
                # Выводим данные в любом случае, как вы и просили
                print(f"\n[{time_stamp}] Данные от {addr}:")
                print(f"{data.strip()}")

                # Проверка интервала (60 секунд)
                if last_received_time != 0 and interval < 60:
                    error_msg = f"INTERVAL {int(interval)}s < 1m"
                    print(f"!!! Ошибка: {error_msg}")
                    conn.sendall(error_msg.encode())
                else:
                    print(">>> Статус: OK")
                    conn.sendall(b"OK")
                    last_received_time = current_time

if __name__ == "__main__":
    start_server()
