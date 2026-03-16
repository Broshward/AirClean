#!/usr/bin/python

import http.server
import ssl

context = ssl.SSLContext(ssl.PROTOCOL_TLS_SERVER)
context.load_cert_chain(certfile="cert.pem", keyfile="key.pem")
# Устанавливаем минимальную версию TLS 1.2 (лучший друг ESP32)
context.minimum_version = ssl.TLSVersion.TLSv1_2
context.set_ciphers('DEFAULT@SECLEVEL=1')

server_address = ('0.0.0.0', 8284)
httpd = http.server.HTTPServer(server_address, http.server.SimpleHTTPRequestHandler)
httpd.socket = context.wrap_socket(httpd.socket, server_side=True)

print("🚀 Безопасный сервер (ECDSA) запущен...")
httpd.serve_forever()
