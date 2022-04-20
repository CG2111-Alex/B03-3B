Main directory is `w8s2pi`. It is directory that is merged with week 9's file. 
Upload `w8s2pi/Alex/Alex.ino` to the Arduino.
Compile `w8s2pi/alex-pi.cpp` and run it in the raspi

# Setup 
Upload `Alex/Alex.ino` to the Arduino 
Compile `tls-alex-client.cpp` on the *HOST LAPTOP*
```g++ tls-alex-client.cpp make_tls_client.cpp tls_client_lib.cpp tls_pthread.cpp tls_common_lib.cpp -pthread -lssl -lcrypto -o tls-alex-client```
Compile `tls-alex-server.cpp` on the *Raspberry Pi*
```g++ tls-alex-server.cpp tls_server_lib.cpp tls_pthread.cpp make_tls_server.cpp tls_common_lib.cpp serial.cpp serialize.cpp -pthread -lssl -lcrypto -o tls-alex-server```
# Running


