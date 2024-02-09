import socket
import re
from numpy.linalg import norm
import numpy as np
import time
from threading import Thread, Event

sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)

# Enter the IP address of the robot (192.168.3.11) here if you're not using a CPRog/iRC simulation
# server_address = ('localhost', 3920)
server_address = ('192.168.3.11', 3920)
print("Connecting...")
sock.connect(server_address)
print("Connected")


class AliveDecode(Thread):
    def __init__(self, event):
        super(AliveDecode, self).__init__()

        self.event = event
        # The ALIVEJOG message needs to be sent regularly (at least once a second) to keep the connection alive
        messageAliveJog = "CRISTART 1234 ALIVEJOG 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 CRIEND"
        encodedAliveJog = messageAliveJog.encode('utf-8')
        self.arrayAliveJog = bytearray(encodedAliveJog)

        # cartesian position
        self.cartesian_position = None
        # digital input
        self.din = []

    def run(self):
        global sock
        i_f = 0
        while True:
            i_f = i_f + 1
            if i_f > 10:
                sock.sendall(self.arrayAliveJog)
                i_f = 0
            data = sock.recv(1024).decode()
            try:
                result1 = re.findall(r'DIN \d+', data, re.DOTALL)
                a = result1[0].split()
                din = int(a[1])
                self.din = [i + 1 for i,
                            j in enumerate(bin(din)[:1:-1]) if j == "1"]
            except:
                pass

            try:
                result2 = re.findall(
                    r'POSCARTROBOT(?: \-?\d+\.?\d+){3}', data, re.DOTALL)
                b = result2[0].split()
                self.cartesian_position = [float(ele) for ele in b[1:]]
            except:
                pass
            # shutdown the thread
            if self.event.is_set():
                break
            # TODO search the website to figure out the synchronization between client and server
            time.sleep(0.02)


def cartesian_move_array(position):
    x = position[0]
    y = position[1]
    z = position[2]
    message = f"CRISTART 1234 CMD Move Cart {x} {y} {z} 0 0 0 0 0 0 200 CRIEND"
    encoded = message.encode('utf-8')
    move_array = bytearray(encoded)
    return move_array


def gripper(sock, D22=1, D23=1):
    if D22 == 0:
        message = "CRISTART 1234 CMD DOUT 22 false CRIEND"
    else:
        message = "CRISTART 1234 CMD DOUT 22 true CRIEND"
    encoded = message.encode('utf-8')
    Dout_array = bytearray(encoded)
    sock.sendall(Dout_array)

    if D23 == 0:
        message = "CRISTART 1234 CMD DOUT 23 false CRIEND"
    else:
        message = "CRISTART 1234 CMD DOUT 23 true CRIEND"
    encoded = message.encode('utf-8')
    Dout_array = bytearray(encoded)
    sock.sendall(Dout_array)


def start_machine():
    messageConnect = "CRISTART 1234 CMD Connect CRIEND"
    messageReset = "CRISTART 1234 CMD Reset CRIEND"
    messageEnable = "CRISTART 1234 CMD Enable CRIEND"
    encodedConnect = messageConnect.encode('utf-8')
    encodedReset = messageReset.encode('utf-8')
    encodedEnable = messageEnable.encode('utf-8')
    array = bytearray(encodedConnect)
    sock.sendall(array)
    time.sleep(1)
    # array = bytearray(encodedReset)
    # sock.sendall(array)
    # time.sleep(1)
    array = bytearray(encodedEnable)
    sock.sendall(array)
    time.sleep(1)
    print("start the machine")


def reference_machine():
    messageReference = "CRISTART 1234 CMD ReferenceAllJoints CRIEND"
    encodedReference = messageReference.encode('utf-8')
    array = bytearray(encodedReference)
    sock.sendall(array)
    print("start reference the robot.")
    time.sleep(30)
    print("finish the robot referencing.")


def close_machine():
    messageDisconnect = "CRISTART 1234 CMD Disconnect CRIEND"
    messageDisable = "CRISTART 1234 CMD Disable CRIEND"

    encodedDisconnect = messageDisconnect.encode('utf-8')
    encodedDisable = messageDisable.encode('utf-8')
    array = bytearray(encodedDisconnect)
    sock.sendall(array)
    time.sleep(1)
    array = bytearray(encodedDisable)
    sock.sendall(array)
    time.sleep(1)
    print("close the machine")


def main():
    start_machine()
    stop_event = Event()
    alive_client = AliveDecode(stop_event)

    home = [0, 0, 300]
    position = []
    position1 = [0, 0, 300]
    position2 = [0, 0, 100]
    position3 = [0, 150, 150]
    position4 = [0, 0, 300]
    position.append(position1)
    position.append(position2)
    position.append(position3)
    position.append(position4)

    alive_client.start()
    gripper(sock, 1, 1)

    array = cartesian_move_array(home)
    sock.sendall(array)

    time.sleep(2)
    print("Start the project!")

    actual_pos = alive_client.cartesian_position
    while actual_pos is None:
        time.sleep(0.1)
        actual_pos = alive_client.cartesian_position
        print(f"actual_pos is {actual_pos}")

    t = time.time()
    for i in range(len(position)):

        array = cartesian_move_array(position[i])
        sock.sendall(array)

        time.sleep(0.2)
        actual_pos = alive_client.cartesian_position

        while norm(np.array(actual_pos) - np.array(position[i])) > 5:
            actual_pos = alive_client.cartesian_position
            print(f"actual position is {actual_pos}")
            time.sleep(0.01)

    print(f"elapsed time is {time.time()-t}.")
    print("Finally")

    stop_event.set()
    alive_client.join()
    close_machine()
    sock.close()
    time.sleep(0.5)
    exit()


if __name__ == "__main__":
    main()
