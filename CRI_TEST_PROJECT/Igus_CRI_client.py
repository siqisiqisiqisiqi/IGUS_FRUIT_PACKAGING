import socket
import re
from numpy.linalg import norm
import numpy as np
import time
from threading import Thread, Event

sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)

# Enter the IP address of the robot (192.168.3.11) here if you're not using a CPRog/iRC simulation
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
        while True:
            sock.sendall(self.arrayAliveJog)
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
            time.sleep(0.05)


def cartesian_move_array(position):
    x = position[0]
    y = position[1]
    z = position[2]
    message = f"CRISTART 1234 CMD Move Cart {x} {y} {z} 0 0 0 0 0 0 100 CRIEND"
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


def main():
    stop_event = Event()
    alive_client = AliveDecode(stop_event)

    position = []
    position1 = [0, 0, 300]
    position2 = [0, 0, 100]
    position3 = [100, 0, 100]
    home = [0, 0, 300]
    position.append(position1)
    position.append(position2)
    position.append(position3)

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

    j = 0
    t = time.time()
    for i in range(1, 6):

        array = cartesian_move_array(position[j])
        sock.sendall(array)

        time.sleep(0.2)
        actual_pos = alive_client.cartesian_position

        while norm(np.array(actual_pos) - np.array(position[j])) > 10:
            actual_pos = alive_client.cartesian_position
            print(f"actual position is {actual_pos}")
            time.sleep(0.01)

        j = j + 1
        if j >= len(position):
            j = 0

    print(f"elapsed time is {time.time()-t}.")
    print("Finally")

    stop_event.set()
    alive_client.join()
    sock.close()
    time.sleep(0.5)
    exit()


if __name__ == "__main__":
    main()
