import numpy as np


class IgusDriverEncoder():
    def cartesian_move(self, position):
        x = position[0]
        y = position[1]
        z = position[2]
        # constranit the work range
        if z < 50:
            z = 50
            print("z exceed the range!!!")
        if z > 300:
            z = 300
            print("z exceed the range!!!")
        if abs(x) > 200:
            x = np.sign(x) * 200
            print("x exceed the range!!!")
        if abs(y) > 200:
            y = np.sign(y) * 200
            print("y exceed the range!!!")
        message = f"CRISTART 1234 CMD Move Cart {x} {y} {z} 0 0 0 0 0 0 100 CRIEND"
        encoded = message.encode('utf-8')
        move_array = bytearray(encoded)
        return message

    def gripper(self, D21=0, D22=0):
        if D22 == 0:
            messageD22 = "CRISTART 1234 CMD DOUT 22 false CRIEND"
        else:
            messageD22 = "CRISTART 1234 CMD DOUT 22 true CRIEND"
        encoded = messageD22.encode('utf-8')
        Dout_arrayD22 = bytearray(encoded)

        if D21 == 0:
            messageD21 = "CRISTART 1234 CMD DOUT 21 false CRIEND"
        else:
            messageD21 = "CRISTART 1234 CMD DOUT 21 true CRIEND"
        encoded = messageD21.encode('utf-8')
        Dout_arrayD21 = bytearray(encoded)
        return messageD21
