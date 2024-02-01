class IgusDriverEncoder():
    def cartesian_move(self, position):
        x = position[0]
        y = position[1]
        z = position[2]
        # constranit the work range
        if z < 50:
            z = 50
        if z > 300:
            z = 300
        message = f"CRISTART 1234 CMD Move Cart {x} {y} {z} 0 0 0 0 0 0 100 CRIEND"
        encoded = message.encode('utf-8')
        move_array = bytearray(encoded)
        return message

    def gripper(self, D22=1, D23=1):
        if D22 == 0:
            message = "CRISTART 1234 CMD DOUT 22 false CRIEND"
        else:
            message = "CRISTART 1234 CMD DOUT 22 true CRIEND"
        encoded = message.encode('utf-8')
        Dout_array = bytearray(encoded)

        if D23 == 0:
            message = "CRISTART 1234 CMD DOUT 23 false CRIEND"
        else:
            message = "CRISTART 1234 CMD DOUT 23 true CRIEND"
        encoded = message.encode('utf-8')
        Dout_array = bytearray(encoded)
        return message 
