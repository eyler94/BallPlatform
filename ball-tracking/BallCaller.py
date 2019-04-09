from BallPosition import BallPosition

BP = BallPosition()

pos = BP.Position("green")

print("pos:",pos)
print("x:",pos[0])

for i in range(0,1):
    pos = BP.Position("orange")
    print("pos:",pos)
    print("x:",pos[0])

BP.shutdown()
