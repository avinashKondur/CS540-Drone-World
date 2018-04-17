import random

def takeInputs():
    fileName = input("Enter the name of file ** without .txt") + ".txt"
    noOfRedB, noOfBlueB, noOfGreenB, noOfYellowB = map(int, input(
        "Enter the no. of red, blue, green and yellow box **USE , TO SEPARATE**").split(','))

    z_max = int(input("Enter the maximum height(y-coordinate):  -1 for default values"))
    if z_max == -1:
        return createWorld(noOfRedB, noOfBlueB, noOfGreenB, noOfYellowB, fileName)
    else:
        return createWorld(noOfRedB, noOfBlueB, noOfGreenB, noOfYellowB, fileName, z_max)


def createWorld(noOfRedB, noOfBlueB, noOfGreenB, noOfYellowB, fileName, z_max=50):
    totalNoBlocks = noOfRedB + noOfBlueB + noOfGreenB + noOfYellowB
    file = open(fileName, "a")
    ans = input("want to specify drone coordinates?(y/n)")
    if ans == 'y':
        DronePos = input("Enter the position of drone(x,y,z)")
        file.write("(" + DronePos + "," + "drone)\n")
    coordinates = []
    noOfB, assignedR, assignedB, assignedG, assignedY = 0, 0, 0, 0, 0

    while noOfB <= totalNoBlocks:
        x = random.randint(-50, 50)
        y = random.randint(0, z_max)
        z = random.randint(-50, 50)
        coord = x, y, z
        if coord in coordinates:
            continue
        else:
            coordinates.append(coord)
            noOfB += 1

    while (assignedR < noOfRedB):
        nX, nY, nZ = coordinates.pop(0)
        file.write("(" + str(nX) + "," + str(nY) + "," + str(nZ) + "," + "red)\n")
        assignedR += 1

    while (assignedB < noOfBlueB):
        nX, nY, nZ = coordinates.pop(0)
        file.write("(" + str(nX) + "," + str(nY) + "," + str(nZ) + "," + "blue)\n")
        assignedB += 1

    while (assignedG < noOfGreenB):
        nX, nY, nZ = coordinates.pop(0)
        file.write("(" + str(nX) + "," + str(nY) + "," + str(nZ) + "," + "green)\n")
        assignedG += 1

    print(len(coordinates))
    while (assignedY <= noOfYellowB):
        nX, nY, nZ = coordinates.pop(0)
        file.write("(" + str(nX) + "," + str(nY) + "," + str(nZ) + "," + "yellow)\n")
        assignedY += 1

if __name__ == '__main__':
    takeInputs()