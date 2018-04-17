# -*- coding: utf-8 -*-
"""
Created on Sun Feb  4 11:08:37 2018

@author: Avi
"""
import numpy as np
import random
#from math import sqrt
from Helpers import  EuclideanDistance


class DroneSimulator:
    
    def __init__(self,nx,ny,nz):
        
        self._nx = nx
        self._ny = ny
        self._nz = nz
        
        self.CurrentDronePos = None
        self.colors = dict()
        self.OccupiedPos = []
        self.IsBlockAttached = False
        
        self.Grid = [[['' for k in range(self._nx+1)] for j in range(self._ny+1)] for i in range(self._nz+1)]
    
    def Initialise(self, fileName):

        # try:
        fo = open(fileName, 'r')
        data = fo.read()
        lines = data.split('\n')

        print(lines)
        # fetch the drones information from the list
        drones = list(filter(lambda x: 'DRONE' in str.upper(str.strip(x)), lines))

        # fetch blocks information from the list
        blocks = list(filter(lambda x: 'DRONE' not in str.upper(str.strip(x)), lines))

        #print(blocks)
        # check if there is only one drone mentioned in the file
        if len(drones) != 1:
            print('Drone world should have one Drone. Please correct the input file and restart the process')
            return False

            # self.CurrentDronePos,obj,isValid = self.__getDataFromLine(drones[0])

        pos, color = self.ExtractPosColorFromInput(drones[0])
        self.CurrentDronePos = self.GetPosFromString(pos)
        self.CurrentDronePos, isValid = self.ValidatePos(self.CurrentDronePos, True)

        # `print("Drone Position : ", self.CurrentDronePos)
        if isValid == True:
            # print(obj)
            self.Grid[self.CurrentDronePos[0]][self.CurrentDronePos[1]][self.CurrentDronePos[2]] = color

            self.OccupiedPos.append(self.CurrentDronePos)

            print("Drone added to the world = " + str(self.OccupiedPos) + str(self.CurrentDronePos), str(isValid))

        if isValid == False:
            return False

        blocksDict = dict()

        for block in blocks:

            if block.strip() == '':
                continue

            # OccupiedPos,color,isValid = self.__getDataFromLine(block)
            pos, color = self.ExtractPosColorFromInput(block)
            pos = self.GetPosFromString(pos)

            blocksDict[tuple(pos)] = str.upper(color)

        blocksList = list(blocksDict.keys())
        blocksList = sorted(blocksList, key=lambda p: p[1])

        # print(blocksList)
        for block in blocksList:

            isValid = self.ValidatePos(list(block))

            if isValid == False:
                continue

            # print(block)
            # update the block pos dictionary
            self.OccupiedPos.append(list(block))

            # set the grid with respective Color
            self.Grid[block[0]][block[1]][block[2]] = blocksDict[block]

            # update the colors dictionary
            self.colors[str.upper(blocksDict[block])] = self.colors.get(str.upper(blocksDict[block]), 0) + 1

            # except IOError:
            # print('Exception while reading the file - ', fileName)
            # except:
            # print('Exception raised in initialising block world.....')


    def ReadGoalFile(self, fileName):
        fo = open(fileName, 'r')
        data = fo.read()
        lines = data.split('\n')
        drones = list(filter(lambda x: 'DRONE' in str.upper(str.strip(x)), lines))
        blocks = list(filter(lambda x: 'DRONE' not in str.upper(str.strip(x)), lines))

        '''if len(drones) != 1:
            print('Drone world should have one Drone. Please correct the Goal file and restart the process')
            return False'''
        
        goalStatesList = []
        # pos, color = self.ExtractPosColorFromInput(drones[0])
        for block in blocks:

            if block.strip() == '':
                continue
            # OccupiedPos,color,isValid = self.__getDataFromLine(block)
            pos, color = self.ExtractPosColorFromInput(block)
            pos = self.GetPosFromString(pos)
            if color != '?':
                color = str.upper(color)
            goalStatesList.append([pos,color])
        
        if len(drones) > 0:
            pos, color = self.ExtractPosColorFromInput(drones[0])
            pos = self.GetPosFromString(pos)
    
            goalStatesList.append([pos,color])

        return goalStatesList

    def Attach(self):
        dronePos = self.CurrentDronePos
        if (dronePos[0], dronePos[1] - 1, dronePos[2]) in self.OccupiedPos:
            self.IsBlockAttached = True
            return True
        else:
            print("Block cannot be attached as there is no block below.")
            return False

    def Move(self, dx, dy, dz):

        if dx not in [-1, 0, 1] or dy not in [-1, 0, 1] or dz not in [-1, 0, 1]:
            print("displacement length should not exceed more than 1 in length")
            return False

        newDronePos = [self.CurrentDronePos[0] + dx, self.CurrentDronePos[1] + dy, self.CurrentDronePos[2] + dz]

        if newDronePos in self.OccupiedPos:
            print("Destination square for drone is occupied")
            return False

        if self.IsBlockAttached:
            newOccupiedPos = [newDronePos[0], newDronePos[1] - 1, newDronePos[2]]

            if newOccupiedPos in self.OccupiedPos:
                print("Drone has block attached and the new position has a block obstructing the dron movement")
                return False

            currentOccupiedPos = [self.CurrentDronePos[0], self.CurrentDronePos[1] - 1, self.CurrentDronePos[2]]

            # move the block to new Position
            self.Grid[newOccupiedPos[0]][newOccupiedPos[1]][newOccupiedPos[2]] = \
            self.Grid[currentOccupiedPos[0]][currentOccupiedPos[1]][currentOccupiedPos[2]]
            self.OccupiedPos.append(newOccupiedPos)

            self.Grid[currentOccupiedPos[0]][currentOccupiedPos[1]][currentOccupiedPos[2]] = ''
            self.OccupiedPos.remove(currentOccupiedPos)

        # move the Drone to new Position
        self.Grid[newDronePos[0]][newDronePos[1]][newDronePos[2]] = \
        self.Grid[self.CurrentDronePos[0]][self.CurrentDronePos[1]][self.CurrentDronePos[2]]
        self.OccupiedPos.append(newDronePos)

        self.Grid[self.CurrentDronePos[0]][self.CurrentDronePos[1]][self.CurrentDronePos[2]] = ''
        self.OccupiedPos.remove(self.CurrentDronePos)

        self.CurrentDronePos = newDronePos

        return True

    def Release(self):
        if not self.IsBlockAttached:
            print("There is no block atttached and Release cannot be performed")
            return False

        currentOccupiedPos = (self.CurrentDronePos[0], self.CurrentDronePos[1] - 1, self.CurrentDronePos[2])
        while (currentOccupiedPos[0], currentOccupiedPos[1] - 1, currentOccupiedPos[2]) not in self.OccupiedPos:
            currentOccupiedPos = (currentOccupiedPos[0], currentOccupiedPos[1] - 1, currentOccupiedPos[2])

        self.IsBlockAttached = False
        return True

    def __getDataFromLine(self, line):

        pos, obj = self.ExtractPosColorFromInput(line)

        # validate the position
        transfromedPos, isValid = self.ValidatePos(self.GetPosFromString(pos), str.upper(obj) == 'DRONE')

        if isValid == False:
            print('Input has invalid Position : {0} for object : {1} '.format(pos, obj))
            if str.upper(obj) == 'DRONE':
                print('System will terminate now as the position for Drone is invalid')

        return (transfromedPos, str.upper(obj), isValid)

    def ExtractPosColorFromInput(self, line):

        x=line.split(" ")
        pos=x[0]+","+x[1]+","+x[2]
        obj=x[3]
        #pos = line[str.index(line, "(") + 1:str.rindex(line, ",")].strip()
        #obj = line[str.rindex(line, ",") + 1:str.rindex(line, ")")].strip()

        return pos, obj

    def GetPosFromString(self, pos):
        x = pos[:str.index(pos, ",")].strip()
        rest = pos[str.index(pos, ",") + 1:].strip()
        y = rest[:str.index(rest, ",")].strip()
        z = rest[str.index(rest, ",") + 1:].strip()
        return self.GetTransformedGridPosition([x, y, z])

    def GetTransformedGridPosition(self, pos):
        newX = int(pos[0]) + 50 if pos[0]!='?' else '?'
        newY=int(pos[1]) if pos[1]!='?' else '?'
        newZ = int(pos[2]) + 50 if pos[2]!='?' else '?'

        return [newX, newY, newZ]

    def GetTransformedUserFormat(self, pos):
        return [pos[0] - 50, pos[1], pos[2] - 50]

    def ValidatePos(self, pos, isDrone=False, pathSearch=False):

        (x, y, z) = (pos[0], pos[1], pos[2])

        # print(x,y,z)
        # check if coordinates are in valid range
        if x not in range(0, self._nx + 1) or y not in range(0, self._ny + 1) or z not in range(0, self._nz + 1):
            # print('Given coordinates {0} are out of range '.format(pos))
            return None, False

        # check if the block position is not in air
        if isDrone == False and pathSearch == False and y != 0 and [x, y - 1, z] not in self.OccupiedPos and [x, y - 1,
                                                                                                              z] != self.CurrentDronePos:
            # print('There is no supporting block below for pos = {0}'.format(pos))
            return None, False

        # if this is called in path search then, we would need to check position above the block is also un occcupied for the
        # Drone to move, after finding the path
        if isDrone == False and pathSearch == True and [x, y + 1, z] in self.OccupiedPos:
            return None, False

        # check if the position is already occupied by another block
        if pos in self.OccupiedPos:
            # print('There is already a block in the given position = {0}'.format(pos))
            return None, False

        return (pos, True)

    def GetLocationsOfMovableBlock(self, color, forDrone=False):

        '''print(self.colors)
        if color != '?' and str.upper(color) not in list(self.colors.keys()):
            print("There is no block with color : ", color)
            return None,False

        if color == '?':
            color = random.choice(list(self.colors.keys()))
            print("Color is not mentioned in the input, so randmonly choosen color is : ",color)'''

        nGrid = np.asarray(self.Grid)

        # print(np.where(nGrid == str.upper(color)), color)
        xi, yi, zi = np.where(nGrid == str.upper(color))

        if forDrone == True:
            indices = [[x, y + 1, z] for x, y, z in zip(*(xi, yi, zi))]
            # print('indices = ', indices)
            return list(filter(lambda p: nGrid[p[0]][p[1]][p[2]] == '', indices))
        else:
            indices = [[x, y, z] for x, y, z in zip(*(xi, yi, zi))]
            return list(filter(lambda p: nGrid[p[0]][p[1] + 1][p[2]] == '', indices))

    def GetDronePosition(self):
        # print(self.CurrentDronePos)
        return self.CurrentDronePos

    def GetRandomEmpty(self):
        nGrid = np.asarray(self.Grid)
        xi, yi, zi = np.where(nGrid == '')

        indices = [[x, y, z] for x, y, z in zip(*(xi, yi, zi))]
        return random.choice(list(filter(lambda p: nGrid[p[0]][p[1] - 1][p[2]] != '', indices)))

    def GetPossibleGoalPos(self, gPos):

        (xmissing, ymissing, zmissing) = (gPos[0] == '?', gPos[1] == '?', gPos[2] == '?')

        # print('gPos = ', gPos)

        nGrid = np.asarray(self.Grid)
        xi, yi, zi = np.where(nGrid == '')
        indices = [[x, y, z] for x, y, z in zip(*(xi, yi, zi))]

        # print('len(indices)=',len(indices))
        if ymissing == False:
            yLevelIndices = list(filter(lambda p: (p[1] == 0 and p[1] == int(gPos[1])) or (
                        nGrid[p[0]][int(gPos[1]) - 1][p[2]] != '' and p[1] == int(gPos[1])), indices))
        else:

            yLevelIndices = list(filter(lambda p: nGrid[p[0]][p[1] - 1][p[2]] != '', indices))
        # print('len(yLevelIndices)=',yLevelIndices)

        if xmissing == False:
            yLevelIndices = list(filter(lambda p: p[0] == int(gPos[0]) + 50 , yLevelIndices))
            # print('len(yLevelIndices)=',yLevelIndices)

        if zmissing == False:
            yLevelIndices = list(filter(lambda p: p[2] == int(gPos[2]) + 50 and self.ValidatePos(p,False), yLevelIndices))
            # print('len(yLevelIndices)=',yLevelIndices)

        # print('yLevelIndices = ',yLevelIndices)
        return yLevelIndices

    def IsPositionAvailable(self, pos):
        return pos not in self.OccupiedPos

    def GetColor(self, pos):
        x, y, z = pos[0], pos[1], pos[2]
        return self.Grid[x][y][z]

    def GetRandomAvailableColor(self):
        return random.choice(list(self.colors.keys()))

    def checkGoalAlreadyReached(self,color,goalState,finalGoalStates):
        xMissing, yMissing, zMissing = goalState[0] == '?', goalState[1] == '?', goalState[2] == '?'
        
        isColorGiven = True if color != '?' else False
        
        if xMissing:
                if yMissing and not zMissing:
                    OccupiedBlocks = list(filter(lambda x: x[2] == goalState[2], self.OccupiedPos))
                elif yMissing and zMissing:
                    OccupiedBlocks=self.OccupiedPos
                elif zMissing:
                    OccupiedBlocks = list(filter(lambda x: x[1] == goalState[1], self.OccupiedPos))
                else:
                    OccupiedBlocks = list(filter(lambda x: x[1] == goalState[1] and x[2] == goalState[2], self.OccupiedPos))

        else:
                if yMissing:
                    if zMissing:
                        OccupiedBlocks = list(filter(lambda x: x[0] == goalState[0], self.OccupiedPos))
                    else:
                        OccupiedBlocks = list(filter(lambda x: x[0] == goalState[0] and x[2] == goalState[2], self.OccupiedPos))
                else:
                    OccupiedBlocks = list(filter(lambda x: x[0] == goalState[0] and x[1] == goalState[1], self.OccupiedPos))

        if isColorGiven == True and OccupiedBlocks!=[]:
            OccupiedBlocks = list(filter(lambda x: self.GetColor(x) == color,OccupiedBlocks))

        #avail = [m if m not in [row[0] for row in finalGoalStates] else None for m in OccupiedBlocks]
        possiblePos = list(filter(lambda a: a not in finalGoalStates, OccupiedBlocks))
        return possiblePos

    def hasSupportingBlock(self,lists):
        return list(filter(lambda x: [x[0], x[1] - 1, x[2]] != world.CurrentDronePos,lists))
    
    def GetMaxHeight(self, goalState=None):
        
         '''
         Gets the max height of the plane x,z when goalState is not None.
         if goalState is None, returns the max height of the world
         '''
         nGrid = np.asarray(self.Grid)
         xi,yi,zi = np.where(nGrid != '')
         indices = [[x,y,z] for x,y,z in zip(*(xi,yi,zi))]
         
         if goalState != None:
             if nGrid[goalState[0]][0][goalState[2]] == '' :
                 maxHeight = 0
             else:
                 maxHeight = max(list(filter(lambda p : p[0] == goalState[0] and p[2] == goalState[2] , indices)))[1]
         else:
             maxHeight = max(indices)[1]
         return maxHeight
     
    def GetPositions(self,goalState, k, color, param):
        
        worldHeight = self.GetMaxHeight()
        planeHeight = self.GetMaxHeight(goalState)
         
        nGrid = np.asarray(self.Grid)
         
        if param[0] == 'Blocks':
            xi,yi,zi = np.where(nGrid != '')
            indices = [[x,y,z] for x,y,z in zip(*(xi,yi,zi))]
            indices = list(filter(lambda p : p[1] >= planeHeight and p[1] <= worldHeight 
                                  and nGrid[p[0]][p[1]+1][p[2]] == '' 
                                  and nGrid[p[0]][p[1]][p[2]] != color
                                  and p not in param[1], indices))
            if indices == []:
               indices = [[x,y,z] for x,y,z in zip(*(xi,yi,zi))]
               indices = list(filter(lambda p : nGrid[p[0]][p[1]+1][p[2]] == '' 
                                     and nGrid[p[0]][p[1]][p[2]] != color
                                     and p not in param[1], indices))
                              
        if param[0] == 'Empty':
            xi,yi,zi = np.where(nGrid == '')
            indices = [[x,y,z] for x,y,z in zip(*(xi,yi,zi))]
            indices = list(filter(lambda p : nGrid[p[0]][p[1]-1][p[2]] != '' or p[1] == 0, indices))
            indices = list(filter(lambda p : p not in param[1], indices))
        
        dists = [(index,EuclideanDistance(goalState, index)) for index in indices]
        dists = sorted(dists, key = lambda i : i[1])
        return [ block[0] for block in dists[:k]]
    
    def GetAvailableBlocks(self, color):
        
        nGrid = np.asarray(self.Grid)
        xi,yi,zi = np.where(nGrid == color)
        indices = [[x,y,z] for x,y,z in zip(*(xi,yi,zi))]
        
        return indices
        
    
if __name__ == '__main__':
    world = DroneSimulator(100,50,100)
    world.Initialise('grid3.txt')
    print(world.ReadGoalFile('test.txt')       )