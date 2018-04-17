# -*- coding: utf-8 -*-
"""
Created on Sun Feb 11 11:20:14 2018

@author: Avi
"""
import copy
from math import sqrt

class Node:

    def __init__(self, state, f=0, g=0, h=0):
        self.state = state

        self.f = f
        self.g = g
        self.h = h

    def __repr__(self):
        return "Node(" + repr(self.state) + ", f=" + repr(self.f) + ", g=" + repr(self.g) + ", h=" + repr(self.h) + ")"

class ActionFunctions:
    
    def __init__(self, goalState, isDrone):
        self.goalState = goalState
        self.isSearchForDonePath = isDrone
        
    def goalTestF(self,parent):
        if ((parent[0]== self.goalState[0]) and (parent[2]==self.goalState[2]) and parent[1]>self.goalState[1]):
            return True
        elif parent==self.goalState:
            return True
        return False
    
    def actionF(self,state,world):
        actions=self.__allActions()
        dy = self.goalState[1] - state[1]
        dz = self.goalState[2] - state[2]
        dx = self.goalState[0] - state[0]
    
        modiAction = []
    
        for action in actions:
            newPos,_= self.takeActionF(state,action)
            
            if newPos[1]<0:
                continue
            
            _,validBlock=world.ValidatePos(newPos, False, True)
                        
            # if the search is for Drone to source block, then the below statement 
            validDrone = False
            if self.isSearchForDonePath == True:
                
                DronePos=[newPos[0],newPos[1]+1,newPos[2]]
                _,validDrone=world.ValidatePos(DronePos,True,True)
                
                #if validDrone == False:
                    #print(validDrone, DronePos)
                
            if (validBlock == True and validDrone == True) or (validBlock == True and self.isSearchForDonePath == False):
    
                if dx >= 0 and action[0] >= 0 and action not in modiAction:  # right
                    modiAction.append(action)
                elif dx < 0 and action[0] < 0 and action not in modiAction:  # left
                    modiAction.append(action)
                if dy >= 0 and action[1] >= 0 and action not in modiAction:  # up
                    modiAction.append(action)
                elif dy < 0 and action[1] < 0 and action not in modiAction:  # down
                    modiAction.append(action)
                if dz >= 0 and action[2] >= 0 and action not in modiAction:  # forward
                    modiAction.append(action)
                elif dz < 0 and action[1] < 0 and action not in modiAction:  # backward
                    modiAction.append(action)
                if action not in modiAction:
                    modiAction.append(action)
        
        return(modiAction)
    
    def __allActions(self):
        return ([[-1, 0, -1],
                 [-1, 0, 0],
                 [-1, 0, 1],
                 [0, 0, 1],
                 [0, 0, - 1],
                 [1, 0, 1],
                 [1, 0, 0],
                 [1, 0, - 1],
                 [-1, -1, -1],
                 [-1, -1, 0],
                 [-1, -1, 1],
                 [-1, 1, 0],
                 [-1, 1, -1],
                 [-1, 1, 1],
                 [0, 1, 1],
                 [0, 1, 0],
                 [0, 1, -1],
                 [0, -1, 0],
                 [0, -1, 1],
                 [0, -1, -1],
                 [1, 1, 1],
                 [1, 1, 0],
                 [1, 1, - 1],
                 [1, - 1, 0],
                 [1, - 1, 1],
                 [1, - 1, - 1]])
    
    
    def takeActionF(self,parentState, action):
        #state=copy.copy(parentState)
        state=[sum(n) for n in zip(*[parentState,action])]
        return(state,1)
    
class HeuristicFunctions:
    
    def __init__(self):
        pass
    
    #Euclidean distance
    def hf(self,state, goalState):
        h=EuclideanDistance(state,goalState)
        return h
    
    #Manhatten distance
    def hf2(self,state, goalState):
        h=abs(state[0]-goalState [0])+abs(state[1]-goalState [1])+abs(state[2]-goalState [2])
        return h
    
    #force - should be minimised  ----- Not a good hueristic function
    def hf3(self,state, goalState):
        h=goalState [1]-state[1]
        return h
    
    def hf_1(self,state,goalState):
        x1,x2 = state[0],goalState [0]
        y1,y2 = state[1],goalState [1]
        z1,z2 = state[2],goalState [2]
        h = abs(x1-x2)+abs(y1-y2)+abs(z1-z2)
        return h
    
def EuclideanDistance(s, t):
    return sqrt((s[0]-t[0])**2+(s[1]-t [1])**2+(s[2]-t [2])**2)