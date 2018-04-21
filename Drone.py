# -*- coding: utf-8 -*-
"""
Created on Sun Feb 11 12:13:47 2018

@author: Avi
"""
from DroneWorld import DroneSimulator
from Astar import AStartSearch
from Helpers import  HeuristicFunctions,EuclideanDistance
from RelaxedAStar import RAStarSearch
from Plots import PlotPath
import random
import time
import numpy as np
import copy

class PathFinder:
    
    def __init__(self,world,searchAlgorithm):
        
        self.world = world
        self.SearchAlgo = searchAlgorithm
        self.__goalStates = [] 
        self.finalActions = dict()
        self.AvailbelColors = dict()
    
    
    def AchieveGoalStates(self, goalStates):
        
        self.__goalStates = [goal[0] for goal in goalStates]
        
        self.AvailbelColors = copy.deepcopy(self.world.colors)

        for goal in goalStates:
            
            start_time = time.time()
            
            goalState, color = goal[0], goal[1]
            
            print('*************************************************************')
            print('Idendifying actions for goal state = {}'.format(goal))
            print('*************************************************************')
            
            #identify goalstate if incomplete and also all actions required to perform
            goalState, actions,color = self.__identifyActions(self.world,goalState, color)
            
            print('obtained actions ....\n Starting performing actions .... ')
            
            self.finalActions[tuple(goalState)] = actions
            
            #iterate through all the actions and perform actions
            self.__performActions(actions)
            
            print('All actions are performed.......')

            print(' Currently Occupied Positions of World are ', self.world.OccupiedPos)
            
            #add the new goalState to the list if it empty, so that next steps in identify the goalState will eliminate this.
            if goal[0].count('?') > 0 :
                self.__goalStates.append(goalState)

            self.AvailbelColors[color] -= 1 
            
            self.__calculateTime(start_time)
        
    
    def __getRandomColor(self):
        
        availableColor = []
        for color in list(self.AvailbelColors.keys()):
            
            if self.AvailbelColors[color]  > 0:
                availableColor.append(color)
        return random.choice(availableColor)
        
    def __identifyActions(self,world,goalState,color):
        
        #identify if goal is complete or not
        isGoalComplete = False if goalState.count('?') > 0 else True
        
        #check if color is mentioned or not
        isColorMentioned = False if color == '?' else True
        
        #Update color if not mentioned.
        if isColorMentioned == False:
            if isGoalComplete == True and not world.IsPositionAvailable(goalState):
                color = world.GetColor(goalState)
            else:
                color = self.__getRandomColor()
                print('this is the color assigned : ', color)
                
        # identify goal state if it is incomplete        
        if isGoalComplete == False:
            print('Goal state is not complete... identifying the goal state')
            
            goalState,color = self.__identifyGoalState(world,goalState,color)
            
            print('Goalstate successfully identified, new goal state is {}, {}'.format(goalState,color))
        
        print('Identifying the actions on complete goal {}, {}'.format(goalState,color))
        
        #identify actions for the complete goal state
        actions = self.__getActionsForCompleteGoal(world,goalState,color)
        
        print('Actions are successfully identified inorder to achieve goalState {}'.format(actions))
        
        return goalState, actions,color
    
    def __identifyGoalState(self,world,goalState,color):
        #xMissing, yMissing, zMissing = goalState[0] == '?', goalState[1] == '?', goalState[2] == '?'
        '''isColorGiven = False if color == '?' else True
        
        possiblePos = world.checkGoalAlreadyReached(color, goalState,self.__goalStates)


        if possiblePos != []:
            newGoalState=random.choice(possiblePos)            
            if isColorGiven == False:
                color=world.GetColor(newGoalState)                
        else:
            saved=world.GetPossibleGoalPos(goalState)
            #print(saved)
            #saved=world.hasSupportingBlock(saved)
            #avail = [m if m not in self.__goalStates else None for m in saved]
            possiblePos = list(filter(lambda a: a not in self.__goalStates, saved))
            
            #print(possiblePos)
            newGoalState=self.__getNearestPositions(possiblePos,world.GetDronePosition())
            
        return newGoalState,color'''

        xMissing, yMissing, zMissing = goalState[0] == '?', goalState[1] == '?', goalState[2] == '?'
        x=0
        y=0
        z=0

        if xMissing == False:
            x = goalState[0]

        if yMissing == False:
            y = goalState[1]

        if zMissing == False:
            z = goalState[2]        

        i = 1
        while (world.GetColor([x,y,z]) != 'EMPTY'):
            if i % 3 == 1 and yMissing == True:
                y += 1                
            if i % 3 == 2 and xMissing == True:
                x+=1
            if i % 3 == 0 and zMissing == True:
                z+=1

            i+= 1

        return [x,y,z],color
    
    def __getActionsForCompleteGoal(self, world,goalState,color):
        
        if world.GetColor(goalState) == color:
            print('Goal = {} is already having a block with goal color = {}'.format(goalState, color))
            return []
        currentHeight = world.GetMaxHeight(goalState)        
        x, goalHeight, z  = goalState[0],goalState[1],goalState[2] 

        print(goalHeight, currentHeight)
        #dronePos = world.GetDronePosition()
        
        actions = []
        #if the target location is empty and there is a supporting block below
        if (currentHeight == goalHeight-1) or (goalHeight == 0  and currentHeight == goalHeight):
            print('goal state is empty and no blocks need to be moved....')
            sourcePos,sourcePosActions = self.__identifySourcePosition(world,goalState,color)
            
            actions += sourcePosActions
            
            actions += self.__getActions(world, sourcePos, goalState,color)
        
        # if the target location does not have supporting block below
        elif currentHeight < goalHeight-1:

            print('The plane of the goal state does not have the required minimum height so blocks need to be placed.. = ',goalHeight-currentHeight-1)
            # get blocks near by that can be place to achieve the desired height
            neighbours = self.__getBlocks(goalHeight-currentHeight-1,world,goalState,color)
            
            #identify source position that can be moved to goal state
            sourcePos,sourcePosActions = self.__identifySourcePosition(world,goalState,color)
            
            height = currentHeight+1
            #create actions for neighbours and source block
            for neighbor in neighbours:
                #get block color
                blockColor = world.GetColor(neighbor)
                
                actions += self.__getActions(world, neighbor, [x,height,z],blockColor)
                
                height += 1
            
            actions += sourcePosActions
            
            actions += self.__getActions(world, sourcePos, goalState,color)
        
        #if the target location is occupied and there are blocks on top of it.
        elif currentHeight > goalHeight-1:
            print('The plane of the goal state has greater than the required height so blocks need to be removed..',currentHeight-goalHeight+1)
            # get Empty locations near by so that that blocks can be moved to achieve the desired height
            neighbours = self.__getEmptyLocations(currentHeight-goalHeight+1,world,goalState,color)
                        
            height = currentHeight
            saved = []
            #create actions for neighbours and source block
            for neighbor in neighbours:
                
                #get block color
                blockColor = world.GetColor([x,height,z])
                
                actions += self.__getActions(world,  [x,height,z],neighbor,blockColor)
                
                
                #if block color matches with goal color then save it
                if blockColor == color:
                    saved.append(neighbor)
                height -= 1
                
            #identify source position that can be moved to goal state
            if saved != []:
                sourcePos = self.__getNearestPositions(saved,goalState)
            else:
                sourcePos,sourcePosActions = self.__identifySourcePosition(world,goalState,color)
            
                # perform actions to move any blocks required for source block to be available to move
                actions += sourcePosActions
            
            #perform actions required to move the source block
            actions += self.__getActions(world, sourcePos, goalState,color)
        
        return actions

    def __identifySourcePosition(self,world,goalState,color):
        
        '''sourceLoc = world.GetLocationsOfMovableBlock(color, self.__goalStates)
        
        print(sourceLoc)
        # if there is a block already available in the upper level, we will pick the one that is is nearer
        if sourceLoc != []:
            dists = [(index,EuclideanDistance(goalState, index)) for index in sourceLoc]
            dists = sorted(dists, key = lambda i : i[1])            
                           
            return dists[0][0] ,[]   '''
        
        #if there are no blocks of the color readily available to move.
        heights = [(index, world.GetMaxHeight(index)-index[1]) for index in world.GetAvailableBlocks(color,self.__goalStates)]
        heights = sorted(heights, key = lambda i : i[1])
        
        print("Avilable :",world.GetAvailableBlocks(color,self.__goalStates))

        pos, h = heights[0][0], world.GetMaxHeight(heights[0][0])
        print('Identifying source position = ', pos ,h, world.GetColor(pos))
        
        print('Identifying {} neighnours  to move the blocks in order to free the source blokc'.format(h-pos[1]))
        
        # get Empty locations near by so that that blocks can be moved to achieve the desired height
        neighbours = self.__getEmptyLocations(h-pos[1],world,goalState,color)
                
        print('Neighbours = ', neighbours)
        height = h
        actions = []
        x,z = pos[0],pos[2]
        
        
        #create actions for neighbours and source block
        for neighbor in neighbours:
            #print(height, len(neighbours))
            #get block color
            blockColor = world.GetColor([x,height,z])
            
            actions += self.__getActions(world,  [x,height,z],neighbor,blockColor)
            height -= 1
        
        print('actions idenitfied for sourceposition = ',actions)
        return pos, actions
    
    def __getActions(self,world, source, goal,blockColor):
        
        return [('Drone',source,blockColor),('Block',source,goal,blockColor)]
    
    def __getBlocks(self,k,world,goalState,color):
        '''2) when blocks need to be placed
                a) pick  planes with height >= height(Xg,Zg) and height <= maxHeight(world) + 1                
                b) identify the K planes with minimum euclidian distance from goal (Xg,Zg) plane'''
               
        return world.GetPositions(goalState, k, color,  ('Blocks',self.__goalStates))    
    
    def __getEmptyLocations(self,k,world,goalState,color):
        '''1) when blocks need to be removed
                a) pick  planes with height <= height(Xg,Zg) or height <= maxHeight(world) + 1                
                b) identify the K planes with minimum euclidian distance from goal (Xg,Zg) plane'''
        return world.GetPositions(goalState, k, color ,('Empty',self.__goalStates))        
    
    def __getNearestPositions(self,saved,goalState):
        
        dists = [(index,EuclideanDistance(goalState, index)) for index in saved]
        dists = sorted(dists, key = lambda i : i[1])
        
        #first one is the min distance point
        return dists[0][0]
    
    def __performActions(self,actions):
        
        if actions == []:
            print('No Actions to perform')
            return
        print('Actions to perform are = ', actions)
        
        for action in actions:            
            if action[0] == 'Drone':
                goal = [action[1][0],action[1][1]+1,action[1][2]]
                self.__action(self.world.GetDronePosition(), goal, False)
            
            if action[0] == 'Block':
                self.__action(action[1], action[2], True)
    
    def __action(self, startPos, goalPos, hasBlock):
        
        print('Identifying the path for startPos = {} and goalPos = {} for {}'.format(startPos,goalPos, 'Drone' if hasBlock == False else self.world.GetColor(startPos)))
        
        start_time = time.time()
        
        
        dx,dy,dz = (goalPos[0]-startPos[0], goalPos[1]-startPos[1],goalPos[2]-startPos[2])
        if dx in [-1, 0, 1] and dy in [-1, 0, 1] and dz in [-1, 0, 1]:
            moved = self.world.Move(startPos, goalPos,hasBlock)
            print('{} has to move only one position. so moved successfully'.format( 'Drone' if hasBlock == False else self.world.GetColor(startPos) ))
            return    
                

        path,_ = self.SearchAlgo.Search(startPos,goalPos,self.world, isDrone = (hasBlock == False))
        
        self.__calculateTime(start_time)
        
        print('path identified successfully with length = {}...... path = {}\n current Drone Pos = {} '.format(len(path),path,world.GetDronePosition()))
        
        #Attach drone to the block
        if hasBlock == True:            
            success = self.world.Attach()
            if success == True:
                print('Block attached successfully')
            else:
                print('failed to attach block')

        print('Performing moves in the world using obtained path')
        
        steps = len(path)
        i = 1
        oldPos = startPos                
        while(i < steps):
            newPos = path[i]
            
            #identify the change in the Positions
            #dx,dy,dz = (newPos[0]-oldPos[0], newPos[1]-oldPos[1],newPos[2]-oldPos[2])
            

            #Perform the move in the world.
            moved = self.world.Move(oldPos, newPos,hasBlock)
            
            #print('Successfully moved the block')

            if moved == False:
                print('Move operation failed.. Terminating')
                exit
            
            oldPos = newPos
            
            i+= 1
            
        if hasBlock == True:
            success = self.world.Release(goalPos)
            if success == True:
                print('Block released successfully')
        
        print('Successfully performed moves')
        print('Current Drone Pos = ', self.world.GetDronePosition())
        print('Current world is , ',self.world.OccupiedPos)
        print('Count of empty in the world : ',self.world.getcountempty())
        
    def __calculateTime(self, startTime):        
        print("--- time take %s seconds  = {} ---\n" % (time.time() - startTime))
    
            
        
if __name__ == '__main__':
    
    #goalState = '(6,0,-27,yellow)'
    
    world = DroneSimulator(100,50,100)
    world.Initialise('input3.txt')   
    #world.Initialise('myInput.txt')   
    hueristics = HeuristicFunctions()
    astar = AStartSearch(lambda x,y : hueristics.hf2(x,y))
    #astar = RAStarSearch(lambda x,y : hueristics.hf2(x,y))

    goalStates = world.ReadGoalFile("output5.txt")
    #goalStates = world.ReadGoalFile("myOutput.txt")  


    
    pathFinder = PathFinder(world,astar)
    #for mygoal in goalStates:
     #   print("****************************************")
      #  print("searching for goal state : ",mygoal)
       # print("****************************************")
    pathFinder.AchieveGoalStates(goalStates)

    
    nGrid = np.asarray(world.Grid)
    xi, yi, zi = np.where(nGrid != 'EMPTY')
    indices = [[x, y, z] for x, y, z in zip(*(xi, yi, zi))]
    print(indices)