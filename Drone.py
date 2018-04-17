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

class PathFinder:
    
    def __init__(self,world,searchAlgorithm):
        
        self.world = world
        self.SearchAlgo = searchAlgorithm
        self.__goalStates = [] 
        self.finalActions = dict()
    
    
    def AchieveGoalStates(self, goalStates):
        
        self.__goalStates = [goal[0] for goal in goalStates]
        
        for goal in goalStates:
            
            start_time = time.time()
            
            goalState, color = goal[0], goal[1]
            
            print('*************************************************************')
            print('Idendifying actions for goal state = {}'.format(goal))
            print('*************************************************************')
            
            #identify goalstate if incomplete and also all actions required to perform
            goalState, actions = self.__identifyActions(self.world,goalState, color)
            
            print('obtained actions ....\n Starting performing actions .... ')
            
            self.finalActions[tuple(goalState)] = actions
            
            #iterate through all the actions and perform actions
            self.__performActions(actions, self.world)
            
            print('All actions are performed.......')
            
            #add the new goalState to the list if it empty, so that next steps in identify the goalState will eliminate this.
            if goal[0].count('?') > 0 :
                self.__goalStates.append(goalState)  
            
            self.__calculateTime(start_time)
        
        
        
    def __identifyActions(self,world,goalState,color):
        
        #identify if goal is complete or not
        isGoalComplete = False if goalState.count('?') > 0 else True
        
        #check if color is mentioned or not
        isColorMentioned = False if color == '?' else True
        
        #Update color if not mentioned.
        if isColorMentioned == False:
            if isGoalComplete == True and world.IsPositionAvailable(goalState):
                color = world.GetColor(goalState)
            else:
                color = world.GetRandomAvailableColor()
                
        # identify goal state if it is incomplete        
        if isGoalComplete == False:
            print('Goal state is not complete... identifying the goal state')
            
            goalState,color = self.__identifyGoalState(world,goalState,color)
            
            print('Goalstate successfully identified, new goal state is {}, {}'.format(goalState,color))
        
        print('Identifying the actions on complete goal {}, {}'.format(goalState,color))
        
        #identify actions for the complete goal state
        actions = self.__getActionsForCompleteGoal(world,goalState,color)
        
        print('Actions are successfully identified inorder to achieve goalState {}'.format(actions))
        
        return goalState, actions
    
    def __identifyGoalState(self,world,goalState,color):
        #xMissing, yMissing, zMissing = goalState[0] == '?', goalState[1] == '?', goalState[2] == '?'
        isColorGiven = False if color == '?' else True
        
        possiblePos = world.checkGoalAlreadyReached(color, goalState,self.__goalStates)


        if possiblePos != []:
            newGoalState=random.choice(possiblePos)            
            if isColorGiven == False:
                color=world.GetColor(newGoalState)                
        else:
            saved=world.GetPossibleGoalPos(goalState)
            saved=world.hasSupportingBlock(saved)
            #avail = [m if m not in self.__goalStates else None for m in saved]
            possiblePos = list(filter(lambda a: a not in self.__goalStates, saved))
            newGoalState=self.__getNearestPositions(possiblePos,world.GetDronePosition())
            
        return newGoalState,color
    
    def __getActionsForCompleteGoal(self, world,goalState,color):
                
        currentHeight = world.GetMaxHeight(goalState)        
        x, goalHeight, z  = goalState[0],goalState[1],goalState[2] 
        #dronePos = world.GetDronePosition()
        
        actions = []
        #if the target location is empty and there is a supporting block below
        if currentHeight == goalHeight-1:
            sourcePos = self.__identifySourcePosition(world,goalState,color)
            actions = self.__getActions(world, sourcePos, goalState,color)
        
        # if the target location does not have supporting block below
        if currentHeight < goalHeight-1:
            # get blocks near by that can be place to achieve the desired height
            neighbours = self.__getBlocks(goalHeight-currentHeight-1,world,goalState,color)
            
            #identify source position that can be moved to goal state
            sourcePos,sourcePosActions = self.__identifySourcePosition(world,goalState,color)
            
            height = currentHeight+1
            #create actions for neighbours and source block
            for neighbor in neighbours:
                #get block color
                blockColor = world.GetColor([x,height,z])
                
                actions += self.__getActions(world, neighbor, [x,height,z],blockColor)
                
                height += 1
            
            actions += sourcePosActions
            
            actions += self.__getActions(world, sourcePos, goalState,color)
        
        #if the target location is occupied and there are blocks on top of it.
        if currentHeight > goalHeight-1:
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
        
        sourceLoc = world.GetLocationsOfMovableBlock(color)
        
        print(sourceLoc)
        # if there is a block already available in the upper level, we will pick the one that is is nearer
        if sourceLoc != []:
            dists = [(index,EuclideanDistance(goalState, index)) for index in sourceLoc]
            dists = sorted(dists, key = lambda i : i[1])            
                           
            return dists[0][0] ,[]   
        
        #if there are no blocks of the color readily available to move.
        heights = [(index, index[1]-world.GetMaxHeight(index)) for index in world.world.GetAvailableBlocks(color)]
        heights = sorted(heights, key = lambda i : i[1])
        
        pos, h = heights[0][0], world.GetMaxHeight(heights[0][0])
        
        # get Empty locations near by so that that blocks can be moved to achieve the desired height
        neighbours = self.__getEmptyLocations(pos[1]-h+1,world,goalState,color)
                    
        height = h
        actions = []
        x,z = pos[0],pos[2]
        
        #create actions for neighbours and source block
        for neighbor in neighbours:
            
            #get block color
            blockColor = world.GetColor([x,height,z])
            
            actions += self.__getActions(world,  [x,height,z],neighbor,blockColor)
            height -= 1
        
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
    
    def __performActions(self,actions, world):
        for action in actions:            
            if action[0] == 'Drone':
                self.__action(world, world.GetDronePosition(), action[1], False)
            
            if action[0] == 'Block':
                self.__action(world, action[1], action[2], True)
    
    def __action(self, world, startPos, goalPos, hasBlock):
        
        print('Identifying the path for startPos = {} and goalPos = {} for {}'.format(startPos,goalPos, 'Drone' if hasBlock == False else 'Block'))
        
        start_time = time.time()
        
        path,_ = self.SearchAlgo.Search(startPos,goalPos,world, isDrone = (hasBlock == False))
        
        self.__calculateTime(start_time)
        
        print('path identified successfully with length = {}...... path = {} '.format(len(path),path))
        
        #Attach drone to the block
        if hasBlock == True:            
            success = world.Attach()
            if success == True:
                print('Block attached successfully')
        
        print('Performing moves in the world using obtained path')
        
        steps = len(path)
        i = 0
        oldPos = startPos                
        while(i < steps):
            newPos = path[i]
            
            #identify the change in the Positions
            dx,dy,dz = (newPos[0]-oldPos[0], newPos[1]-oldPos[1],newPos[2]-oldPos[2])
            
            #Perform the move in the world.
            world.Move(dx,dy,dz)
            i+= 1
            
        if hasBlock == True:
            success = world.Release()
            if success == True:
                print('Block released successfully')
        
        print('Successfully performed moves')
        
    def __calculateTime(self, startTime):        
        print("--- time take %s seconds  = {} ---\n" % (time.time() - startTime))
    
            
        
if __name__ == '__main__':
    
    #goalState = '(6,0,-27,yellow)'
    
    world = DroneSimulator(100,50,100)
    world.Initialise('grid3.txt')   
    hueristics = HeuristicFunctions()
    astar = AStartSearch(lambda x,y : hueristics.hf2(x,y))
    #astar = RAStarSearch(lambda x,y : hueristics.hf2(x,y))

    goalStates = world.ReadGoalFile("test.txt")       
    
    pathFinder = PathFinder(world,astar)
    pathFinder.AchieveGoalStates(goalStates)