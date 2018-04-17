import time
from DroneWorld import DroneSimulator
from Helpers import Node, ActionFunctions,HeuristicFunctions

class AStartSearch:
    
    def __init__(self,hF):
        #To keep track of number of nodes expanded while searching for goal state
        self.noOfNodes = 0
        self.Heuristic = hF
    
    def Search(self,startState,goalState,world, isDrone = False):
        visited = []
        unExplored = []       
        h = self.Heuristic(startState,goalState)
        startNode = Node(startState, f=0 + h, g=0, h=h)
        actions = ActionFunctions(goalState,isDrone)
        return self.aStarSearchHelper(startNode, goalState,actions.actionF, actions.takeActionF, actions.goalTestF, self.Heuristic, float('inf'), visited, unExplored,world)
    
    def aStarSearchHelper(self,parentNode, goalState,actionF, takeActionF, goalTestF, hF, fmax, visited, unExplored,world):
    
        if goalTestF(parentNode.state):
            return ([parentNode.state], parentNode.g)
    
        if parentNode.state in unExplored:
            unExplored.pop(unExplored.index(parentNode.state))
        if parentNode.state not in visited:
            visited.append(parentNode.state)
        actions = actionF(parentNode.state,world)
    
        if not actions:
            print(actions)
            return ("Path does not exists", float('inf'))
        children = []
    
        for action in actions:
            self.noOfNodes=self.noOfNodes+1
            (childState, stepCost) = takeActionF(parentNode.state, action)
            h = hF(childState,goalState)
            g = parentNode.g + stepCost
            f = max(h + g, parentNode.f)
            childNode = Node(state=childState, f=f, g=g, h=h)
            children.append(childNode)
    
            if (childNode.state not in visited) and (childNode.state not in unExplored):
                unExplored.append(childNode.state)
    
        while True:
            # print(children)
            children.sort(key=lambda n: n.f)  # sort by f value
            bestChild = children[0]
            if bestChild.f > fmax:
                return ("failure", bestChild.f)
            alternativef = children[1].f if len(children) > 1 else float('inf')
            result, bestChild.f = self.aStarSearchHelper(bestChild,goalState,actionF, takeActionF, goalTestF, hF, min(fmax, alternativef), visited, unExplored,world)
            if result is not "failure":
                result.insert(0, parentNode.state)
                return (result, bestChild.f)
            if unExplored == []:
                   return ("failure", float('inf'))
    
    def ebf(self,depth, precision=0.01):
        nNodes=self.noOfNodes
        lowerVal=0
        upperVal=nNodes
        finalBF=1
        minRange=upperVal
        estimatedValue = 0
        if nNodes <= 0 or depth < 0:  # No nodes dicovered
            return 0.000
        if nNodes == 1 and depth == 0:
            return 1
        while lowerVal <= upperVal:
    
            b = (upperVal + lowerVal) / 2
            if b != 1:
                estimatedValue = (1 - (b ** (depth + 1))) / (1 - b)  # 1+b+b^2+⋯+b^d
    
            rangeV = abs(estimatedValue - nNodes)
            if rangeV <= minRange:
                minRange = rangeV  # keeping track of minimum difference found between actual no. of nodes and estimated no. of nodes
                finalBF = b
            if rangeV <= precision:  # precision tells tolerable difference actual no. of nodes and estimated no. of nodes
                return finalBF
            if estimatedValue < nNodes:
                lowerVal = b
            elif estimatedValue > nNodes:
                upperVal = b
        return estimatedValue

if __name__ == '__main__':
    startState=[1,0,1]      #Assuming it to be the position of block
    goalState=[4,1,3]
    world = DroneSimulator(100, 50, 100)
    world.Initialise('data.txt')
    start_time = time.time()        
    hueristics = HeuristicFunctions(goalState)
    astar = AStartSearch(hueristics.hf)
    out,cost=astar.Search(startState,goalState, world)
    print("--- %s seconds ---" % (time.time() - start_time))
    print("path=",out,"\npath length=",len(out))
    print("Effective branching factor =",astar.ebf(cost,0.01))