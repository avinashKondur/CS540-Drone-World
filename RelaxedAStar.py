
import timeit
from DroneWorld import DroneSimulator
from Helpers import Node, ActionFunctions,HeuristicFunctions

class RAStarSearch:
    
    def __init__(self,hF):
        self.Heuristic = hF

    def costly_func(self):
       return map(lambda x: x^2, range(10))
        
    def Search(self,startState, goalState,world,isDrone = False):
        global num_nodesAstar
        num_nodesAstar = 0
        h = self.Heuristic (startState,goalState)
        startNode = Node(state=startState, f=0+h, g=0, h=h)
        c = 0
        actions = ActionFunctions(goalState,isDrone)
        self.RSearchFunc(startNode,goalState, actions.actionF, actions.takeActionF, actions.goalTestF, self.Heuristic , float('inf'),c,world)
        for i in range(0,len(retrace)):
                        PtoP.append(retrace[i][0].state)
        PtoP.reverse()
        PtoP.insert(0,startState) 
        return PtoP,num_nodesAstar
    
    
    def RSearchFunc(self,parentNode,goalState, actionsF, takeActionF, goalTestF, hF, fmax,c,world):
        global children,PtoP,retrace,num_nodesAstar
        if c==0:
            PtoP,retrace = [],[]
            c=c+1    
        if goalTestF(parentNode.state):
            return ([parentNode.state], parentNode.g)
        ## Construct list of children nodes with f, g, and h values
        actions = actionsF(parentNode.state,world)
        if not actions:
            return ("failure", float('inf'))
        children = []
        for action in actions:
            num_nodesAstar = num_nodesAstar + 1
            (childState,stepCost) = takeActionF(parentNode.state, action)
            h = hF(childState,goalState)
            g = (1-0.004)*(parentNode.g + stepCost)
            f = max(h+g, parentNode.f)
            childNode = Node(state=childState, f=f, g=g, h=h)
            children.append(childNode)
        while True:
            # find best child
            children.sort(key = lambda n: n.f) # sort by f value
            retrace.insert(0,children)
            bestChild = children[0]
            if bestChild.f > fmax:
                return ("failure",bestChild.f)
            # next lowest f value
            alternativef = children[1].f if len(children) > 1 else float('inf')
            # expand best child, reassign its f value to be returned value
            result,bestChild.f = self.RSearchFunc(bestChild,goalState,actionsF, takeActionF, goalTestF,hF, min(fmax,alternativef),c,world)
            if result is not "failure":         
                result.insert(0,parentNode.state)   
                return (result, bestChild.f)   
        
if __name__ == '__main__':
    startState=[1,0,1]
    goalState=[4,1,3]
    world = DroneSimulator(100, 50, 100);
    world.Initialise('data.txt');
    grid = world.Grid;
    print(world)
    
    hueristics = HeuristicFunctions(goalState)
    rastar = RAStarSearch(hueristics.hf_1)
    path,nodes = rastar.Search(startState, goalState,world)
    print(path)
    
    print('The time taken to find the path is %s seconds'%(float("{0:.9f}".format(timeit.timeit(rastar.costly_func)))))  
        
        
        
        
        
        
        
        