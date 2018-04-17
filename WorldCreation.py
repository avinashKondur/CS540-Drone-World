# -*- coding: utf-8 -*-
"""
Created on Sun Feb 18 14:40:13 2018

@author: Avi
"""
import random

class CreateWorld:
    
    def __init__(self):
        pass
    
    def CreateWorld(self):
        
        filename = 'sampleworld.txt'
        file = open(filename, "w")
        count = input("How many pillars required: ")
        colors = input("List of colors with comma seperated : ")
        
        colors = colors.split(',')
        
        for i in range(int(count)):
            x = random.choice(list(range(-50,50)))
            y = random.choice(list(range(0,50)))            
            z = random.choice(list(range(-50,50)))
            
            for yV in range(y):
                file.write("(" + str(x) + "," + str(yV) + "," + str(z) + "," +  random.choice(colors)+")\n")
        
        x = random.choice(list(range(-50,50)))
        y = random.choice(list(range(0,50)))            
        z = random.choice(list(range(-50,50)))
        
        file.write("(" + str(x) + "," + str(yV) + "," + str(z) + ","  +"drone)\n")
        

if __name__ == '__main__':
    
    world = CreateWorld()
    world.CreateWorld()
        
            
        
        
    