# -*- coding: utf-8 -*-
"""
Created on Tue May  8 11:58:20 2018

@author: Avi
"""

#import nltk
from nltk.tokenize import sent_tokenize, word_tokenize
import os
from nltk.tag.stanford import StanfordPOSTagger
from Constants import Colours,PositionalWords, Adjective, Digit, Noun,Nouns,Preposition,Adverb, P_Noun,Colours_Plural
from DroneWorld import DroneSimulator
import random

class TextParser:
    
    def __init__(self, model, tagger):
        self.__tagger = StanfordPOSTagger(model, tagger)
    
    def GetTaggers(self, line):
        
        sent_token = sent_tokenize(line)        
        
        tags = []
        for sent in sent_token:
            word_tokens = word_tokenize(sent)
            
            #print('tokens = ', word_tokens)            
            
            tagged = self.__tagger.tag(word_tokens)
            
            tags += tagged
            
            #print(tagged)
        return [(str.upper(tags[i][0]),tags[i][1],i) for i in range(len(tags))]
    
    def GetTokensWithTag(self,tags,posTag):
        return list(filter(lambda x : x[1] in posTag, tags))
    
    def GetItems(self, tags, categoryList):
        #print(tags)
        return [word for word in tags if str.upper(word[0]) in categoryList]


class GoalIdentifier:
    
    def __init__(self,x,z,world):
        self.__x = x
        self.__z = z
        self.__world = world
    
    def __top(self, colors):
         
        return [(self.__x,i,self.__z,colors[i][0]) for i in range(len(colors))]
    
    def __height(self, colors, height):        
        return [(self.__x+i,height[0],self.__z+i,colors[i][0]) for i in range(len(colors))]  

    def __height2(self, color, height, quantity):
        return [(self.__x+i,height[0],self.__z+i,color[0]) for i in range(len(quantity)-1)] 
    
    def __height3(self,colors,digits):
        return [(self.__x+i,digits[i][0],self.__z+i,colors[i][0]) for i in range(len(colors))]  
    
    def __side(self, colors):
        return [(self.__x+i,0,self.__z+i,colors[i][0]) for i in range(len(colors))]
    
    def __Xaxis(self,color,units):
        pos = random.choice(self.__world.GetBlock(color[0]))
        #print(pos)
        return [(pos[0]+units,pos[1],pos[2],color[0])]
    
    def __Yaxis(self, color, units):
        positions = self.__world.GetBlock(color[0])        
        pos = random.choice(positions)
        return [(pos[0],pos[1]+units,pos[2],color[0])]
    
    def __getPositions(self, colors, positions):
        
        filled = []
        pos = []
        for color in colors:
            color_list = list(filter(lambda x : self.__world.GetColor(x) == color[0] and x not in filled,positions))
            pos.append(color_list[0]) 
        return pos
    def __Xaxis2(self, colors, units,direction):
        positions = self.__world.GetAllBlocks()        
        
        avilable_positions = self.__getPositions(colors,positions)
        
        if direction == '+':
            return [(avilable_positions[i][0]+int(units[i][0]),avilable_positions[i][1],avilable_positions[i][2],colors[i][0]) for i in range(len(colors))]
        
        return [(avilable_positions[i][0]-int(units[i][0]),avilable_positions[i][1],avilable_positions[i][2],colors[i][0]) for i in range(len(colors))]
    
        
    def __Yaxis2(self, colors, units,direction):
        positions = self.__world.GetAllBlocks()        
        
        avilable_positions = self.__getPositions(colors,positions)
        
        if direction == '+':
            return [(avilable_positions[i][0],avilable_positions[i][1]+int(units[i][0]),avilable_positions[i][2],colors[i][0]) for i in range(len(colors))]
        return [(avilable_positions[i][0],avilable_positions[i][1]-int(units[i][0]),avilable_positions[i][2],colors[i][0]) for i in range(len(colors))]
            
    def GetActions(self):
        
        return {'TOP': self.__top,
                'HEIGHT': self.__height,
                'HEIGHT_2': self.__height2,
                'HEIGHT_3':self.__height3,
                'SIDE': self.__side,
                'X':self.__Xaxis,
                'Y':self.__Yaxis,
                'X2':self.__Xaxis2,
                'Y2':self.__Yaxis2}

def getIndexKey(item):
    return item[2]

def GetSingular(colors, Colours_Plural):
    newList = []
    for color in colors:
        if color[0] in Colours_Plural:
            newList.append((color[0][:len(color[0])-1],color[1],color[2]))
        else:
            newList.append(color)
    
    return newList

def VerifyColors(colors,world):
    
    world_colors = list(world.colors.keys())
    
    for color in colors:
        if color[0] not in world_colors:
            return False,color[0]
    return True,None
    
if __name__ == '__main__':
    _path_to_model =  os.curdir+'\stanford-postagger\models\english-bidirectional-distsim.tagger'
    _path_to_jar =  os.curdir+'\stanford-postagger\stanford-postagger.jar'
    parser = TextParser(_path_to_model, _path_to_jar)
    
    file = open('output1.txt','r')
    lines = file.read()
    
    world = DroneSimulator(100,50,100)
    world.Initialise('input1.txt')
    goalId = GoalIdentifier(0,0,world)
    

    for line in lines.strip().split('\n'):
        
        # skip the commented line
        if line[0] == '#':
            continue
        
              
        tags = parser.GetTaggers(line)
        print('Sentence = ', line) 
        #print('Final tags are = ', tags)        
        #print('Adjectives = ', parser.GetTokensWithTag(tags, Adjective))
        #print('Digits = ', parser.GetTokensWithTag(tags, Digit))
        
        # seperate the tags colors
        adjs = parser.GetTokensWithTag(tags, [Adjective])
        nouns = parser.GetTokensWithTag(tags, [Noun,Nouns,P_Noun])
        prepositions = parser.GetTokensWithTag(tags, [Preposition])
        digits = parser.GetTokensWithTag(tags, [Digit])
        adVerbs = parser.GetTokensWithTag(tags, [Digit,Adverb])
        
        colors = parser.GetItems(adjs+nouns, Colours)
        positional_words = parser.GetItems(nouns+prepositions+adVerbs,PositionalWords) 
        
        colors = GetSingular(colors, Colours_Plural)
        
        valid,color = VerifyColors(colors,world)
        
        if valid == False:
            print('Color {} mentioned in the input does not exist in the world. Available colors are {}'.format(color,list(world.colors.keys())))
        else:    
            #print('Colors in the sentence are  = ', colors)
            #print('Positional Words in the sentence are  = ', positional_words)
            
            
            actions = goalId.GetActions()
            
            goals = []
            for pw in positional_words:
                word = pw[0]
                if word in ['BELOW','TOP']:
                    left_colors = list(filter(lambda c: c[2] < pw[2], colors))
                    right_colors = list(filter(lambda c: c[2] > pw[2], colors))
                    
                    if word == 'TOP':
                        goals += actions['TOP'](right_colors+left_colors)
                    if word == 'BELOW':
                        goals += actions['TOP'](left_colors+right_colors)
                        
                if word == 'HEIGHT':
                    index = min(colors,key = getIndexKey)
                    quantity = list(filter(lambda h : h[2]<index[2], digits))
                    if quantity == [] and len(digits) == 1:                    
                        goals += actions['HEIGHT'](colors,digits[0]) 
                    
                    elif quantity != [] and len(colors) == 1:                                            
                        goals+= actions['HEIGHT_2'](colors[0], digits[1], quantity[0])
                    elif quantity == [] and len(colors) == len(digits):
                        goals+= actions['HEIGHT_3'](colors,digits)
                    
                  
                if word == 'SIDE':
                    goals += actions['SIDE'](colors)
                
                if word == 'RIGHT':
                    if len(colors) == 1 and len(digits) == 1:
                        units = list(filter(lambda h : h[2]>colors[0][2] and h[2]<pw[2], digits))                    
                        if units != []:
                            goals += actions['X'](colors[0], int(units[0][0]))
                    
                    elif len(colors) == len(digits):
                        goals+= actions['X2'](colors,digits,'+')
                
                if word == 'LEFT':
                    if len(colors) == 1 and len(digits) == 1:
                        units = list(filter(lambda h : h[2]>colors[0][2] and h[2]<pw[2], digits))
                        if units != []:                        
                            goals += actions['X'](colors[0], -int(units[0][0]))
                    elif len(colors) == len(digits):
                        goals+= actions['X2'](colors,digits,'-')
                        
                if word == 'UP':
                    if len(colors) == 1 and len(digits) == 1:
                        units = list(filter(lambda h : h[2]>colors[0][2] and h[2]<pw[2], digits))
                        if units != []:                        
                            goals += actions['Y'](colors[0], int(units[0][0]))
                    elif len(colors) == len(digits):
                        goals+= actions['Y2'](colors,digits,'+')
                if word == 'DOWN':
                    if len(colors) == 1 and len(digits) == 1:
                        units = list(filter(lambda h : h[2]>colors[0][2] and h[2]<pw[2], digits))
                        print(units)
                        if units != []:                        
                            goals += actions['Y'](colors[0], -int(units[0][0]))
                    elif len(colors) == len(digits):
                        goals+= actions['Y2'](colors,digits,'-')        
            if goals == []:
                print('I am sorry I could not understand you, Could you please rephrase with out ambiguity?')
            else:
                print('Goals from text are = ', goals)
            

