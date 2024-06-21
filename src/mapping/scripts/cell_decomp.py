from matplotlib import image
from matplotlib import pyplot as plt
import numpy as np

class node():
    def __init__(self,row,col,parent,cost):
       self.row=row
       self.col=col
       self.parent=parent
       self.cost=cost

    def equals(self,a):
        return [self.row,self.col]==[a.row,a.col]
    
    def toString(self):
        return[self.row,self.col]

def getDist(x,y):
    return ((x[0]-y[0])**2)+((x[1]-y[1])**2)

class PriorityQueue(object):
    def __init__(self,goal):
        self.queue = []
        self.goal = goal
 
    def __str__(self):
        return ' '.join([str(i) for i in self.queue])
 
    def isEmpty(self):
        return len(self.queue) == 0
 
    def insert(self, data):
        self.queue.append(data)
 
    def delete(self):
        min_val = 0
        for i in range(len(self.queue)):
            if self.queue[i].cost+getDist([self.queue[i].row,self.queue[i].col],self.goal) < self.queue[min_val].cost+getDist([self.queue[min_val].row,self.queue[min_val].col],self.goal):
                min_val = i
        item = self.queue[min_val]
        del self.queue[min_val]
        return item   
    


def segment_image_into_grid(image_path, kernel):
    myImage = image.imread(image_path)
    out = myImage.copy()
    row=0
    while(row+kernel<myImage.shape[0]):
        col=0
        while (col+kernel<myImage.shape[1]):
            signal=1
            for i in range(row,row+kernel):
                for j in range(col,col+kernel):
                    if myImage[i][j][0]==0.0:
                        signal=0

            for i in range(row,row+kernel):
                for j in range(col,col+kernel):
                    out[i][j]=[signal,signal,signal,1]

            col+=kernel
        
        row+=kernel

    return out



def AStar(start,goal,dMap):
    map= dMap.copy()
    visited=[]

    queue = PriorityQueue(goal)
    queue.insert(node(start[0],start[1],None,0))

    found=False

    while not(queue.isEmpty()):
        old=False

        curr=queue.delete()

        for i in visited:
            if(curr.equals(i)):
                old=True
                break
        
        if(old):
            continue

        visited.append(curr)

        if([curr.row,curr.col]==goal):
            visited.append(curr)
            found=True
            last=curr
            break
        
        if(curr.row+1<map.shape[0])and(map[curr.row+1][curr.col][0]!=0.0):
            next = node(curr.row+1,curr.col,curr,curr.cost+1)
            queue.insert(next) 

        if(curr.row-1>-1)and(map[curr.row-1][curr.col][0]!=0.0):
            next = node(curr.row-1,curr.col,curr,curr.cost+1)
            queue.insert(next) 

        if(curr.col+1<map.shape[1])and(map[curr.row][curr.col+1][0]!=0.0):
            next = node(curr.row,curr.col+1,curr,curr.cost+1)
            queue.insert(next) 

        if(curr.col-1>-1)and(map[curr.row][curr.col-1][0]!=0.0):
            next = node(curr.row,curr.col-1,curr,curr.cost+1)
            queue.insert(next) 


    path=[]

    if(found): 
       
       while(not last.equals(node(start[0],start[1],None,0))):
           path.append(last)
           
           for i in visited:
                if last.equals(i):
                    last=i.parent
                    break

    for i in path:
        map[i.row][i.col]=0.5

    return path,map
    
decomposedMap = segment_image_into_grid("map1.png", 7)
path,newMap = AStar([293,350],[188,147],decomposedMap)

#for i in path:
    #print(i.toString())

print(decomposedMap.shape)

plt.figure(figsize=(10, 5))

plt.subplot(1, 2, 1)
plt.title("Original Map")
plt.imshow(decomposedMap)

plt.subplot(1, 2, 2)
plt.title("Path")
plt.imshow(newMap)

plt.show()