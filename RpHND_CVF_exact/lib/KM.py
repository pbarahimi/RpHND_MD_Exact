import sys

from numpy import loadtxt
from math import inf, sqrt
from platform import node
from scipy import cluster
import numpy as np
from numpy.core.defchararray import center

class K_medoids:
    '''
    classdocs
    '''
    def __init__(self, x, k, itr, distances):
        '''
        Constructor
        '''
        self.x = x
        self.k = k
        self.itr = itr
#         self.max_itr = max_itr
#         self.tol = tol
        self.distances = np.empty((len(x),len(x)))
        if distances is None:
            for i in range(len(x)):
                for j in range(len(x)):
                    self.distances[i][j] = self.getDist(x[i], x[j])
        else:           
            self.distances = distances
        
    def run(self):
        output = {}
        SSE = inf
        
        # repeat the algorithm 'itr' times to avoid local minimum
        for i in range(self.itr):
            # random selection of initial centers
            currentCenters = np.random.choice(len(self.x), self.k, replace=False)
            currentSSE = inf
            newCenters = [None]*self.k
                        
            # assignment of nodes to the centers
            (clusters,newSSE) = self.assignNodes(currentCenters);
            
            while currentSSE > newSSE:
                currentSSE = newSSE
                
                # swapping centers and nodes in each cluster to find new centers
                i = 0
                for center,cluster in clusters.items():
                    newCenters[i] = self.getNewCenter(center, cluster)
                    i += 1
                
                # assignment of nodes to the new centers and update SSE
                (clusters,newSSE) = self.assignNodes(newCenters)

            # update best SSE and centers if the current iteration yields a better SSE    
            if newSSE < SSE:
                SSE = newSSE
                output = clusters
        
        return output   
    
    def assignNodes(self,centers):
        clusters = {center_index:[] for center_index in centers}
        clustersSSE = 0
        for node in self.x:
            best_dist = inf
            for j in centers:
                if self.distances[node][self.x[j]] < best_dist:
                    best_dist = self.distances[node][self.x[j]]
                    closest_center = j
            clusters[closest_center].append(node)
            clustersSSE += best_dist
        return (clusters,clustersSSE)
       
    def getDist(self,x,y):
        output = 0
        for i in range( len(x) ):
            output += (x[i]-y[i])**2
        return sqrt(output)
    
    def getSSE(self,center, cluster):
        SSE = 0
        for node in cluster:
            SSE += self.distances[node][self.x[node]]
        return SSE
    
    def getNewCenter(self,oldCenter,cluster):        
        bestSSE = inf
        for node in cluster:
            temp_SSE = self.getSSE(node, cluster)
            if temp_SSE < bestSSE:
                new_center = node
        return new_center
filePath = sys.argv[1]
k = int(sys.argv[2])
distances = loadtxt(filePath,dtype=float)
N = len(distances)
itr = int(N/2)
clusters = K_medoids(range(N),k,itr,distances).run()
print(list(clusters.keys()))