#!/usr/bin/env python
from __future__ import print_function

class AstarNode():
    def __init__(self, position, parent = None):
        self.position = position
        self.parent = parent

        self.g = 0
        self.h = 0
        self.f = 0
    
class minHeap():
    def __init__(self):
        self.heap = []

    def __str__(self):
        return ' '.join([str(i) for i in self.heap])
    
    def insert(self, data):
        self.heap.append(data)
    
    def deleteMin(self):
        try:
            min = 0
            for i in range(len(self.heap)):
                if self.heap[i] < self.heap[min]:
                    min = i
            item = self.heap[min]
            del self.heap[min]
            return item 
        except IndexError:
            print()
            exit()



def main():
    H = minHeap()
    #H.insert(3)
    #H.insert(4)
    #H.insert(5)
    print(H.deleteMin())
    #print(H)


if __name__ == "__main__":
    main()