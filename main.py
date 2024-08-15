
import time as tm
import tracemalloc as tcml
from queue import PriorityQueue 

class Graph:
    def __init__(self, vertices):
        self.vertices = vertices
        self.graph = [[0 for _ in range(vertices)] for _ in range(vertices)]
        self.heuristic = [0 for _ in range(vertices)]
        
    def printGraph(self):
        for i in range(self.vertices):
            print(self.graph[i])
        print(self.heuristic)

    #-------------------------------------------------#
    # Blind Search Algorithms: BFS, DFS, IDS
        # Breath First Search
    def BFS(self, start, goal) -> tuple:
        startTime = tm.time()
        tcml.start()
        inFrontier = [False for _ in range(self.vertices)]
        frontier = []
        backpointer = [-1 for _ in range(self.vertices)]
        frontier.append(start)
        inFrontier[start] = True
        while frontier:
            current = frontier.pop(0)
            for i in range(self.vertices):
                if self.graph[current][i] != 0 and not inFrontier[i]:
                    frontier.append(i)
                    inFrontier[i] = True
                    backpointer[i] = current
                    if i == goal:
                        break
            if inFrontier[goal]:
                break
        if inFrontier[goal]:
            path = []
            current = goal
            while current != -1:
                path.append(current)
                current = backpointer[current]
            path.reverse()
        else:
            path = -1
        memory = tcml.get_traced_memory()[0] / 1024
        tcml.stop()
        endTime = tm.time()
        runtime = (endTime - startTime) * 1000
        return [path, runtime, memory]
    #-------------------------------------------------#
        # Depth First Search
    def DFS(self, start, goal) -> tuple:
        tcml.start()
        startTime = tm.time()
        path = []
        frontier = []
        visited = [False for _ in range(self.vertices)]
        backpointer = [-1 for _ in range(self.vertices)]
        frontier.append(start)
        havePath = False
        while frontier:
            current = frontier.pop()
            if visited[current]:
                continue
            visited[current] = True
            for i in range(self.vertices-1, -1, -1):
                if self.graph[current][i] != 0 and not visited[i]:
                    frontier.append(i)
                    backpointer[i] = current
                    if i == goal:
                        havePath = True
                        break;
            if havePath:
                break
        if havePath:
            path = []
            current = goal
            while current != -1:
                path.append(current)
                current = backpointer[current]
            path.reverse()
        else:
            path = -1 
        memory = tcml.get_traced_memory()[0] / 1024
        tcml.stop()
        endTime = tm.time()
        runtime = (endTime - startTime) * 1000
        return [path, runtime, memory]
    #-------------------------------------------------#
        # Depth Limited Search recursive
    def DLS(self, start, goal, limit, visited) -> list:
        if start == goal:
            return [start]
        if limit == 0:
            return []
        visited[start] = True
        for i in range(self.vertices):
            if self.graph[start][i] != 0 and not visited[i]:
                path = self.DLS(i, goal, limit-1, visited)
                if path:
                    path.insert(0, start)
                    return path
        return []

        # Iterative Deepening Search    
    def IDS(self, start, goal) -> tuple:
        startTime = tm.time()
        tcml.start()
        limit = 0
        path = []
        while limit <= self.vertices:
            visited = [False for _ in range(self.vertices)]
            path = self.DLS(start, goal, limit, visited)
            if path:
                break
            limit += 1
        if not path:
            path = -1          
        memory = tcml.get_traced_memory()[0] / 1024
        tcml.stop()
        endTime = tm.time()
        runtime = (endTime - startTime) * 1000
        return [path, runtime, memory]
    
    #-------------------------------------------------#
    # Heuristic Search Algorithms: GBFS, UCS, A*
        # Greedy Breath First Search
    def GBFS(self, start, goal) -> tuple:
        #use priority queue
        startTime = tm.time()
        tcml.start()
        frontier = PriorityQueue()
        frontier.put((self.heuristic[start], start))
        visited = [False for _ in range(self.vertices)]
        backpointer = [-1 for _ in range(self.vertices)]
        havePath = False
        while not frontier.empty():
            current = frontier.get()[1]
            if visited[current]:
                continue
            visited[current] = True
            for i in range(self.vertices):
                if self.graph[current][i] != 0 and not visited[i]:
                    frontier.put((self.heuristic[i], i))
                    backpointer[i] = current
                    if i == goal:
                        havePath = True
                        break
            if havePath:
                break
        if havePath:
            path = []
            current = goal
            while current != -1:
                path.append(current)
                current = backpointer[current]
            path.reverse()
        else:
            path = -1
        memory = tcml.get_traced_memory()[0] / 1024
        tcml.stop()
        endTime = tm.time()
        runtime = (endTime - startTime) * 1000
        return [path, runtime, memory]
    #-------------------------------------------------#
        # Uniform Cost Search
    def UCS(self, start, goal) -> tuple:
        startTime = tm.time()
        tcml.start()
        frontier = PriorityQueue()
        cost = [1000000 for _ in range(self.vertices)]
        frontier.put((0, start))
        visited = [False for _ in range(self.vertices)]
        backpointer = [-1 for _ in range(self.vertices)]
        havePath = False
        while not frontier.empty():
            value, current = frontier.get()
            if current == goal:
                havePath = True
                break
            if visited[current]:
                continue
            visited[current] = True
            for i in range(self.vertices):
                if self.graph[current][i] != 0 and not visited[i]:
                    ivalue = value + self.graph[current][i]
                    if ivalue < cost[i]:
                        cost[i] = ivalue
                        frontier.put((ivalue, i))
                        backpointer[i] = current
        if havePath:
            path = []
            current = goal
            while current != -1:
                path.append(current)
                current = backpointer[current]
            path.reverse()
        else:
            path = -1
        memory = tcml.get_traced_memory()[0] / 1024
        tcml.stop()
        endTime = tm.time()
        runtime = (endTime - startTime) * 1000
        return [path, runtime, memory]
    #-------------------------------------------------#
        # A* Search
    def Astar(self, start, goal) -> tuple:
        startTime = tm.time()
        tcml.start()
        frontier = PriorityQueue()
        cost = [1000000 for _ in range(self.vertices)]
        frontier.put((0, start))
        visited = [False for _ in range(self.vertices)]
        backpointer = [-1 for _ in range(self.vertices)]
        havePath = False
        while not frontier.empty():
            value, current = frontier.get()
            value -= self.heuristic[current]
            if current == goal:
                havePath = True
                break
            if visited[current]:
                continue
            visited[current] = True
            for i in range(self.vertices):
                if self.graph[current][i] != 0 and not visited[i]:
                    ivalue = value + self.graph[current][i] + self.heuristic[i]
                    if ivalue < cost[i]:
                        cost[i] = ivalue
                        frontier.put((ivalue, i))
                        backpointer[i] = current
        if havePath:
            path = []
            current = goal
            while current != -1:
                path.append(current)
                current = backpointer[current]
            path.reverse()
        else:
            path = -1            
        memory = tcml.get_traced_memory()[0] / 1024
        tcml.stop()
        endTime = tm.time()
        runtime = (endTime - startTime) * 1000
        return [path, runtime, memory]
    
    #-------------------------------------------------#
    # Local Search
        # Hill climbing
    def hillClimbing(self, start, goal) -> tuple:
        startTime = tm.time()
        tcml.start()
        current = start
        path = [current]
        while current != goal:
            next = current
            for i in range(self.vertices):
                if self.graph[current][i] != 0 and i != current:
                    if self.heuristic[i] < self.heuristic[next]:
                        next = i
            if next == current:
                break
            path.append(next)
            current = next
        if current != goal:
            path = -1
        memory = tcml.get_traced_memory()[0] / 1024
        tcml.stop()
        endTime = tm.time()
        runtime = (endTime - startTime) * 1000
        return [path, runtime, memory]
               

def printResult(path, runtime, memory, file):
    file.write("Path: ")
    if path == -1:
        file.write(str(path) + "\n")
    else:
        for i in range(len(path)-1):
            file.write(str(path[i]) + " -> ")
        file.write(str(path[-1]) + "\n")
    file.write(f"Time:  {runtime:.16f} miliseconds\n")
    file.write(f"Memory: {memory:.2f} KB\n")

def run(inputFile, outputFile):
    with open(inputFile, 'r') as file:
        n = int(file.readline())
        #next line is start and goal
        start, goal = list(map(int, file.readline().split()))
        graph = Graph(n)
        for i in range(n):
            graph.graph[i] = list(map(int, file.readline().split()))
        graph.heuristic = list(map(int, file.readline().split()))
    
    with open(outputFile, 'w') as file:
        file.write("BFS\n")
        path, time, memory = graph.BFS(start, goal)
        printResult(path, time, memory, file)
        file.write("---------------\n")
        file.write("DFS\n")
        path, time, memory = graph.DFS(start, goal)
        printResult(path, time, memory, file)
        file.write("---------------\n")
        file.write("UCS\n")
        path, time, memory = graph.UCS(start, goal)
        printResult(path, time, memory, file)
        file.write("---------------\n")
        file.write("IDS\n")
        path, time, memory = graph.IDS(start, goal)
        printResult(path, time, memory, file)
        file.write("---------------\n")
        file.write("GBFS\n")
        path, time, memory = graph.GBFS(start, goal)
        printResult(path, time, memory, file)
        file.write("---------------\n")
        file.write("A*\n")
        path, time, memory = graph.Astar(start, goal)
        printResult(path, time, memory, file)
        file.write("---------------\n")
        file.write("Hill Climbing\n")
        path, time, memory = graph.hillClimbing(start, goal)
        printResult(path, time, memory, file)

        print("> Done running for " + inputFile)
        
def runSetofData(size):
    print("Running for " + size + " data")
    folder = "self-test/" + size;
    run(folder + "/random_graph.txt", folder + "/random_graph-out.txt")
    run(folder + "/admissible.txt", folder + "/admissible-out.txt")
    run(folder + "/inadmissible.txt", folder + "/inadmissible-out.txt")
    run(folder + "/cycle_graph.txt", folder + "/cycle_graph-out.txt")
    run(folder + "/sparse_graph.txt", folder + "/sparse_graph-out.txt")
    print("Done running for " + size + " data")
    print("-------------------------------------------------")
    
    

if __name__ == "__main__":
    run("input.txt", "output.txt")
    runSetofData("small")
    runSetofData("medium")
    runSetofData("large")
    
    