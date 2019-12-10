import sys
import numpy as np
from collections import defaultdict

# helper function: counterclockwise
def ccw(A,B,C):
    return (C[1]-A[1]) * (B[0]-A[0]) > (B[1]-A[1]) * (C[0]-A[0])

# Return true if line segments AB and CD intersect
def intersect(A,B,C,D):
    return ccw(A,C,D) != ccw(B,C,D) and ccw(A,B,C) != ccw(A,B,D)

# return euclidean distance of two points
def distance(a, b):
    return np.sqrt((a[0] - b[0])**2 + (a[1] - b[1])**2)

def isReflexive(P1, P2, P3):

    x1 = P1[0] - P2[0]
    y1 = P1[1] - P2[1]
    x2 = P3[0] - P2[0]
    y2 = P3[1] - P2[1]

    dot = x1 * x2 + y1 * y2  # dot product
    det = x1 * y2 - y1 * x2  # determinant
    angle = np.arctan2(det, dot)

    #print(P2, angle)
    return angle > 0
'''
Report reflexive vertices
'''
def findReflexiveVertices(polygons):
    vertices = []
    for polygon in polygons:

        #edge case for first point
        if isReflexive(polygon[-1], polygon[0], polygon[1]):
            vertices.append(polygon[0])

        for i in range(0, len(polygon)-2):
            if isReflexive(polygon[i], polygon[i+1], polygon[i+2]):
                vertices.append(polygon[i+1])

        #edge case for last point
        if isReflexive(polygon[-2], polygon[-1], polygon[0]):
            vertices.append(polygon[-1])
    
    return vertices

def isBiTangent(P1, P2, polygons):

    for polygon in polygons:
        for i in range(0, len(polygon)):
            for j in range(0, len(polygon)):
                if not (P1 == polygon[i]) and not (P1 == polygon[j]) and \
                        not (P2 == polygon[i]) and not (P2 == polygon[j]):
                    if intersect(P1, P2, polygon[i], polygon[j]):
                        return False
    return True

'''
Compute the roadmap graph
'''
def computeSPRoadmap(polygons, reflexVertices):
    vertexMap = dict()
    adjacencyListMap = defaultdict(list)

    # create a dictionary for the vertex map
    for i in range(0, len(reflexVertices)):
        vertexMap[i + 1] = reflexVertices[i]

    for i in range(1, len(vertexMap)+1):
        for j in range(1, len(vertexMap)+1):

            if i == j:
                continue

            A1 = vertexMap[i]
            B1 = vertexMap[j]

            if isBiTangent(A1, B1, polygons):
                #good edge
                dist = distance(A1, B1)
                adjacencyListMap[i].append([j, dist])
    
    return vertexMap, adjacencyListMap

'''
Perform uniform cost search 
'''
def uniformCostSearch(adjListMap, start, goal):
    path = []
    pathLength = 0

    distance_from_start = {start: 0}
    previous = {start: start}
    queue = [start]
    while queue:
        minVal = float('inf')
        minIndex = float('inf')
        for q in range(0,len(queue)):
            if minVal > distance_from_start[queue[q]]:
                minVal = distance_from_start[queue[q]]
                minIndex = q

        nextPoint = queue.pop(minIndex)
        if nextPoint == goal:
            x = -1
            path.append(x)
            while x != 0:
                path.insert(0, previous[x])
                x = previous[x]
            return path, pathLength

        neighbors = adjListMap[nextPoint]
        for n in neighbors:
            new_distance = distance_from_start[nextPoint] + n[1]
            if (n[0] not in previous) or (new_distance < distance_from_start[n[0]]):
                queue.append(n[0])
                previous[n[0]] = nextPoint
                pathLength = new_distance

            if ((n[0] not in distance_from_start)):
                distance_from_start[n[0]] = new_distance
                previous[n[0]] = nextPoint
                pathLength = new_distance

    return path, pathLength

'''
Agument roadmap to include start and goal
'''
def updateRoadmap(polygons, vertexMap, adjListMap, x1, y1, x2, y2):
    updatedALMap = dict()
    startLabel = 0
    goalLabel = -1

    terminalPoints = [[x1, y1], [x2, y2]]
    updatedALMap = adjListMap.copy()

    for i in range(-1, 1):
        adjacent = []
        for m in range(1, len(vertexMap) + 1):
            if isBiTangent(terminalPoints[i], vertexMap[m], polygons):
                dist = distance(terminalPoints[i], vertexMap[m])
                adjacent.append([m, dist])
                updatedALMap[m].append([i, dist])
        updatedALMap[i] = adjacent

    return startLabel, goalLabel, updatedALMap


if __name__ == "__main__":
    # Retrive file name for input data
    if(len(sys.argv) < 6):
        print("Five arguments required: python spr.py [env-file] [x1] [y1] [x2] [y2]")
        exit()
    
    filename = sys.argv[1]
    x1 = float(sys.argv[2])
    y1 = float(sys.argv[3])
    x2 = float(sys.argv[4])
    y2 = float(sys.argv[5])

    # Read data and parse polygons
    lines = [line.rstrip('\n') for line in open(filename)]
    polygons = []
    for line in range(0, len(lines)):
        xys = lines[line].split(';')
        polygon = []
        for p in range(0, len(xys)):
            polygon.append([float(i) for i in xys[p].split(',')])
        polygons.append(polygon)

    # Print out the data
    print("Pologonal obstacles:")
    for p in range(0, len(polygons)):
        print(str(polygons[p]))
    print("")

    # Compute reflex vertices
    reflexVertices = findReflexiveVertices(polygons)
    print("Reflexive vertices:")
    print(str(reflexVertices))
    print("")

    # Compute the roadmap 
    vertexMap, adjListMap = computeSPRoadmap(polygons, reflexVertices)
    print("Vertex map:")
    print(str(vertexMap))
    print("")
    print("Base roadmap:")
    print(dict(adjListMap))
    print("")

    # Update roadmap
    start, goal, updatedALMap = updateRoadmap(polygons, vertexMap, adjListMap, x1, y1, x2, y2)
    print("Updated roadmap:")
    print(dict(updatedALMap))
    print("")

    # Search for a solution     
    path, length = uniformCostSearch(updatedALMap, start, goal)
    print("Final path:")
    print(str(path))
    print("Final path length:" + str(length))

