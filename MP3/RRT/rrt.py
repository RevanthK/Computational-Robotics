import sys

import matplotlib.pyplot as plt
from matplotlib.path import Path
import matplotlib.patches as patches
from matplotlib import collections as mc
import numpy as np

'''
Set up matplotlib to create a plot with an empty square
'''
def setupPlot():
    fig = plt.figure(num=None, figsize=(5, 5), dpi=120, facecolor='w', edgecolor='k')
    ax = fig.subplots()
    ax.set_axisbelow(True)
    ax.set_ylim(-1, 11)
    ax.set_xlim(-1, 11)
    ax.grid(which='minor', linestyle=':', alpha=0.2)
    ax.grid(which='major', linestyle=':', alpha=0.5)
    return fig, ax

'''
Make a patch for a single pology 
'''
def createPolygonPatch(polygon, color):
    verts = []
    codes= []
    for v in range(0, len(polygon)):
        xy = polygon[v]
        verts.append((xy[0], xy[1]))
        if v == 0:
            codes.append(Path.MOVETO)
        else:
            codes.append(Path.LINETO)
    verts.append(verts[0])
    codes.append(Path.CLOSEPOLY)
    path = Path(verts, codes)
    patch = patches.PathPatch(path, facecolor=color, lw=1)

    return patch

'''
Render the problem  
'''
def drawProblem(robotStart, robotGoal, polygons):
    _, ax = setupPlot()
    patch = createPolygonPatch(robotStart, 'green')
    ax.add_patch(patch)    
    patch = createPolygonPatch(robotGoal, 'red')
    ax.add_patch(patch)    
    for p in range(0, len(polygons)):
        patch = createPolygonPatch(polygons[p], 'gray')
        ax.add_patch(patch)    
    plt.show()

#Find the closest between a point P and a line segment from (A,B) in 2D
def closestPoint(P, A, B):

    A = np.array(A)
    B = np.array(B)

    AB = np.subtract(B, A)
    AP = np.subtract(P, A)
    sq = (AB[0] * AB[0] + AB[1] * AB[1])
    t = (AP[0] * AB[0] + AP[1] * AB[1]) / sq
    isMiddle = True
    if t < 0:
        t = 0
        isMiddle = False
    if t > 1:
        t = 1
        isMiddle = False

    return list(A + t * AB), isMiddle

def distance(a, b):
    return np.linalg.norm(a-b)

def indexOf(D, value):
    for i in range(1, len(D)+1):
        if D[i] == value:
            return i
    return -1

'''
Grow a simple RRT 
'''
def growSimpleRRT(points):
    newPoints = points.copy()
    adjListMap = {1: [2], 2: [1]}
    # Your code goes here
    for p in points.keys():
        if p == 1 or p == 2:
            continue
        min = sys.maxsize
        minIndex = -1
        P2Loc = []
        isExtraPoint = False
        Q1 = -1
        Q2 = -1
        for q1 in adjListMap.keys():
            for q2 in adjListMap[q1]:
                q1Loc = list(newPoints[q1])
                q2Loc = list(newPoints[q2])
                pLoc = list(points[p])

                if q1Loc == q2Loc or p == q2:
                    continue

                # find the closest point of edge (q1Loc, q2Loc) to node pLoc
                p2Loc, isExtraPoint2 = closestPoint(pLoc, q1Loc, q2Loc)
                # Find the closest node to the Point p
                if distance(np.array(pLoc), np.array(p2Loc)) < min:
                    min = distance(np.array(pLoc), np.array(p2Loc))
                    if isExtraPoint2:
                        minIndex = len(newPoints)+1
                    else:
                        minIndex = indexOf(newPoints, tuple(p2Loc))
                    P2Loc = p2Loc
                    isExtraPoint = isExtraPoint2
                    Q1 = q1
                    Q2 = q2
        if minIndex == -1:
            continue
        else:
            if isExtraPoint:
                P2 = len(newPoints)+1
                newPoints[P2] = tuple(P2Loc)
                if adjListMap.__contains__(Q1):
                    if Q2 in adjListMap[Q1]:
                        adjListMap[Q1].remove(Q2)
                if adjListMap.__contains__(Q2):
                    if Q1 in adjListMap[Q2]:
                        adjListMap[Q2].remove(Q1)
                if adjListMap.get(Q1) is None:
                    adjListMap[Q1] = []
                adjListMap[Q1].append(P2)
                if adjListMap.get(P2) is None:
                    adjListMap[P2] = []
                adjListMap[P2].append(Q1)
                adjListMap[P2].append(Q2)
                if adjListMap.get(Q2) is None:
                    adjListMap[Q2] = []
                adjListMap[Q2].append(P2)

            if adjListMap.get(minIndex) is None:
                adjListMap[minIndex] = []
            if adjListMap.get(p) is None:
                adjListMap[p] = []
            adjListMap[minIndex].append(p)
            adjListMap[p].append(minIndex)

    return newPoints, adjListMap

'''
Perform basic search 
'''
def basicSearch(tree, start, goal):
    queue = []
    queue.append([start])
    visited = []
    while queue:
        path = queue.pop(0)
        curr = path[-1]
        if curr == goal:
            queue.append(goal)
            return path
        if curr in visited:
            continue
        else:
            visited.append(curr)
        for adjacent in tree.get(curr, []):
            newPath = list(path)
            newPath.append(adjacent)
            queue.append(newPath)
    return []
    #raise RuntimeError("Goal is not in tree")

'''
Display the RRT and Path
'''
def displayRRTandPath(points, adjListMap, path, robotStart = None, robotGoal = None, obstacles = None):
    list_of_tuples = points.values()
    pointsArray = [list(elem) for elem in list_of_tuples]
    data = np.array(pointsArray)
    x = data.T[0]
    y = data.T[1]

    fig, ax = plt.subplots()
    ax.scatter(x, y)
    plt.xlim(-1, 11)
    plt.ylim(-1, 11)


    blacklines = []
    for key, value in adjListMap.items():
        for edgeVertex in value:
            blacklines.append([points[key], points[edgeVertex]])

    lc = mc.LineCollection(blacklines, colors=("black"), linewidths=2)
    ax.add_collection(lc)

    orangelines = []
    for i in range(1, len(path)):
        orangelines.append([points[path[i - 1]], points[path[i]]])

    lc2 = mc.LineCollection(orangelines, colors=("orange"), linewidths=2)
    ax.add_collection(lc2)

    if robotStart is not None:
        patch = createPolygonPatch(robotStart, 'green')
        ax.add_patch(patch)
    if robotGoal is not None:
        patch = createPolygonPatch(robotGoal, 'red')
        ax.add_patch(patch)
    if obstacles is not None:
        for p in range(0, len(obstacles)):
            patch = createPolygonPatch(obstacles[p], 'gray')
            ax.add_patch(patch)
    plt.show()

# helper function: counterclockwise
def ccw(A,B,C):
    val = (B[1] - A[1]) * (C[0] - B[0]) - (B[0] - A[0]) * (C[1] - B[1])
    if (val == 0):
        return 0 # colinear
    if val > 0:
        return 1 # clockwise
    else:
        return 2 # counterclockwise

def onSegment(A, B, C):
    if B[0] <= max(A[0], C[0]) and B[0] >= min(A[0], C[0]) and \
            B[1] <= max(A[1], C[1]) and B[1] >= min(A[1], C[1]):
        return True
    return False

    # Return true if line segments AB and CD intersect
def intersect(A,B,C,D):

    o1 = ccw(A,B,C)
    o2 = ccw(A,B,D)
    o3 = ccw(C,D,A)
    o4 = ccw(C,D,B)
                        # DO NOT CHANGE
    #general case
    if o1 != o2 and o3 != o4:
        return True

    #special case of colinearity
    if o1 == 0 and onSegment(A, C, B):
        return True
    if o2 == 0 and onSegment(A, D, B):
        return True
    if o3 == 0 and onSegment(C, A, D):
        return True
    if o4 == 0 and onSegment(C, B, D):
        return True

    return False

'''
Collision checking
'''
def isCollisionFree(robot, point, obstacles):

    robotCopy = robot.copy()
    #offset robot by point
    for p in range(0,len(robotCopy)):
        robotCopy[p] = list(np.add(np.array(robotCopy[p]), np.array(point)))

    #for each obstacle, check if robot is inside obstacle OR its edges intersect with it obstacle's edges
    for obstacle in obstacles:
        numIntersections = 0
        #check intersection between every edge of robot with every edge of obstacle
        for obsEdge in range(0,len(obstacle)):
            for robotEdge in range(0, len(robotCopy)):
                if intersect(obstacle[obsEdge-1], obstacle[obsEdge], robotCopy[robotEdge-1], robotCopy[robotEdge]):
                    return False
            # count number of intersections using ray casting
            if intersect(obstacle[obsEdge-1], obstacle[obsEdge], robotCopy[0],
                         list(np.add(np.array(robotCopy[0]), np.array([10.4729, 11.9837])))):
                numIntersections += 1
            # subtract when passing through obstacle corner because it will double count intersection
            #LIMITATION OF RAY CASTING: if line goes through point is on the edge of the polygon.
            # picked a very rare slope so the chance is very low
            if ccw(robotCopy[0], obstacle[obsEdge],
                         list(np.add(np.array(robotCopy[0]), np.array([10.4729, 11.9837])))) == 0:
                if onSegment(robotCopy[0], obstacle[obsEdge],
                             list(np.add(np.array(robotCopy[0]), np.array([10.4729, 11.9837])))):
                    numIntersections -= 1


        # check if robot is inside an obstacle
        if numIntersections % 2 == 1:
            return False

    return True

def generateRandomPoint():
    return np.random.uniform(low=0, high=10, size=(2))

def isFreePath(robot, obstacles, startPoint, endPoint):
    startRobot = robot.copy()
    endRobot = robot.copy()

    for p in range(0, len(startRobot)):
        startRobot[p] = list(np.add(np.array(startRobot[p]), np.array(startPoint)))
        endRobot[p] = list(np.add(np.array(endRobot[p]), np.array(endPoint)))

    for obstacle in obstacles:
        for obstEdge in range(0, len(obstacle)):
            for p in range(0, len(startRobot)):
                if intersect(obstacle[obstEdge - 1], obstacle[obstEdge], endRobot[p], startRobot[p]):
                    return False
    return True

'''
The full RRT algorithm
'''
def RRT(robot, obstacles, startPoint, goalPoint):

    points = {1: startPoint}
    adjListMap = {1: [2], 2: [1]}
    indexCounter = 3

    if not isCollisionFree(robot, startPoint, obstacles):
        raise RuntimeError("Start point is colliding; no path can exists")

    targetRobot = robot.copy()
    for p in range(0, len(targetRobot)):
        targetRobot[p] = list(np.add(np.array(targetRobot[p]), np.array(goalPoint)))

    tempPLoc = generateRandomPoint()
    while not isFreePath(robot, obstacles, startPoint, tempPLoc):
        tempPLoc = generateRandomPoint()
    points[2] = tuple(tempPLoc)

    while isCollisionFree(robot, points[indexCounter-1], [targetRobot]) and isCollisionFree(robot, points[indexCounter-2], [targetRobot]):
        tempPLoc = generateRandomPoint()
        while not isCollisionFree(robot, tempPLoc, obstacles):
            tempPLoc = generateRandomPoint()
        p = indexCounter
        points[p] = tuple(tempPLoc)
        min = sys.maxsize
        minIndex = -1
        P2Loc = []
        isExtraPoint = False
        Q1 = -1
        Q2 = -1
        for q1 in adjListMap.keys():
            for q2 in adjListMap[q1]:
                q1Loc = list(points[q1])
                q2Loc = list(points[q2])
                pLoc = list(points[p])

                if q1Loc == q2Loc or p == q2:
                    continue

                # find the closest point of edge (q1Loc, q2Loc) to node pLoc
                p2Loc, isExtraPoint2 = closestPoint(pLoc, q1Loc, q2Loc)
                # Find the closest node to the Point p
                if distance(np.array(pLoc), np.array(p2Loc)) < min:
                    min = distance(np.array(pLoc), np.array(p2Loc))
                    if isExtraPoint2:
                        minIndex = len(points)+1
                    else:
                        minIndex = indexOf(points, tuple(p2Loc))
                    P2Loc = p2Loc
                    isExtraPoint = isExtraPoint2
                    Q1 = q1
                    Q2 = q2

        if not isFreePath(robot, obstacles, tuple(P2Loc), points[p]):
            continue
        if minIndex == -1:
            continue
        else:
            if isExtraPoint:
                indexCounter+=1
                P2 = len(points)+1
                points[P2] = tuple(P2Loc)
                if adjListMap.__contains__(Q1):
                    if Q2 in adjListMap[Q1]:
                        adjListMap[Q1].remove(Q2)
                if adjListMap.__contains__(Q2):
                    if Q1 in adjListMap[Q2]:
                        adjListMap[Q2].remove(Q1)
                if adjListMap.get(Q1) is None:
                    adjListMap[Q1] = []
                adjListMap[Q1].append(P2)
                if adjListMap.get(P2) is None:
                    adjListMap[P2] = []
                adjListMap[P2].append(Q1)
                adjListMap[P2].append(Q2)
                if adjListMap.get(Q2) is None:
                    adjListMap[Q2] = []
                adjListMap[Q2].append(P2)

            if adjListMap.get(minIndex) is None:
                adjListMap[minIndex] = []
            if adjListMap.get(p) is None:
                adjListMap[p] = []
            adjListMap[minIndex].append(p)
            adjListMap[p].append(minIndex)
        indexCounter += 1

    if not isCollisionFree(robot, points[indexCounter - 1], [targetRobot]):
        points[indexCounter] = goalPoint
        if adjListMap.get(indexCounter) is None:
            adjListMap[indexCounter] = []
        adjListMap[indexCounter].append(indexCounter - 1)
        if adjListMap.get(indexCounter-1) is None:
            adjListMap[indexCounter-1] = []
        adjListMap[indexCounter-1].append(indexCounter)
        return points, adjListMap, basicSearch(adjListMap, 1, indexCounter - 1)
    else:
        points[indexCounter] = goalPoint
        if adjListMap.get(indexCounter) is None:
            adjListMap[indexCounter] = []
        adjListMap[indexCounter].append(indexCounter - 2)
        if adjListMap.get(indexCounter - 2) is None:
            adjListMap[indexCounter - 2] = []
        adjListMap[indexCounter - 2].append(indexCounter)
        return points, adjListMap, basicSearch(adjListMap, 1, indexCounter - 2)

def main(filename, x1, y1, x2, y2, display=''):
    # Read data and parse polygons
    lines = [line.rstrip('\n') for line in open(filename)]
    robot = []
    obstacles = []
    for line in range(0, len(lines)):
        xys = lines[line].split(';')
        polygon = []
        for p in range(0, len(xys)):
            xy = xys[p].split(',')
            polygon.append((float(xy[0]), float(xy[1])))
        if line == 0 :
            robot = polygon
        else:
            obstacles.append(polygon)

    # Print out the data
    print("Robot:")
    print(str(robot))
    print("Pologonal obstacles:")
    for p in range(0, len(obstacles)):
        print(str(obstacles[p]))
    print("")

    robotLocation = (0, 0)

    # Visualize
    if display == 'display':
        robotStart = [(x + x1, y + y1) for x, y in robot]
        robotGoal = [(x + x2, y + y2) for x, y in robot]
        drawProblem(robotStart, robotGoal, obstacles)

    # Example points for calling growSimpleRRT
    # You should expect many more points, e.g., 200-500
    points = dict()
    points[1] = (5, 5)
    points[2] = (7, 8.2)
    points[3] = (6.5, 5.2)
    points[4] = (0.3, 4)
    points[5] = (6, 3.7)
    points[6] = (9.7, 6.4)
    points[7] = (4.4, 2.8)
    points[8] = (9.1, 3.1)
    points[9] = (8.1, 6.5)
    points[10] = (0.7, 5.4)
    points[11] = (5.1, 3.9)
    points[12] = (2, 6)
    points[13] = (0.5, 6.7)
    points[14] = (8.3, 2.1)
    points[15] = (7.7, 6.3)
    points[16] = (7.9, 5)
    points[17] = (4.8, 6.1)
    points[18] = (3.2, 9.3)
    points[19] = (7.3, 5.8)
    points[20] = (9, 0.6)

    # points = {1: (5, 5), 2: (7, 8.2), 3: (6.5, 5.2), 4: (0.3, 4), 5: (6, 3.7), 6: (6, 8), 7: (4.4, 2.8), 8: (8.3, 2.1),
    # 9: (7.7, 6.3), 10: (9, 0.6)}

    # Printing the points
    print("")
    print("The input points are:")
    print(str(points))
    print("")
    
    points, adjListMap = growSimpleRRT(points)
    print("")
    print("The new points are:")
    print(str(points))
    print("")
    print("")
    print("The tree is:")
    print(str(adjListMap))
    print("")

    # Search for a solution  
    # change 1 and 20 as you want
    path = basicSearch(adjListMap, 1, 10)
    print("")
    print("The path is:")
    print(str(path))
    print("")

    # Your visualization code 
    if display == 'display':
        displayRRTandPath(points, adjListMap, path) 

    # Solve a real RRT problem
    points, adjListMap, path = RRT(robot, obstacles, (x1, y1), (x2, y2))
    
    # Your visualization code 
    if display == 'display':
         displayRRTandPath(points, adjListMap, path, robotStart, robotGoal, obstacles)


if __name__ == "__main__":
    # Retrive file name for input data
    if(len(sys.argv) < 6):
        print("Five arguments required: python rrt.py [env-file] [x1] [y1] [x2] [y2]")
        exit()
    
    filename = sys.argv[1]
    x1 = float(sys.argv[2])
    y1 = float(sys.argv[3])
    x2 = float(sys.argv[4])
    y2 = float(sys.argv[5])
    display = ''
    if(len(sys.argv) == 7):
        display = sys.argv[6]

    main(filename, x1, y1, x2, y2, display)