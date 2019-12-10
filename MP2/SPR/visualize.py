import sys

import matplotlib.pyplot as plt
from matplotlib.path import Path
import matplotlib.patches as patches
import spr

'''
Set up matplotlib to create a plot with an empty square
'''
def setupPlot():
    fig = plt.figure(num=None, dpi=120, facecolor='w', edgecolor='k')
    ax = fig.subplots()
    ax.set_axisbelow(True)
    ax.grid(which='minor', linestyle=':', alpha=0.2)
    ax.grid(which='major', linestyle=':', alpha=0.5)
    return fig, ax

'''
Make a patch for a single pology 
'''
def createPolygonPatch(polygon):
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
    patch = patches.PathPatch(path, facecolor='gray', lw=1)

    return patch
    
'''
Make a patch for the robot
'''
def createPolygonPatchForRobot(polygon):
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
    patch = patches.PathPatch(path, facecolor='gray', lw=1)

    return patch
    

'''
Render polygon obstacles  
'''
def drawPolygons(polygons, fig, ax):
    for p in range(0, len(polygons)):
        patch = createPolygonPatch(polygons[p])
        ax.add_patch(patch)

def drawLine(x1, y1, x2, y2, c):
    plt.plot([x1, x2], [y1, y2], color=c, linestyle='-', linewidth=3)


if __name__ == "__main__":

    # Retrive file name for input data
    if (len(sys.argv) < 6):
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

    # Setup
    fig, ax = setupPlot()

    # Draw the polygons
    drawPolygons(polygons, fig, ax)

    # Compute reflex vertices
    reflexVertices = spr.findReflexiveVertices(polygons)
    # Compute the roadmap
    vertexMap, adjListMap = spr.computeSPRoadmap(polygons, reflexVertices)
    # Update roadmap
    start, goal, updatedALMap = spr.updateRoadmap(polygons, vertexMap, adjListMap, x1, y1, x2, y2)
    # Search for a solution
    path, length = spr.uniformCostSearch(updatedALMap, start, goal)

    # for convenience, added start and end to vertexMap
    vertexMap[-1] = [x2, y2]
    vertexMap[0] = [x1, y1]

    # Draw SP Road map in Green
    for key in updatedALMap.keys():
        for val in updatedALMap[key]:
            p1 = vertexMap[key]
            p2 = vertexMap[val[0]]
            drawLine(p1[0], p1[1], p2[0], p2[1], "g")

    # Draw Shortest Path in Red
    for i in range(1,len(path)):
        p1 = vertexMap[path[i-1]]
        p2 = vertexMap[path[i]]
        drawLine(p1[0], p1[1], p2[0], p2[1], "r")

    # ===== delete the following line before you make some changes =====
    ax.plot()
    # ======= delete the above line before you make some changes =======
    
    plt.show()