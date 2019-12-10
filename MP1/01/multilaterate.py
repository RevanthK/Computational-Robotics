import sys
import numpy as np

def check(points, target, distance):
    if np.isnan(target).any():
        return False
    for it in range(0,(int)(points.size/3)):
        if np.linalg.norm(points[it] - target) != distance[it]:
            return False
    return True

def multilaterate(distances):

    p1 = np.array(distances[0][:3])
    p2 = np.array(distances[1][:3])
    p3 = np.array(distances[2][:3])
    p4 = np.array(distances[3][:3])

    r1 = distances[0][-1]
    r2 = distances[1][-1]
    r3 = distances[2][-1]
    r4 = distances[3][-1]

    # check if points are unique. if not, how many are repeated
    points = np.array([p1, p2, p3, p4])
    radii = np.array([r1, r2, r3, r4])
    np.sort(points, axis=None)
    if (points[2]==points[3]).all():
        points = np.delete(points, 3, 0)
        radii = np.delete(radii, 3)
    if (points[1]==points[2]).all():
        points = np.delete(points, 2, 0)
        radii = np.delete(radii, 2)
    if (points[0]==points[1]).all():
        points = np.delete(points, 1, 0)
        radii = np.delete(radii, 1)

    if points.size/3 == 4:
        eX = (p2 - p1)/np.linalg.norm(p2 - p1)
        i = np.dot(eX, (p3 - p1))
        eY = (p3 - p1 - (i * eX))/(np.linalg.norm(p3 - p1 - (i * eX)))
        eZ = np.cross(eX, eY)
        d0 = np.linalg.norm(p2 - p1)
        j = np.dot(eY, (p3 - p1))
        x = ((r1 ** 2) - (r2 ** 2) + (d0 ** 2))/(2 * d0)
        y = (((r1 ** 2) - (r3 ** 2) + (i ** 2) + (j ** 2)) / (2 * j)) - ((i/j) * (x))
        z1 = np.sqrt(r1 ** 2 - x ** 2 - y ** 2)
        z2 = np.sqrt(r1 ** 2 - x ** 2 - y ** 2) * (-1)
        answer1 = p1 + (x * eX) + (y * eY) + (z1 * eZ)
        answer2 = p1 + (x * eX) + (y * eY) + (z2 * eZ)
        d1 = np.linalg.norm(p4 - answer1)
        d2 = np.linalg.norm(p4 - answer2)
        if np.abs(r4 - d1) < np.abs(r4 - d2):
            ans = round(answer1[0], 6), round(answer1[1], 6), round(answer1[2], 6)
            if np.isnan(ans).any():
                raise ValueError('Improper input/unsolvable')
            else:
                return ans
        else:
            ans = round(answer2[0], 6), round(answer2[1], 6), round(answer2[2], 6)
            if np.isnan(ans).any():
                raise ValueError('Improper input/unsolvable')
            else:
                return ans

    elif points.size/3 == 3:
        p1 = points[0]
        p2 = points[1]
        p3 = points[2]

        eX = (p2 - p1)/np.linalg.norm(p2 - p1)
        i = np.dot(eX, (p3 - p1))
        eY = (p3 - p1 - (i * eX))/(np.linalg.norm(p3 - p1 - (i * eX)))
        eZ = np.cross(eX, eY)
        d0 = np.linalg.norm(p2 - p1)
        j = np.dot(eY, (p3 - p1))
        x = ((r1 ** 2) - (r2 ** 2) + (d0 ** 2))/(2 * d0)
        y = (((r1 ** 2) - (r3 ** 2) + (i ** 2) + (j ** 2)) / (2 * j)) - ((i/j) * (x))
        z1 = np.sqrt(r1 ** 2 - x ** 2 - y ** 2)
        z2 = np.sqrt(r1 ** 2 - x ** 2 - y ** 2) * (-1)
        answer1 = p1 + (x * eX) + (y * eY) + (z1 * eZ)
        answer2 = p1 + (x * eX) + (y * eY) + (z2 * eZ)
        ans = round(answer1[0], 6), round(answer1[1], 6), round(answer1[2], 6)
        ans2 = round(answer2[0], 6), round(answer2[1], 6), round(answer2[2], 6)

        if check(points, ans, radii):
            return ans
        if check(points, ans2, radii):
            return ans2
        raise ValueError('Improper input/unsolvable')
    elif points.size/3 == 2:
        if np.linalg.norm(points[0] - points[1]) == (radii[0] + radii[1]):
            vector = np.multiply((points[0] - points[1]), radii[0]/(radii[0]+radii[1]))
            ans = list(points[1]+vector)
            ans = [round(elem, 6) for elem in ans]
            return ans
        else:
            raise ValueError('Improper input/unsolvable: Only two unique points')
    elif points.size/3 == 1:
        # check if radial distance = 0, then target is at the point
        if r1 == 0:
            return list(p1)
        elif r2 == 0:
            return list(p2)
        elif r3 == 0:
            return list(p3)
        elif r4 == 0:
            return list(p4)
        else:
            raise ValueError('Improper input/unsolvable: Only one unique point')
    else:
        raise ValueError('Improper input/unsolvable')

if __name__ == "__main__":
    
    # Retrive file name for input data
    if(len(sys.argv) == 1):
        print("Please enter data file name.")
        exit()
    
    filename = sys.argv[1]

    # Read data
    lines = [line.rstrip('\n') for line in open(filename)]
    distances = []
    for line in range(0, len(lines)):
        distances.append(list(map(float, lines[line].split(' '))))

    # Print out the data
    print("The input four points and distances, in the format of [x, y, z, d], are:")
    for p in range(0, len(distances)):
        print(*distances[p])

    # Call the function and compute the location 
    location = multilaterate(distances)

    print("The location of the point is: " + str(location))
