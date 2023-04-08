
import collections
import math
import matplotlib.pyplot
import enum


coordClass = collections.namedtuple("coordClass", ["x", "y", "angle"])
nodeClass = collections.namedtuple("nodeClass", ["coords", "parentNode", "g", "h"])

Direction = enum.Enum('Direction', ['LEFT', 'STRAIGHT', 'RIGHT'])

rightCurveVec = coordClass(2.5 * math.sin(math.pi / 8.), 1 - math.cos(math.pi / 8.), None)
leftCurveVec = coordClass(rightCurveVec.x, -rightCurveVec.y, None)

matplotlib.pyplot.ion()
fig = matplotlib.pyplot.figure()
ax = fig.add_subplot(111)


def dist(coords1, coords2):
    return math.sqrt(math.pow((coords1.x - coords2.x), 2) + math.pow((coords1.y - coords2.y), 2))


def h(currentCoords, goalCoords):
    retval = math.pow((goalCoords.x - currentCoords.x), 2) + math.pow((goalCoords.y - currentCoords.y), 2)
    angleDiff = goalCoords.angle - currentCoords.angle
    if angleDiff > math.pi: angleDiff = 2 * math.pi - angleDiff
    retval += pow(angleDiff / (math.pi / 8.), 2) / pow(0.8 + currentCoords.x, 4) #der geht!!
    # retval += pow((goalCoords.angle - currentCoords.angle) / (math.pi / 8.), 2) / pow(retval, 2)
    # retval += pow(angleDiff / (math.pi / 8.), 2) / 5.
    # retval += pow((currentCoords.angle + goalCoords.angle) / (math.pi / 8.), 2) / pow(retval, 2)
    # # retval /= math.log(currentCoords.y + 1) if currentCoords.y > -1 else 1
    # if currentCoords.y < -1: retval *= 100000
    # if currentCoords.x < 0: retval *= 100000
    # if currentCoords.x < 2 and currentCoords.y > 2: retval *= 100000
    if currentCoords.y < -2.5 and currentCoords.x < 2.5: retval *= 100000
    if math.pow(currentCoords.x, 2) + math.pow(currentCoords.y + 2.5, 2) < 6.25: retval += 100000
    if currentCoords.y > 2.5 and currentCoords.x < 2.5: retval *= 100000
    if math.pow(currentCoords.x, 2) + math.pow(currentCoords.y - 2.5, 2) < 6.25: retval += 100000
    if currentCoords.y > 7 or currentCoords.y < -7: retval *= 100000

    # if currentCoords.x < 4 and abs(currentCoords.angle) > math.pi * 0.6: retval *= 100000
    # # if currentCoords.angle > math.pi * 0.5: retval *= 100000
    return retval


def f(node):
    return node.g + node.h


def coordsEqual(coords1, coords2):
    return coords1.x == coords2.x and coords1.y == coords2.y and coords1.angle == coords2.angle


def nodesEqual(node1, node2):
    return coordsEqual(node1.coords, node2.coords)


def rotateCoords(coords, angle): # angle > 0 -> counterclockwise
    return coordClass(coords.x * math.cos(angle) - coords.y * math.sin(angle), coords.x * math.sin(angle) - coords.y * math.cos(angle), None)


def cleanAngle(angle):
    deg = round(math.degrees(angle), 3)
    if abs(deg) > 180:
        deg = deg % (180 if deg < 0 else -180)
    if deg == -180: deg = 180
    return math.radians(deg)


def propagateCoords(d, coords):
    if d == Direction.LEFT:
        addVec = rotateCoords(leftCurveVec, coords.angle)
        return coordClass(coords.x + addVec.x, coords.y + addVec.y, cleanAngle(coords.angle + math.pi/8.))
    elif d == Direction.STRAIGHT:
        return coordClass(coords.x + math.cos(coords.angle), coords.y + math.sin(coords.angle), coords.angle)
    elif d == Direction.RIGHT:
        addVec = rotateCoords(rightCurveVec, coords.angle)
        return coordClass(coords.x + addVec.x, coords.y + addVec.y, cleanAngle(coords.angle - math.pi/8.))
    else:
        raise Exception("ups")


def nodeToStr(node):
    return "(" + str(node.coords.x) + ", " + str(node.coords.y) + ", " + str(math.degrees(node.coords.angle)) + ") | g=" + str(node.g) + ", h=" + str(node.h)


def buildPath(node, path = []):
    if node.parentNode is None:
        return list(reversed(path + [node]))
    return buildPath(node.parentNode, path + [ node ])


def updatePlot(path):
    if not path:
        return
    ax.plot([p.coords.x for p in path], [p.coords.y for p in path])
    ax.set_xlim([0, 10])
    ax.set_ylim([-7, 7])
    fig.canvas.draw()
    fig.canvas.flush_events()


def addNodePathPlot(node1, node2):
    ax.plot([node1.coords.x, node2.coords.x], [node1.coords.y, node2.coords.y])
    ax.set_xlim([0, 10])
    ax.set_ylim([-7, 7])
    fig.canvas.draw()
    fig.canvas.flush_events()


def run_a_star(start, end):
    startNode = nodeClass(start, None, 0, h(start, end))
    openList = [ startNode ]
    closedList = []
    i = 0
    while openList:
        # print("iteration " + str(i))
        # if i > 5: break
        currentNode = min(openList, key=f)
        if i % 1 == 0:
            updatePlot(buildPath(currentNode))
        if i % 50 == 0:
            ax.clear()
        # print("at node " + nodeToStr(currentNode))
        openList.remove(currentNode)
        closedList.append(currentNode)
        # if currentNode.parentNode is not None:
        #     addNodePathPlot(currentNode.parentNode, currentNode)
        if dist(currentNode.coords, end) < 0.3 and abs(currentNode.coords.angle - math.pi) < 0.01:
        # if coordsEqual(currentNode.coords, end):
            print("found it")
            print(nodeToStr(currentNode))
            for n in buildPath(currentNode):
                print("\t" + nodeToStr(n))
        for d in list(Direction):
            # if currentNode.g > 15 and d == Direction.LEFT:
            #     continue
            if currentNode.g == 0 and d != Direction.LEFT:
                continue
            newCoords = propagateCoords(d, currentNode.coords)
            node = nodeClass(newCoords, currentNode, currentNode.g + 1, h(newCoords, end))
            if [ x for x in closedList if coordsEqual(x.coords, newCoords)]:
                continue
            filteredOpenList = [ x for x in openList if coordsEqual(x.coords, newCoords) and node.g > x.g ]
            if filteredOpenList:
                continue
            # print("\tappending node " + nodeToStr(node))
            # if f(node) < f(currentNode):
            #     addNodePathPlot(currentNode, node)
            openList.append(node)
        i += 1


def main():
    start = coordClass(0., 0.5, 0.)
    end = coordClass(0., 0., math.pi)
    # end = coordClass(4., 0., math.pi)
    run_a_star(start, end)


if __name__ == "__main__":
    main()
