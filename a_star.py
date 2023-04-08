
import collections
import math
import matplotlib.pyplot
import numpy
import numpy.linalg
import enum


#coordClass = collections.namedtuple("coordClass", ["x", "y", "angle"])
class coordClass:


    def __init__(self, vec, angle):
        self.vec = vec
        self.angle = angle

    @classmethod
    def fromXYCoords(self, x, y, angle):
        vec = numpy.array([x, y])
        return coordClass(vec, angle)

    @property
    def x(self):
        return self.vec[0]

    @x.setter
    def x(self, val):
        self.vec[0] = val

    @property
    def y(self):
        return self.vec[1]

    @y.setter
    def y(self, val):
        self.vec[1] = val


nodeClass = collections.namedtuple("nodeClass", ["coords", "parentNode", "g", "h", "f"])

Direction = enum.Enum('Direction', ['LEFT', 'STRAIGHT', 'RIGHT'])

leftCurveVec = coordClass.fromXYCoords(2.5 * numpy.sin(numpy.pi / 8.), 2.5 * (1 - numpy.cos(numpy.pi / 8.)), None)
rightCurveVec = coordClass.fromXYCoords(leftCurveVec.x, -leftCurveVec.y, None)

matplotlib.pyplot.ion()
fig = matplotlib.pyplot.figure()
ax = fig.add_subplot(111)


def dist(coords1, coords2):
    return numpy.linalg.norm(coords2.vec - coords1.vec)
    #return math.sqrt(math.pow((coords1.x - coords2.x), 2) + math.pow((coords1.y - coords2.y), 2))


def calc_h(currentCoords, goalCoords):
    # return dist(currentCoords, goalCoords)
    diff = goalCoords.vec - currentCoords.vec
    return numpy.inner(diff, diff)


# def f(node):
#     return node.g + node.h


def coordsEqual(coords1, coords2):
    return numpy.array_equal(coords1.vec, coords2.vec)


def nodesEqual(node1, node2):
    return coordsEqual(node1.coords, node2.coords)


def rotateCoords(coords, angle): # angle > 0 -> counterclockwise
    rot = numpy.array([[numpy.cos(angle), -numpy.sin(angle)], [numpy.sin(angle), numpy.cos(angle)]])
    return numpy.dot(rot, coords.vec)
    # newVec = numpy.dot(rot, coords.vec)
    # return coordClass(newVec, None)
    # return coordClass(coords.x * math.cos(angle) - coords.y * math.sin(angle), coords.x * math.sin(angle) + coords.y * math.cos(angle), None)


def cleanAngle(angle):
    deg = numpy.round(angle / numpy.pi * 180, 3)
    if numpy.abs(deg) > 180:
        deg = numpy.mod(deg, 180 if deg < 0 else -180)
    if deg == -180.: deg = numpy.double(180)
    return deg / 180. * numpy.pi
    # deg = round(math.degrees(angle), 3)
    # if abs(deg) > 180:
    #     deg = deg % (180 if deg < 0 else -180)
    # if deg == -180: deg = 180
    # return math.radians(deg)


def propagateCoords(d, coords):
    addVec = None
    newAngle = None
    if d == Direction.LEFT:
        addVec = rotateCoords(leftCurveVec, coords.angle)
        newAngle = cleanAngle(coords.angle + numpy.pi/8.)
        # return coordClass(coords.vec + addVec, cleanAngle(coords.angle + numpy.pi/8.))
    elif d == Direction.STRAIGHT:
        addVec = numpy.array([numpy.cos(coords.angle), numpy.sin(coords.angle)])
        newAngle = coords.angle
        # return coordClass(coords.x + numpy.cos(coords.angle), coords.y + numpy.sin(coords.angle), coords.angle)
    elif d == Direction.RIGHT:
        addVec = rotateCoords(rightCurveVec, coords.angle)
        newAngle = cleanAngle(coords.angle - numpy.pi/8.)
        # return coordClass(coords.vec + addVec, cleanAngle(coords.angle - numpy.pi/8.))
    else:
        raise Exception("ups")
    return coordClass(coords.vec + addVec, newAngle)


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
    startNode = nodeClass(start, None, 0, calc_h(start, end), calc_h(start, end))
    openList = [ startNode ]
    closedList = []
    i = 0
    while openList:
        currentNode = min(openList, key=lambda x: x.f)
        if i % 1 == 0:
            updatePlot(buildPath(currentNode))
        if i % 50 == 0:
            ax.clear()
        openList.remove(currentNode)
        closedList.append(currentNode)
        if dist(currentNode.coords, end) < 0.3 and abs(currentNode.coords.angle - math.pi) < 0.01:
            print("found it")
            print(nodeToStr(currentNode))
            for n in buildPath(currentNode):
                print("\t" + nodeToStr(n))
        for d in list(Direction):
            newCoords = propagateCoords(d, currentNode.coords)
            g = currentNode.g + 1
            h = calc_h(newCoords, end)
            node = nodeClass(newCoords, currentNode, g, h, g + h)
            if [ x for x in closedList if coordsEqual(x.coords, newCoords)]:
                continue
            filteredOpenList = [ x for x in openList if coordsEqual(x.coords, newCoords) and node.g > x.g ]
            if filteredOpenList:
                continue
            openList.append(node)
        i += 1


def main():
    start = coordClass.fromXYCoords(0., 0.5, numpy.double(0.))
    end = coordClass.fromXYCoords(0., 0., numpy.pi)
    run_a_star(start, end)
    input()

if __name__ == "__main__":
    main()
