

from functools import total_ordering
import heapq
import math
import matplotlib.pyplot
import numpy
import numpy.linalg
import enum


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

    def __eq__(self, other):
        return numpy.array_equal(self.vec, other.vec) and self.angle == other.angle

    def __ne__(self, other):
        return not self.__eq__(other)

    def __hash__(self):
        return hash((self.vec[0], self.vec[1], self.angle))


@total_ordering
class nodeClass:

    def __init__(self, coords, parentNode, g, h, f):
        self.coords = coords
        self.parentNode = parentNode
        self.g = g
        self.h = h
        self.f = f

    def __str__(self):
        return "(" + str(self.coords.x) + ", " + str(self.coords.y) + ", " + str(math.degrees(self.coords.angle)) + ") | g=" + str(self.g) + ", h=" + str(self.h)

    def __eq__(self, other):
        return self.coords.__eq__(other.coords)

    def __ne__(self, other):
        return not self.__eq__(other)

    def __lt__(self, other):
        return self.f < other.f

    def __gt__(self, other):
        return self.f < other.f

    def __hash__(self):
        return hash((self.coords, self.parentNode, self.g, self.h, self.f))


Direction = enum.Enum('Direction', ['LEFT', 'STRAIGHT', 'RIGHT'])

leftCurveVec = coordClass.fromXYCoords(2.5 * numpy.sin(numpy.pi / 8.), 2.5 * (1 - numpy.cos(numpy.pi / 8.)), None)
rightCurveVec = coordClass.fromXYCoords(leftCurveVec.x, -leftCurveVec.y, None)

matplotlib.pyplot.ion()
fig = matplotlib.pyplot.figure()
ax = fig.add_subplot(111)
matplotlib.pyplot.gca().set_aspect('equal')


def dist(coords1, coords2):
    return numpy.linalg.norm(coords2.vec - coords1.vec)


def calc_h(currentCoords, goalCoords):
    diff = goalCoords.vec - currentCoords.vec
    retval = numpy.inner(diff, diff)

    angleDiff = goalCoords.angle - currentCoords.angle
    if angleDiff > numpy.pi: angleDiff = 2 * numpy.pi - angleDiff
    retval += numpy.power(angleDiff / (numpy.pi / 8.), 2) / numpy.power(0.8 + currentCoords.x, 4)
    if currentCoords.y < -2.5 and currentCoords.x < 2.5: retval += 100000000
    if currentCoords.x**2 + (currentCoords.y + 2.5)**2 < 6.25: retval += 100000000
    if currentCoords.y > 2.5 and currentCoords.x < 2.5: retval += 100000000
    if currentCoords.x**2 + (currentCoords.y - 2.5)**2 < 6.25: retval += 100000000
    return retval


def rotateCoords(coords, angle): # angle > 0 -> counterclockwise
    rot = numpy.array([[numpy.cos(angle), -numpy.sin(angle)], [numpy.sin(angle), numpy.cos(angle)]])
    return numpy.dot(rot, coords.vec)


def cleanAngle(angle):
    deg = numpy.round(angle / numpy.pi * 180, 3)
    if numpy.abs(deg) > 180:
        deg = numpy.mod(deg, 180 if deg < 0 else -180)
    if deg == -180.: deg = numpy.double(180)
    return deg / 180. * numpy.pi


def propagateCoords(d, coords):
    addVec = None
    newAngle = None
    if d == Direction.LEFT:
        addVec = rotateCoords(leftCurveVec, coords.angle)
        newAngle = cleanAngle(coords.angle + numpy.pi/8.)
    elif d == Direction.STRAIGHT:
        addVec = numpy.array([numpy.cos(coords.angle), numpy.sin(coords.angle)])
        newAngle = coords.angle
    elif d == Direction.RIGHT:
        addVec = rotateCoords(rightCurveVec, coords.angle)
        newAngle = cleanAngle(coords.angle - numpy.pi/8.)
    else:
        raise Exception("ups")
    return coordClass(coords.vec + addVec, newAngle)


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
    closedList = set()
    i = 0
    while openList:
        currentNode = heapq.heappop(openList)
        if i % 1 == 0:
            updatePlot(buildPath(currentNode))
        if i % 40 == 0:
            ax.clear()
        closedList.add(currentNode)
        if dist(currentNode.coords, end) < 0.3 and abs(currentNode.coords.angle - math.pi) < 0.01:
            print("found it")
            print(str(currentNode))
            for n in buildPath(currentNode):
                print("\t" + str(n))
        for d in list(Direction):
            if currentNode.g == 0 and d != Direction.LEFT:
                continue
            newCoords = propagateCoords(d, currentNode.coords)
            g = currentNode.g + 1
            h = calc_h(newCoords, end)
            node = nodeClass(newCoords, currentNode, g, h, g + h)
            if node in closedList:
                continue
            cont = False
            for olNode in openList:
                if olNode == node and node.g > olNode.g:
                    cont = True
                    break
            if cont: continue
            heapq.heappush(openList, node)
        i += 1


def main():
    start = coordClass.fromXYCoords(0., 0.5, numpy.double(0.))
    end = coordClass.fromXYCoords(0., 0., numpy.pi)
    run_a_star(start, end)
    input()

if __name__ == "__main__":
    main()
