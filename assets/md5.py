import numpy as np
import parse
import struct

class Vec2:
    def __init__(self, x=0, y=0):
        self.x = x
        self.y = y

    def pack(self):
        return struct.pack('<ff', self.x, self.y)

    def __str__(self):
        return '{:.6f} {:.6f}'.format(self.x, self.y)

class Vec3:
    def __init__(self, x=0, y=0, z=0):
        self.x = x
        self.y = y
        self.z = z

    def pack(self):
        return struct.pack('<fff', self.x, self.z, -self.y)

    def trans4x4(self):
        m = np.identity(4)
        m[0][3] = self.x
        m[1][3] = self.y
        m[2][3] = self.z

        return m

    def __str__(self):
        return '{:.6f} {:.6f} {:.6f}'.format(self.x, self.y, self.z)

class Quaternion:
    def __init__(self, x=0, y=0, z=0):
        self.x = x
        self.y = y
        self.z = z

        self.computeW()

    def pack(self):
        return struct.pack('<ffff', self.x, self.z, -self.y, self.w)

    def __str__(self):
        return '{:.6f} {:.6f} {:.6f} {:.6f}'.format(self.x, self.y, self.z, self.w)

    def computeW(self):
        t = 1.0 - (self.x**2) - (self.y**2) - (self.z**2)
        if (t < 0.0):
            self.w = 0.0
        else:
            self.w = -np.sqrt(t)

    def rot4x4(self):
        q0 = self.w
        q1 = self.x
        q2 = self.y
        q3 = self.z
        # First row of the rotation matrix
        r00 = 2 * (q0 * q0 + q1 * q1) - 1
        r01 = 2 * (q1 * q2 - q0 * q3)
        r02 = 2 * (q1 * q3 + q0 * q2)

        # Second row of the rotation matrix
        r10 = 2 * (q1 * q2 + q0 * q3)
        r11 = 2 * (q0 * q0 + q2 * q2) - 1
        r12 = 2 * (q2 * q3 - q0 * q1)

        # Third row of the rotation matrix
        r20 = 2 * (q1 * q3 - q0 * q2)
        r21 = 2 * (q2 * q3 + q0 * q1)
        r22 = 2 * (q0 * q0 + q3 * q3) - 1

        # 3x3 rotation matrix
        m = np.array([[r00, r01, r02, 0],
                      [r10, r11, r12, 0],
                      [r20, r21, r22, 0],
                      [  0,   0,   0, 1]])
        return m

    def rotatePoint(self, pos):
        inv = Quaternion()

        inv.x = -self.x
        inv.y = -self.y
        inv.z = -self.z
        inv.w =  self.w

        inv.normalize()

        tmp = self.multVec(pos)
        final = tmp.multQuat(inv)

        return Vec3(final.x, final.y, final.z)

    def normalize(self):
        mag = np.sqrt((self.x * self.x) + (self.y * self.y) + (self.z * self.z) + (self.w * self.w))
        if mag > 0.0:
            oneOverMag = 1.0 / mag

            self.w *= oneOverMag
            self.x *= oneOverMag
            self.y *= oneOverMag
            self.z *= oneOverMag

    def multQuat(self, qb):
        out = Quaternion()

        out.w = (self.w * qb.w) - (self.x * qb.x) - (self.y * qb.y) - (self.z * qb.z)
        out.x = (self.x * qb.w) + (self.w * qb.x) + (self.y * qb.z) - (self.z * qb.y)
        out.y = (self.y * qb.w) + (self.w * qb.y) + (self.z * qb.x) - (self.x * qb.z)
        out.z = (self.z * qb.w) + (self.w * qb.z) + (self.x * qb.y) - (self.y * qb.x)

        return out

    def multVec(self, v):
        out = Quaternion()

        out.w = - (self.x * v.x) - (self.y * v.y) - (self.z * v.z)
        out.x =   (self.w * v.x) + (self.y * v.z) - (self.z * v.y)
        out.y =   (self.w * v.y) + (self.z * v.x) - (self.x * v.z)
        out.z =   (self.w * v.z) + (self.x * v.y) - (self.y * v.x)

        return out

    def dotProduct(self, qb):
        pass

    def slerp(self, qb, t):
        pass

class MD5Joint:
    def __init__(self, name='', parent=0, pos=Vec3(), orient=Quaternion()):
        self.name = name
        self.parent = parent
        self.pos = pos # Vec3
        self.orient = orient # Quaternion

    def pack(self):
        return struct.pack('<i', self.parent) + self.pos.pack() + self.orient.pack()

    def invBindMat4x4(self, joints):
        trans = self.pos.trans4x4()
        rot = self.orient.rot4x4()
        t = trans.dot(rot)

        parent = self.parent

        while parent >= 0:
            trans = joints[parent].pos.trans4x4()
            rot = joints[parent].orient.rot4x4()
            pt = trans.dot(rot)
            t = pt.dot(t)

            parent = joints[parent].parent

        return np.linalg.inv(t)

    def __str__(self):
        return '"{}": parent: {:d} pos: ({}, {}, {}) orient: ({}, {}, {}, {})'.format(
                self.name, self.parent,
                self.pos.x, self.pos.y, self.pos.z,
                self.orient.x, self.orient.y, self.orient.z, self.orient.w)

class MD5Vertex:
    def __init__(self, st=Vec2(), sw=0, cw=0):
        self.st = st # Vec2
        self.startWeight = sw
        self.countWeight = cw

    def pack(self):
        return self.st.pack() + struct.pack('<ii', self.startWeight, self.countWeight)

    def __str__(self):
        return 'ST: ({}, {}) weights: {} ({})'.format(
            self.st.x, self.st.y,
            self.startWeight, self.countWeight)

class MD5Triangle:
    def __init__(self, vertIndices=[]):
        self.vertIndices = vertIndices

    def pack(self):
        return struct.pack('<iii', self.vertIndices[0], self.vertIndices[2], self.vertIndices[1])

    def __str__(self):
        return '{} {} {}'.format(
            self.vertIndices[0], self.vertIndices[1], self.vertIndices[2])

class MD5Weight:
    def __init__(self, jointIndex=0, bias=0, pos=Vec3()):
        self.jointIndex = jointIndex
        self.bias = bias
        self.pos = pos

    def pack(self):
        return struct.pack('<if', self.jointIndex, self.bias) + self.pos.pack()

    def __str__(self):
        return '{} {} ({}, {}, {})'.format(
            self.jointIndex, self.bias,
            self.pos.x, self.pos.y, self.pos.z)

class MD5Mesh:
    def __init__(self):
        self.shader = ""

        self.numVerts = 0
        self.verts = []

        self.numTris = 0
        self.tris = []

        self.numWeights = 0
        self.weights = []

    def pack(self):
        headerData = struct.pack('<iiii', self.numVerts, self.numTris, self.numWeights, len(self.shader))
        print(self.shader, len(self.shader))

        vertsData = bytearray()
        trisData = bytearray()
        weightsData = bytearray()

        for v in self.verts:
            vertsData += v.pack()
        for t in self.tris:
            trisData += t.pack()
        for w in self.weights:
            weightsData += w.pack()

        return headerData + vertsData + trisData + weightsData + bytes(self.shader, 'ascii')

class MD5Model:
    def __init__(self):
        self.numJoints = 0 # TODO raise if numJoints != len(joints)
        self.numMeshes = 0 # TODO raise if numMesh != len(meshes)
        self.joints = []
        self.meshes = []
        # TODO: bounding boxes ?

    def from_file(self, filename):
        with open(filename) as rawfile:
            while True:
                line = rawfile.readline()
                if not line:
                    break

                if line.startswith('MD5Version'):
                    version = parse.search('MD5Version {:d}', line)[0]
                    if version != 10:
                        raise Exception('wrong md5 version')
                elif line.startswith('numJoints'):
                    self.numJoints = parse.search('numJoints {:d}', line)[0]

                elif line.startswith('numMeshes'):
                    self.numMeshes = parse.search('numMeshes {:d}', line)[0]

                elif line.startswith('joints {'):
                    for i in range(self.numJoints):
                        jointLine = rawfile.readline().lstrip().replace('\t', ' ')
                        jointData = parse.search('"{name}" {parent:d} ( {px:g} {py:g} {pz:g} ) ( {ox:g} {oy:g} {oz:g} )', jointLine)
                        pos = Vec3(jointData['px'], jointData['py'], jointData['pz'])
                        orient = Quaternion(jointData['ox'], jointData['oy'], jointData['oz'])
                        joint = MD5Joint(jointData['name'], jointData['parent'], pos, orient)
                        self.joints.append(joint)

                elif line.startswith('mesh {'):
                    mesh = MD5Mesh()
                    while True:
                        meshLine = rawfile.readline().lstrip()
                        if meshLine == "}\n":
                            self.meshes.append(mesh)
                            break

                        if meshLine.startswith('shader'):
                            # TODO save shader information (use it as texture filename)
                            data = parse.search('shader "{shader:S}"', meshLine)
                            mesh.shader = data['shader']


                        # vertices
                        elif meshLine.startswith('numverts'):
                            mesh.numVerts = parse.search('numverts {:d}', meshLine)[0]
                            mesh.verts = [None] * mesh.numVerts
                        elif meshLine.startswith('vert'):
                            vertData = parse.search('vert {idx:d} ( {s:g} {t:g} ) {sw:d} {cw:d}', meshLine)
                            st = Vec2(vertData['s'], vertData['t'])
                            mesh.verts[vertData['idx']] = MD5Vertex(
                                st,
                                vertData['sw'],
                                vertData['cw'])

                        # triangle
                        elif meshLine.startswith('numtris'):
                            mesh.numTris = parse.search('numtris {:d}', meshLine)[0]
                            mesh.tris = [None] * mesh.numTris
                        elif meshLine.startswith('tri'):
                            triData = parse.search('tri {idx:d} {:d} {:d} {:d}', meshLine)
                            mesh.tris[triData['idx']] = MD5Triangle(
                                [triData[0], triData[1], triData[2]])

                        # weights
                        elif meshLine.startswith('numweights'):
                            mesh.numWeights = parse.search('numweights {:d}', meshLine)[0]
                            mesh.weights = [None] * mesh.numWeights
                        elif meshLine.startswith('weight'):
                            weightData = parse.search('weight {idx:d} {joint:d} {bias:g} ( {px:g} {py:g} {pz:g} )', meshLine)
                            pos = Vec3(weightData['px'], weightData['py'], weightData['pz'])
                            mesh.weights[weightData['idx']] = MD5Weight(
                                weightData['joint'],
                                weightData['bias'],
                                pos)
        return self

    def export(self):
        headerData = struct.pack('<ii', self.numJoints, self.numMeshes)

        jointsData = bytearray()
        for j in self.joints:
            jointsData += j.pack()

        meshesData = bytearray()
        for m in self.meshes:
            meshesData += m.pack()

        with open('md5model.bin', 'wb') as f:
            f.write(headerData + jointsData + meshesData)

class JointInfo:
    def __init__(self, name='', parent=0, flags=0, startIndex=0):
        self.name = name
        self.parent = parent
        self.flags = flags
        self.startIndex = startIndex

    def __str__(self):
        return "{} {} {} {}".format(self.name, self.parent, self.flags, self.startIndex)

class BaseFrameJoint:
    def __init__(self, pos=None, orient=None):
        self.pos = pos # Vec3
        self.orient = orient # Quaternion

    def __str__(self):
        return "({}) ({})".format(self.pos, self.orient)

class MD5Anim:
    def __init__(self):
        self.numFrames = 0
        self.numJoints = 0
        self.frameRate = 0
        self.numAnimatedComponents = 0

        self.frameJoints = []
        self.bbox = []

        self.jointInfos = [] # JointInfo
        self.baseFrame = [] # BaseFrameJoint

    def from_file(self, filename):
        with open(filename) as rawfile:
            self.animationName = 'Take1' # filename.rsplit('.', 1)[0]

            while True:
                line = rawfile.readline()
                if not line:
                    break

                if line.startswith('MD5Version'):
                    version = parse.search('MD5Version {:d}', line)[0]
                    if version != 10:
                        raise Exception('wrong md5 version')

                elif line.startswith('commandline'):
                    pass

                elif line.startswith('numFrames'):
                    self.numFrames = parse.search('numFrames {:d}', line)[0]
                    self.frameJoints = [None] * self.numFrames
                    self.bbox = [None] * self.numFrames

                elif line.startswith('numJoints'):
                    self.numJoints = parse.search('numJoints {:d}', line)[0]
                    for i in range(self.numFrames):
                        self.frameJoints[i] = [None] * self.numJoints

                    self.jointInfos = [None] * self.numJoints
                    self.baseFrame = [None] * self.numJoints

                elif line.startswith('frameRate'):
                    self.frameRate = parse.search('frameRate {:d}', line)[0]

                elif line.startswith('numAnimatedComponents'):
                    self.numAnimatedComponents = parse.search('numAnimatedComponents {:d}', line)[0]


                # hierarchy
                elif line.startswith('hierarchy {'):
                    for i in range(self.numJoints):
                        jointInfosLine = rawfile.readline().lstrip().replace('\t', ' ')
                        data = parse.search('"{name}" {parent:d} {flags:d} {startIndex:d}', jointInfosLine)
                        self.jointInfos[i] = JointInfo(data['name'], data['parent'], data['flags'], data['startIndex'])
                # bounds
                elif line.startswith('bounds {'):
                    pass

                # baseframe
                elif line.startswith('baseframe {'):
                    for i in range(self.numJoints):
                        baseFrameLine = rawfile.readline().lstrip().replace('\t', ' ')
                        data = parse.search('( {px:g} {py:g} {pz:g} ) ( {ox:g} {oy:g} {oz:g} )', baseFrameLine)
                        pos = Vec3(data['px'], data['py'], data['pz'])
                        orient = Quaternion(data['ox'], data['oy'], data['oz'])
                        self.baseFrame[i] = BaseFrameJoint(pos, orient)
                # frames
                elif line.startswith('frame'):
                    frameId = parse.search('frame {id:d}', line)['id']
                    frameLines = []
                    while True:
                        frameLine = rawfile.readline().lstrip()
                        if frameLine == "}\n":
                            break
                        frameLines.append(frameLine.rstrip())
                    animFrameData = [float(x) for x in ' '.join(frameLines).split(' ')]
                    if len(animFrameData) != self.numAnimatedComponents:
                        raise Exception('wrong numAnimatedComponents for frame {}'.format(frameId))
                    self.buildFrameSkeleton(frameId, animFrameData)

    def buildFrameSkeleton(self, frameId, animFrameData):
        for i in range(self.numJoints):
            baseJoint = self.baseFrame[i]
            animatedPos = baseJoint.pos
            animatedOrient = baseJoint.orient
            j = 0

            if self.jointInfos[i].flags & 1: # Tx
                animatedPos.x = animFrameData[self.jointInfos[i].startIndex + j]
                j += 1
            if self.jointInfos[i].flags & 2: # Ty
                animatedPos.y = animFrameData[self.jointInfos[i].startIndex + j]
                j += 1
            if self.jointInfos[i].flags & 4: # Tz
                animatedPos.z = animFrameData[self.jointInfos[i].startIndex + j]
                j += 1
            if self.jointInfos[i].flags & 8:  # Qx
                animatedOrient.x = animFrameData[self.jointInfos[i].startIndex + j]
                j += 1
            if self.jointInfos[i].flags & 16: # Qy
                animatedOrient.y = animFrameData[self.jointInfos[i].startIndex + j]
                j += 1
            if self.jointInfos[i].flags & 32: # Qz
                animatedOrient.z = animFrameData[self.jointInfos[i].startIndex + j]
                j += 1

            animatedOrient.computeW()

            parent = self.jointInfos[i].parent
            thisJoint = MD5Joint()
            thisJoint.parent = parent
            thisJoint.name = self.jointInfos[i].name

            if thisJoint.parent < 0:
                thisJoint.pos = animatedPos
                thisJoint.orient = animatedOrient
            else:
                parentJoint = self.frameJoints[frameId][parent]
                rpos = parentJoint.orient.rotatePoint(animatedPos)
                thisJoint.pos = Vec3()
                thisJoint.pos.x = rpos.x + parentJoint.pos.x
                thisJoint.pos.y = rpos.y + parentJoint.pos.y
                thisJoint.pos.z = rpos.z + parentJoint.pos.z

                thisJoint.orient = parentJoint.orient.multQuat(animatedOrient)
                thisJoint.orient.normalize()

            self.frameJoints[frameId][i] = thisJoint

    def export(self):
        headerData = struct.pack('<iii', self.numFrames, self.numJoints, self.frameRate)

        jointsData = bytearray()

        for f in range(self.numFrames):
            for j in self.frameJoints[f]:
                print(j)
                jointsData += j.pack()
            print()

        with open('md5anim.bin', 'wb') as f:
            f.write(headerData + jointsData)


if __name__ == '__main__':
    model = MD5Model()
    model.from_file('cubeguy.md5mesh')
    model.export()

    anim = MD5Anim()
    anim.from_file('running.md5anim')
    anim.export()

