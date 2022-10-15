import numpy as np
import parse
import struct

class Vec2:
    def __init__(self, x=0, y=0):
        self.x = x
        self.y = y

    def __str__(self):
        return '{:.6f} {:.6f}'.format(self.x, self.y)

class Vec3:
    def __init__(self, x=0, y=0, z=0):
        self.x = x
        self.y = y
        self.z = z

    def __str__(self):
        return '{:.6f} {:.6f} {:.6f}'.format(self.x, self.y, self.z)

class Quaternion:
    def __init__(self, x=0, y=0, z=0):
        self.x = x
        self.y = y
        self.z = z

        t = 1.0 - (x*x) - (y*y) - (z*z)
        if (t < 0.0):
            self.w = 0.0
        else:
            self.w = -np.sqrt(t)

    def __str__(self):
        return '{:.6f} {:.6f} {:.6f} {:.6f}'.format(self.x, self.y, self.z, self.w)

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

class Joint:
    def __init__(self, name='', parent=0, pos=None, orient=None):
        self.name = name
        self.parent = parent
        self.pos = pos # Vec3
        self.orient = orient # Quaternion

    def __str__(self):
        return '"{}": parent: {:d} pos: ({}, {}, {}) orient: ({}, {}, {}, {})'.format(
                self.name, self.parent,
                self.pos.x, self.pos.y, self.pos.z,
                self.orient.x, self.orient.y, self.orient.z, self.orient.w)

class Vertex:
    def __init__(self, st=None, sw=0, cw=0):
        self.st = st # Vec2
        self.startWeight = sw
        self.countWeight = cw

    def __str__(self):
        return 'ST: ({}, {}) weights: {} ({})'.format(
            self.st.x, self.st.y,
            self.startWeight, self.countWeight)

class Triangle:
    def __init__(self, vertIndices=[]):
        self.vertIndices = vertIndices

    def __str__(self):
        return '{} {} {}'.format(
            self.vertIndices[0], self.vertIndices[1], self.vertIndices[2])

    def pack(self):
        return struct.pack('<iii', self.vertIndices[0], self.vertIndices[1], self.vertIndices[2])

class Weight:
    def __init__(self, jointIndex=0, bias=0, pos=None):
        self.jointIndex = jointIndex
        self.bias = bias
        self.pos = pos

    def __str__(self):
        return '{} {} ({}, {}, {})'.format(
            self.jointIndex, self.bias,
            self.pos.x, self.pos.y, self.pos.z)

class Mesh:
    def __init__(self):
        self.shader = ""

        self.numVerts = 0
        self.vertices = []

        self.numTris = 0
        self.tris = []

        self.numWeights = 0
        self.weights = []

class SkinnedVertex:
    def __init__(self, pos=Vec3(), st=Vec2(), bi=[], bw=[]):
        self.pos = pos
        self.uv = st
        self.boneIndices = bi
        self.boneWeights = bw

    def pack(self):
        posData = struct.pack('<fff', self.pos.x, self.pos.z, self.pos.y)
        uvData = struct.pack('<ff', self.uv.x, self.uv.y)
        return posData + uvData

    def packSkinned(self):
        boneIndicesData = bytearray()
        boneWeightsData = bytearray()
        for i in self.boneIndices:
            boneIndicesData += struct.pack('<i', i)
        for f in self.boneWeights:
            boneWeightsData += struct.pack('<f', f)
        return self.pack() + boneIndicesData + boneWeightsData


class MD5Model:
    def __init__(self):
        self.numJoints = 0
        self.numMeshes = 0
        self.joints = []
        self.meshes = []

    def from_file(self, filename):
        reading_joints = False
        reading_mesh = False

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
                        joint = Joint(jointData['name'], jointData['parent'], pos, orient)
                        self.joints.append(joint)

                elif line.startswith('mesh {'):
                    mesh = Mesh()
                    while True:
                        meshLine = rawfile.readline().lstrip()
                        if meshLine == "}\n":
                            self.meshes.append(mesh)
                            break

                        if meshLine.startswith('shader'):
                            # TODO save shader information (use it as texture filename)
                            pass

                        # vertices
                        elif meshLine.startswith('numverts'):
                            mesh.numVerts = parse.search('numverts {:d}', meshLine)[0]
                            mesh.vertices = [None] * mesh.numVerts
                        elif meshLine.startswith('vert'):
                            vertData = parse.search('vert {idx:d} ( {s:g} {t:g} ) {sw:d} {cw:d}', meshLine)
                            st = Vec2(vertData['s'], vertData['t'])
                            mesh.vertices[vertData['idx']] = Vertex(
                                st,
                                vertData['sw'],
                                vertData['cw'])

                        # triangle
                        elif meshLine.startswith('numtris'):
                            mesh.numTris = parse.search('numtris {:d}', meshLine)[0]
                            mesh.tris = [None] * mesh.numTris
                        elif meshLine.startswith('tri'):
                            triData = parse.search('tri {idx:d} {:d} {:d} {:d}', meshLine)
                            mesh.tris[triData['idx']] = Triangle(
                                [triData[0], triData[1], triData[2]])

                        # weights
                        elif meshLine.startswith('numweights'):
                            mesh.numWeights = parse.search('numweights {:d}', meshLine)[0]
                            mesh.weights = [None] * mesh.numWeights
                        elif meshLine.startswith('weight'):
                            weightData = parse.search('weight {idx:d} {joint:d} {bias:g} ( {px:g} {py:g} {pz:g} )', meshLine)
                            pos = Vec3(weightData['px'], weightData['py'], weightData['pz'])
                            mesh.weights[weightData['idx']] = Weight(
                                weightData['joint'],
                                weightData['bias'],
                                pos)
        return self

    def prepareMesh(self, m):
        o_vertices = []

        for v in m.vertices:
            startWeight = v.startWeight
            countWeight = v.countWeight
            finalPos = Vec3()

            for i in range(countWeight):
                w = m.weights[startWeight+i]
                joint = self.joints[w.jointIndex]

                wv = joint.orient.rotatePoint(w.pos)

                finalPos.x += (joint.pos.x + wv.x) * w.bias
                finalPos.y += (joint.pos.y + wv.y) * w.bias
                finalPos.z += (joint.pos.z + wv.z) * w.bias

            o_vertices.append(SkinnedVertex(finalPos, v.st))

        
        vertData = bytearray()
        faceData = bytearray()
        for v in o_vertices:
            vertData += v.pack()
        for t in m.tris:
            faceData += t.pack()
        
        print(m.numVerts, m.numTris * 3)
        headerData = struct.pack('<ii', m.numVerts, m.numTris * 3)
        with open('model.bin', 'wb') as f:
            f.write(headerData + vertData + faceData)

    def to_MDL(self, binary=False):
        for m in self.meshes:
            self.prepareMesh(m)

class MD5Anim:
    pass

if __name__ == '__main__':
    model = MD5Model()
    model.from_file('cubeguy.md5mesh')
    model.to_MDL()
