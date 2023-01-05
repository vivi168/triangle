import md5

MAX_WEIGHT = 4

class Vec2:
    def __init__(self, x=0, y=0):
        self.x = x
        self.y = y

    def pack(self):
        return struct.pack('<ff', self.x, self.y)

    def __str__(self):
        return '{:.6f} {:.6f}'.format(self.x, self.y)

    def __eq__(self, other):
        return self.x == other.x and self.y == other.y


class Vec3:
    def __init__(self, x=0, y=0, z=0):
        self.x = x
        self.y = y
        self.z = z

    def pack(self):
        return struct.pack('<fff', self.x, self.y, self.z)

    def __str__(self):
        return '{:.6f} {:.6f} {:.6f}'.format(self.x, self.y, self.z)

    def __eq__(self, other):
        return self.x == other.x and self.y == other.y and self.z == other.z


class Mat4x4:
    def __init__(self):
        pass


class M3DTriangle:
    def __init__(self, vertIndices=[0] * 3):
        self.vertIndices = vertIndices

    def pack(self):
        return struct.pack('<iii', self.vertIndices[0], self.vertIndices[1], self.vertIndices[2])

    def __str__(self):
        return '{} {} {}'.format(
            self.vertIndices[0], self.vertIndices[1], self.vertIndices[2])


class M3DSkinnedVertex:
    def __init__(self, p=Vec3(), n=Vec3(), t=Vec2()):
        self.position = p
        self.normal = n
        self.uv = t
        self.blend_idx = [0] * MAX_WEIGHT
        self.blend_weights = [0.0] * MAX_WEIGHT

    def blend_idx_str(self):
        return '[{} {} {} {}]'.format(
            self.blend_idx[0],
            self.blend_idx[1],
            self.blend_idx[2],
            self.blend_idx[3])

    def blend_weights_str(self):
        return '({:.6f} {:.6f} {:.6f} {:.6f})'.format(
            self.blend_weights[0],
            self.blend_weights[1],
            self.blend_weights[2],
            self.blend_weights[3])

    def __str__(self):
        return '{} {} {} {} {}'.format(
            self.position,
            self.normal,
            self.uv,
            self.blend_idx_str(),
            self.blend_weights_str())

class M3DSubset:
    def __init__(self, name='', start=0):
        self.start = start
        self.count = 0
        self.name = name

    def __str__(self):
        return '{} {} {}'.format(self.start, self.count, self.name)

    def pack(self):
        data = struct.pack('<iii', self.start, self.count, len(self.name))

        return data + bytes(self.name, 'ascii')


class M3DMesh:
    def __init__(self, md5_model):
        self.md5_model = md5_model

        self.vertices = []
        self.tris = []
        self.subsets = []

    def inverse_bindpose(self):
        for joint in self.md5_model.joints:
            print(joint)

    def prepare_vertice(self, mesh, joints, offset):
        for k in range(mesh.numVerts):
            v = mesh.verts[k]
            finalPos = Vec3()

            assert(v.countWeight <= MAX_WEIGHT)

            for i in range(v.countWeight):
                w = mesh.weights[v.startWeight + i]
                joint = joints[w.jointIndex]

                wv = joint.orient.rotatePoint(w.pos)
                finalPos.x += (joint.pos.x + wv.x) * w.bias
                finalPos.y += (joint.pos.y + wv.y) * w.bias
                finalPos.z += (joint.pos.z + wv.z) * w.bias

                self.vertices[k + offset].blend_idx[i] = w.jointIndex
                self.vertices[k + offset].blend_weights[i] = w.bias

            self.vertices[k + offset].position = finalPos
            self.vertices[k + offset].uv = v.st
            print(self.vertices[k + offset])

    def prepare(self):
        numVerts = 0
        numTris = 0
        numMeshes = self.md5_model.numMeshes

        self.subsets = [M3DSubset()] * numMeshes

        start = 0
        for i in range(numMeshes):
            nt = self.md5_model.meshes[i].numTris
            numVerts += self.md5_model.meshes[i].numVerts
            numTris += nt

            self.subsets[i].start = start
            self.subsets[i].count = nt * 3
            self.subsets[i].name = self.md5_model.meshes[i].shader
            start += nt * 3

        self.vertices = [M3DSkinnedVertex()] * numVerts
        self.tris = [M3DTriangle()] * numTris

        vertOffset = 0
        triOffset = 0
        for i in range(numMeshes):
            self.prepare_vertice(self.md5_model.meshes[i], self.md5_model.joints, vertOffset)

            for t in range(self.md5_model.meshes[i].numTris):
                self.tris[triOffset + t].vertIndices[0] = self.md5_model.meshes[i].tris[t].vertIndices[0] + vertOffset
                self.tris[triOffset + t].vertIndices[1] = self.md5_model.meshes[i].tris[t].vertIndices[1] + vertOffset
                self.tris[triOffset + t].vertIndices[2] = self.md5_model.meshes[i].tris[t].vertIndices[2] + vertOffset

            vertOffset += self.md5_model.meshes[i].numVerts
            triOffset += self.md5_model.meshes[i].numTris

        return self


if __name__ == '__main__':
    md5_model = md5.MD5Model()
    md5_model.from_file('Bob.md5mesh')

    m3d_model = M3DMesh(md5_model)

    m3d_model.inverse_bindpose()

    m3d_model.prepare()
