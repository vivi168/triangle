import md5
import numpy as np

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
    def __init__(self, m):
        self.m = m

    def __str__(self):
        return "{:.6f} {:.6f} {:.6f} {:.6f} {:.6f} {:.6f} {:.6f} {:.6f} {:.6f} {:.6f} {:.6f} {:.6f} {:.6f} {:.6f} {:.6f} {:.6f}".format(
            self.m[0][0], self.m[0][1], self.m[0][2], self.m[0][3],
            self.m[1][0], self.m[1][1], self.m[1][2], self.m[1][3],
            self.m[2][0], self.m[2][1], self.m[2][2], self.m[2][3],
            self.m[3][0], self.m[3][1], self.m[3][2], self.m[3][3]
        )

    def pack(self):
        return struct.pack('<ffffffffffffffff',
            self.m[0][0], self.m[0][1], self.m[0][2], self.m[0][3],
            self.m[1][0], self.m[1][1], self.m[1][2], self.m[1][3],
            self.m[2][0], self.m[2][1], self.m[2][2], self.m[2][3],
            self.m[3][0], self.m[3][1], self.m[3][2], self.m[3][3]
        )

class M3DTriangle:
    def __init__(self, a=0, b=0, c=0):
        self.a = a
        self.b = b
        self.c = c

    def pack(self):
        return struct.pack('<iii', self.a, self.b, self.c)

    def __str__(self):
        return '{} {} {}'.format(self.a, self.b, self.c)


class M3DSkinnedVertex:
    def __init__(self, p=Vec3(), n=Vec3(), t=Vec2()):
        self.position = p
        self.normal = n
        self.uv = t
        self.blend_idx = [0] * MAX_WEIGHT
        self.blend_weights = [0.0] * MAX_WEIGHT

    def blend_idx_str(self):
        return '{} {} {} {}'.format(
            self.blend_idx[0],
            self.blend_idx[1],
            self.blend_idx[2],
            self.blend_idx[3])

    def blend_weights_str(self):
        return '{:.6f} {:.6f} {:.6f} {:.6f}'.format(
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

        self.inv_bindpose = []

    def compute_inv_bindpose(self):
        for joint in self.md5_model.joints:
            transMat = joint.pos.trans4x4()
            rotMat = joint.orient.rot4x4()

            mat = np.linalg.inv(np.matmul(transMat, rotMat))
            self.inv_bindpose.append(Mat4x4(mat))


    def prepare_vertice(self, mesh, joints, offset):
        # TODO: Should also compute normals
        for k in range(mesh.numVerts):
            vertice = M3DSkinnedVertex()
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

                vertice.blend_idx[i] = w.jointIndex
                vertice.blend_weights[i] = w.bias

            vertice.position = finalPos
            vertice.uv = v.st

            self.vertices[k + offset] = vertice

    def prepare(self):
        numVerts = 0
        numTris = 0
        numMeshes = self.md5_model.numMeshes

        self.subsets = [None] * numMeshes

        start = 0
        for i in range(numMeshes):
            subset = M3DSubset()
            nt = self.md5_model.meshes[i].numTris
            numVerts += self.md5_model.meshes[i].numVerts
            numTris += nt

            subset.start = start
            subset.count = nt * 3
            subset.name = self.md5_model.meshes[i].shader
            self.subsets[i] = subset
            start += nt * 3

        self.vertices = [None] * numVerts
        self.tris = [None] * numTris

        vertOffset = 0
        triOffset = 0
        for i in range(numMeshes):
            self.prepare_vertice(self.md5_model.meshes[i], self.md5_model.joints, vertOffset)

            for t in range(self.md5_model.meshes[i].numTris):
                triangle = M3DTriangle()

                triangle.a = self.md5_model.meshes[i].tris[t].vertIndices[0] + vertOffset
                triangle.b = self.md5_model.meshes[i].tris[t].vertIndices[1] + vertOffset
                triangle.c = self.md5_model.meshes[i].tris[t].vertIndices[2] + vertOffset
                self.tris[triOffset + t] = triangle

            vertOffset += self.md5_model.meshes[i].numVerts
            triOffset += self.md5_model.meshes[i].numTris

        return self

    def export_header(self):
        print("### Header ###")
        print("Vertices {}".format(len(self.vertices)))
        print("Triangles {}".format(len(self.tris)))
        print("Subsets {}".format(len(self.subsets)))
        print("Bones {}".format(len(self.md5_model.joints)))
        print()

    def export_subsets(self):
        print("### Subsets ###")
        for s in self.subsets:
            print(s)
        print()

    def export_vertices(self):
        print("### Vertices ###")
        for v in self.vertices:
            print(v)
        print()

    def export_triangles(self):
        print("### Triangles ###")
        for t in self.tris:
            print(t)
        print()

    def export_inv_bindpose(self):
        print("### Inverse Bindpose Matrices ###")
        for m in self.inv_bindpose:
            print(m)
        print()

    def export(self):
        self.export_header()
        self.export_subsets()
        self.export_vertices()
        self.export_triangles()
        self.export_inv_bindpose()

class M3DAnimation:
    def __init__(self, md5_anim):
        self.md5_anim = md5_anim

    def export(self):
        for f, frameJoints in enumerate(self.md5_anim.frameJoints):
            print(f)
            for joint in frameJoints:
                print(joint)
            print()

if __name__ == '__main__':
    # --- Model
    md5_model = md5.MD5Model()
    md5_model.from_file('Bob.md5mesh')

    m3d_model = M3DMesh(md5_model)
    m3d_model.prepare()
    m3d_model.compute_inv_bindpose()
    m3d_model.export()

    # --- Animation
    md5_anim = md5.MD5Anim()
    md5_anim.from_file('Bob.md5anim')

    m3d_anim = M3DAnimation(md5_anim)
    # m3d_anim.export()
