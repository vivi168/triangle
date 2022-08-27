import struct

# import miniball
import numpy as np
from pygltflib import GLTF2

BYTE = 5120
UNSIGNED_BYTE = 5121
SHORT = 5122
UNSIGNED_SHORT = 5123
UNSIGNED_INT = 5125
FLOAT = 5126


COMPONENT_SIZE = {
    BYTE: 1,
    UNSIGNED_BYTE: 1,
    SHORT: 2,
    UNSIGNED_SHORT: 2,
    UNSIGNED_INT: 4,
    FLOAT: 4,
}

SCALAR = "SCALAR"
VEC2 = "VEC2"
VEC3 = "VEC3"
VEC4 = "VEC4"
MAT2 = "MAT2"
MAT3 = "MAT3"
MAT4 = "MAT4"

TYPE_SIZE = {
    SCALAR: 1,
    VEC2: 2,
    VEC3: 3,
    VEC4: 4,
    MAT2: 4,
    MAT3: 9,
    MAT4: 12,
}

ARRAY_BUFFER = 34962  # eg vertex data
ELEMENT_ARRAY_BUFFER = 34963  # eg index data


class Vertex:
    def __init__(self):
        self.position = []
        self.normals = []
        self.uvs = []
        self.joint_ids = []
        self.weights = []

# load an example gltf file from the khronos collection
# fname = "/home/vbihl/Documents/gltf/human.gltf"
fname = 'simple.gltf'
gltf = GLTF2().load(fname)

# get the first mesh in the current scene (in this example there is only one scene and one mesh)
node_id = gltf.scenes[gltf.scene].nodes[0]
node = gltf.nodes[node_id]
mesh = gltf.meshes[node_id]

# get the vertices for each primitive in the mesh (in this example there is only one)
# TODO: handle multiple primitives (inn.gltf)
for primitive in mesh.primitives:
    # TODO append everything in there to a global object, to handle multiple primitives
    # get the binary data for this mesh primitive from the buffer
    accessor = gltf.accessors[primitive.attributes.POSITION]
    bufferView = gltf.bufferViews[accessor.bufferView]
    buffer = gltf.buffers[bufferView.buffer]
    data = gltf.get_data_from_buffer_uri(buffer.uri)

    # pull each vertex from the binary buffer and convert it into a tuple of python floats
    vertices = []
    SIZE = COMPONENT_SIZE[accessor.componentType] * TYPE_SIZE[accessor.type]
    # print(COMPONENT_SIZE[accessor.componentType])
    # print(TYPE_SIZE[accessor.type])
    # print(accessor.type)
    # print(accessor.componentType)
    # TODO also need normal, UV, bone indices, bone weights
    for i in range(accessor.count):
        index = bufferView.byteOffset + accessor.byteOffset + i*SIZE  # the location in the buffer of this vertex
        d = data[index:index+SIZE]  # the vertex data
        v = Vertex()
        v.position = list(struct.unpack("<fff", d))   # convert from base64 to three floats
        vertices.append(v)

    # TODO: uvs, normals : make function to do the above, with certain primitive.attributes.XXX
    # TODO: For each primite : record VertexStart,VertexCount,FaceStart,FaceCount

    ## Joint_ids TODO : create a vertex struct to hold position, normal, uv, joint id, weights
    accessor = gltf.accessors[primitive.attributes.JOINTS_0]
    # print(accessor)
    bufferView = gltf.bufferViews[accessor.bufferView]
    # print(bufferView)
    buffer = gltf.buffers[bufferView.buffer]
    data = gltf.get_data_from_buffer_uri(buffer.uri)
    joint_ids = []
    SIZE = COMPONENT_SIZE[accessor.componentType] * TYPE_SIZE[accessor.type]
    # print(SIZE)
    for i in range(accessor.count):
        index = bufferView.byteOffset + accessor.byteOffset + i*bufferView.byteStride # TODO determine when to use SIZE vs byteStride
        d = data[index:index+SIZE]
        j = list(struct.unpack("<HHHH", d))
        # print(j)
        joint_ids.append(j)
        vertices[i].joint_ids = j



    ## Weights
    accessor = gltf.accessors[primitive.attributes.WEIGHTS_0]
    bufferView = gltf.bufferViews[accessor.bufferView]
    buffer = gltf.buffers[bufferView.buffer]
    data = gltf.get_data_from_buffer_uri(buffer.uri)
    weights = []
    SIZE = COMPONENT_SIZE[accessor.componentType] * TYPE_SIZE[accessor.type]
    for i in range(accessor.count):
        index = bufferView.byteOffset + accessor.byteOffset + i*SIZE  # the location in the buffer of this vertex
        d = data[index:index+SIZE]  # the vertex data
        w = list(struct.unpack("<ffff", d))
        weights.append(w)
        # print(w)
        vertices[i].weights = w


    # ## Indices
    # print(primitive.attributes.POSITION)
    # print(primitive.indices)
    # print()
    accessor = gltf.accessors[primitive.indices]
    bufferView = gltf.bufferViews[accessor.bufferView]
    buffer = gltf.buffers[bufferView.buffer]
    data = gltf.get_data_from_buffer_uri(buffer.uri)

    # print(accessor.count)

    indices = []
    SIZE = COMPONENT_SIZE[accessor.componentType] * 3
    for i in range(accessor.count // 3):
        index = bufferView.byteOffset + accessor.byteOffset + i*SIZE
        d = data[index:index+SIZE]
        idx = list(struct.unpack("<HHH", d))
        indices.append(idx)



## Skins
## https://github.com/KhronosGroup/glTF-Tutorials/blob/master/gltfTutorial/gltfTutorial_020_Skins.md
skins = gltf.skins
# print(skins)

for skin in skins:
    ## Inverse bind matrices
    accessor = gltf.accessors[skin.inverseBindMatrices]
    bufferView = gltf.bufferViews[accessor.bufferView]
    buffer = gltf.buffers[bufferView.buffer]
    data = gltf.get_data_from_buffer_uri(buffer.uri)
    stride = 16 * 4 # TODO
    size = 16 * 4 # TODO
    matrices = []
    for i in range(accessor.count):
        index = bufferView.byteOffset + accessor.byteOffset + i*stride
        d = data[index:index+size]
        m = list(struct.unpack("<ffffffffffffffff", d))
        #print(m)
        matrices.append(m)


    ## Joint hierarchy
    ## "joints" : [ 1, 2 ]
    # joints[0] -> node[1] -> children = [2]
    # joints[1] -> node[2]
    # output should be :
    # joint -> parent

    """
    bone 0: -1
    bone 1: 0
    """
    jointdata = []
    joint_hierarchy = [-1]
    idx = 0
    for joint in skin.joints:
        jointdata.append({
                'idx': idx,
                'children': [item - 1 for item in gltf.nodes[joint].children],
                'node_children': gltf.nodes[joint].children,
                'rotation': gltf.nodes[joint].rotation,
                'translation': gltf.nodes[joint].translation,
                'scale': gltf.nodes[joint].scale,
            })
        idx += 1
    # print(jointdata)
    for joint in jointdata:
        children = joint['children']
        for child in children:
            for j in jointdata:
                if child in j['children']:
                    joint_hierarchy.insert(child, j['idx'])

    # print(joint_hierarchy)
## Animation
## https://github.com/KhronosGroup/glTF-Tutorials/blob/master/gltfTutorial/gltfTutorial_007_Animations.md


print('vertices:', len(vertices))
print('triangles:', len(indices))
print('inverse bind matrices:', len(matrices))
print('bones:', len(joint_hierarchy))

print("# Vertices")
for vertex in vertices:
    print('position', vertex.position[0], vertex.position[1], vertex.position[2])
    print('joint_ids', vertex.joint_ids[0], vertex.joint_ids[1], vertex.joint_ids[2], vertex.joint_ids[3])
    print('weights', vertex.weights[0], vertex.weights[1], vertex.weights[2], vertex.weights[3])

print("# Triangles")
for index in indices:
    print(index[0], index[1], index[2])

print("# Inverse bind matrices (one per bone)")
for m in matrices:
    print(m[0], m[1], m[2], m[3], m[4], m[5], m[6], m[7], m[8], m[9], m[10], m[11], m[12], m[13], m[14], m[15])

print("# Joints hierarchy")
for j in joint_hierarchy:
    print(j)

# convert a numpy array for some manipulation
# S = numpy.array(vertices)

# print(S)

# use a third party library to perform Ritter's algorithm for finding smallest bounding sphere
# C, radius_squared = miniball.get_bounding_ball(S)

# output the results
# print(f"center of bounding sphere: {C}\nradius squared of bounding sphere: {radius_squared}")
