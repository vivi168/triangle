#pragma once

#include <glm/glm.hpp>

enum {
	X = 0, Y, Z, W
};

typedef float vec2[2];
typedef float vec3[3];
typedef float quat[4];

typedef struct md5_vertex_t {
	vec2 st;
	int startWeight;
	int countWeight;
} MD5Vertex;

typedef struct md5_triangle_t {
	int vertIndices[3];
} MD5Triangle;

typedef struct MD5_weight_t {
	int jointIndex;
	float bias;
	vec3 pos;
} MD5Weight;

typedef struct md5_joint_t {
	int parent;
	vec3 pos;
	quat orient;
} MD5Joint;

typedef struct md5_mesh_header_t {
	int numVerts;
	int numTris;
	int numWeights;
} MD5MeshHeader;

typedef struct md5_mesh_t {
	MD5MeshHeader header;

	MD5Vertex* vertices;
	MD5Triangle* tris;
	MD5Weight* weights;
} MD5Mesh;

typedef struct md5_model_header_t {
	int numJoints;
	int numMeshes;
} MD5ModelHeader;

typedef struct md5_model_t {
	MD5ModelHeader header;

	MD5Joint* joints;
	MD5Mesh* meshes;
} MD5Model;

typedef struct vertex_t {
	glm::vec3 position;
	glm::vec2 uv;
} Vertex;

void read_md5model(const char* filename, MD5Model* model);
void prepare_model(MD5Model* model, Vertex** vertices, int** indices, int*, int*);
