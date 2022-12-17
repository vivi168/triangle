#pragma once

#include <vector>

#include "common.h"

#define MAX_BONES 100
#define MAX_WEIGHTS 4

struct SkinnedVertex : Vertex {
	float blend_idx[MAX_WEIGHTS];
	float blend_weights[MAX_WEIGHTS];
};

struct MD5Vertex {
	vec2 st;
	int startWeight;
	int countWeight;
};

struct MD5Weight {
	int jointIndex;
	float bias;
	vec3 pos;
};

struct MD5Joint {
	int parent;
	vec3 pos;
	quat orient;
};

struct MD5MeshHeader {
	int numVerts;
	int numTris;
	int numWeights;
};

struct MD5Mesh {
	MD5MeshHeader header;

	MD5Vertex* vertices;
	int* indices;
	MD5Weight* weights;
};

struct MD5ModelHeader {
	int numJoints;
	int numMeshes;
};

struct MD5Model {
	MD5ModelHeader header;

	MD5Joint* joints;
	MD5Mesh* meshes;

	std::vector<glm::mat4x4> invBindPose;
};

struct MD5AnimHeader {
	int numFrames;
	int numJoints;
	int frameRate;
};

struct MD5Anim {
	MD5AnimHeader header;
	MD5Joint** frameJoints; // frameJoints[numFrames][numJoints]
};

struct MD5AnimInfo {
	int currFrame;
	int nextFrame;

	double time;
	double frameDuration;
};

void read_md5model(const char* filename, MD5Model* model);
void read_md5anim(const char* filename, MD5Anim* anim);

void prepare_model(const MD5Model* model, const MD5Joint* joints, SkinnedVertex** vertices, int** indices, int*, int*);

void animate(const MD5Anim* anim, MD5AnimInfo* animInfo, float dt);

// for GPU

void build_invbindpose(MD5Model* model);
std::vector<glm::mat4> build_bonematrix(const MD5Model* model, const MD5Anim* anim, int frame);
