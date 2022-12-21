#pragma once

#include <string>

#include "common.h"

#define MAX_BONES 100
#define MAX_WEIGHTS 4

struct SkinnedVertex : Vertex {
	float blend_idx[MAX_WEIGHTS];
	float blend_weights[MAX_WEIGHTS];
};

struct SkinnedMesh {
	std::vector<SkinnedVertex> vertices;
	std::vector<int> indices;
	std::vector<Subset> subsets;
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
	int shaderSize;
};

struct MD5Mesh {
	MD5MeshHeader header;

	std::vector<MD5Vertex> vertices;
	std::vector<int> indices;
	std::vector<MD5Weight> weights;
	std::string shader;
};

struct MD5ModelHeader {
	int numJoints;
	int numMeshes;
};

struct MD5Model {
	MD5ModelHeader header;

	std::vector<MD5Joint> joints;
	std::vector<MD5Mesh> meshes;

	void read(const char* filename);
	SkinnedMesh prepare() const;
	std::vector<glm::mat4> inv_bindpose_matrices() const;
};

struct MD5AnimHeader {
	int numFrames;
	int numJoints;
	int frameRate;
};

struct MD5Anim {
	MD5AnimHeader header;
	std::vector<std::vector<MD5Joint>> frameJoints; // frameJoints[numFrames][numJoints]

	void read(const char* filename);
	std::vector<glm::mat4> bone_matrices(int frame);
};

struct MD5AnimInfo {
	int currFrame;
	int nextFrame;

	double time;
	double frameDuration;
};

void animate(const MD5Anim* anim, MD5AnimInfo* animInfo, float dt);
