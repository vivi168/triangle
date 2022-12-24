#pragma once

#include <vector>
#include <glm/glm.hpp>

enum {
	X = 0, Y, Z, W
};

typedef float vec2[2];
typedef float vec3[3];
typedef float quat[4];

struct Vertex {
	vec3 position;
	vec3 normal;
	vec2 uv;
};

struct SubsetHeader {
	int start, count, nameSize;
};

struct Subset {
	SubsetHeader header;
	std::string name;

	int getNameSize()
	{
		return header.nameSize;
	}
};

struct BaseMesh {
	std::vector<int> indices;
	std::vector<Subset> subsets;
};

struct Mesh : BaseMesh {
	std::vector<Vertex> vertices;
};
