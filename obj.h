#pragma once

#include "common.h"

struct ObjModel {
	std::vector<Vertex> vertices;
	std::vector<int> indices;
	std::vector<Subset> subsets;
};