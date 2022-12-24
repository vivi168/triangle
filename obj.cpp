#include <iostream>
#include "obj.h"

struct ObjHeader {
	int numVerts;
	int numTris;
	int numSubsets;
};

Mesh Obj::prepare(const char* filename)
{
	Mesh mesh;

	FILE* fp;
#ifdef _WIN32
	fopen_s(&fp, filename, "rb");
#else
	fp = fopen(filename, "rb");
#endif

	assert(fp);

	// Header
	ObjHeader header;

	fread(&header, sizeof(ObjHeader), 1, fp);

	// Verts
	mesh.vertices.resize(header.numVerts);
	fread(mesh.vertices.data(), sizeof(Vertex), header.numVerts, fp);

	// Tris
	mesh.indices.resize(header.numTris * 3);
	fread(mesh.indices.data(), sizeof(int) * 3, header.numTris, fp);

	// Subsets
	mesh.subsets.resize(header.numSubsets);

	for (int i = 0; i < header.numSubsets; i++) {
		fread(&mesh.subsets[i].header, sizeof(SubsetHeader), 1, fp);

		mesh.subsets[i].name.resize(mesh.subsets[i].getNameSize());
		fread(mesh.subsets[i].name.data(), sizeof(char), mesh.subsets[i].getNameSize(), fp);

		std::cout << mesh.subsets[i].name << " " << mesh.subsets[i].getNameSize() << "\n";
	}

	return mesh;
}