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

	// std::cout << "HEADER " << header.numVerts << " " << header.numTris << " " << header.numSubsets << "\n";

	// Verts
	mesh.vertices.resize(header.numVerts);
	fread(mesh.vertices.data(), sizeof(Vertex), header.numVerts, fp);

	// Tris
	mesh.indices.resize(header.numTris * 3);
	fread(mesh.indices.data(), sizeof(int) * 3, header.numTris, fp);

	// Subsets
	mesh.subsets.resize(header.numSubsets);
	fread(mesh.subsets.data(), sizeof(Subset), header.numVerts, fp);

	//for (auto v : mesh.vertices) {
	//	std::cout <<
	//		"(" << v.position[X] << " " << v.position[Y] << " " << v.position[Z] << ") " <<
	//		"(" << v.normal[X] << " " << v.normal[Y] << " " << v.normal[Z] << ") " <<
	//		"(" << v.uv[X] << " " << v.uv[Y] << ")\n";
	//}

	//int i = 0;
	//for (auto t : mesh.indices) {
	//	std::cout << t << " ";
	//	i++;
	//	if (i % 3 == 0) std::cout << "\n";
	//}

	//for (auto t : mesh.subsets) {

	//	std::cout << t.start << " " << t.count << "\n";
	//}

	return mesh;
}