#include <cstdio>
#include <cstdlib>
#include "md5.h"

/*
model.bin -> numJoint, numMeshes, joints[]
		  -> numVerts, numTris, numWeights, verts[], tris[], weights[]

sizeof:
vertex: 16
triangle: 12
weight: 20
joint: 32



read first 8 bytes (2 int)
malloc numJoint * sizeof(Joint) (numJoint * 32 bytes)
malloc numMeshes * sizeof(mesh)
read numJoint * 32 bytes
for i in range(numMeshes)
	read 12 bytes (3 int)
	MD5Mesh *mesh = &model->mesh[i]
	malloc numVerts * sizeof(Vert), numTris * sizeof(Tri), numWeight * sizeof(Weight)
	mesh->vertes = malloc(numVerts * sizeof(Vert))
*/

void read_md5model(const char* filename, MD5Model* model)
{
	FILE* fp;
#ifdef _WIN32
	fopen_s(&fp, filename, "rb");
#else
	fp = fopen(filename, "rb");
#endif

	if (!fp)
		exit(EXIT_FAILURE);

	fread(&model->header, sizeof(MD5ModelHeader), 1, fp);

	printf("%d, %d\n", model->header.numJoints, model->header.numMeshes);

	model->joints = (MD5Joint*)malloc(sizeof(MD5Joint) * model->header.numJoints);

	fread(model->joints, sizeof(MD5Joint), model->header.numJoints, fp);

	for (int i = 0; i < model->header.numJoints; i++) {
		MD5Joint* j = &model->joints[i];

		printf("parent: %d (%f %f %f) (%f %f %f %f)\n",
			j->parent,
			j->pos[X], j->pos[Y], j->pos[Z],
			j->orient[X], j->orient[Y], j->orient[Z], j->orient[W]
		);
	}

	// READ MESH
	model->meshes = (MD5Mesh*)malloc(sizeof(MD5Mesh) * model->header.numMeshes);

	for (int i = 0; i < model->header.numMeshes; i++) {
		MD5Mesh* mesh = &model->meshes[i];

		fread(&mesh->header, sizeof(MD5MeshHeader), 1, fp);

		// Verts
		mesh->vertices = (MD5Vertex*)malloc(sizeof(MD5Vertex) * mesh->header.numVerts);
		fread(mesh->vertices, sizeof(MD5Vertex), mesh->header.numVerts, fp);

		for (int v = 0; v < mesh->header.numVerts; v++) {
			printf("(%f %f) %d %d\n",
				mesh->vertices[v].st[X], mesh->vertices[v].st[Y],
				mesh->vertices[v].startWeight, mesh->vertices[v].countWeight
				);
		}

		// Tris
		mesh->tris = (MD5Triangle*)malloc(sizeof(MD5Triangle) * mesh->header.numTris);
		fread(mesh->tris, sizeof(MD5Triangle), mesh->header.numTris, fp);

		for (int t = 0; t < mesh->header.numTris; t++) {
			printf("%d %d %d\n",
				mesh->tris[t].vertIndices[0],
				mesh->tris[t].vertIndices[1],
				mesh->tris[t].vertIndices[2]
			);
		}

		// Weights
		mesh->weights = (MD5Weight*)malloc(sizeof(MD5Weight) * mesh->header.numWeights);
		fread(mesh->weights, sizeof(MD5Weight), mesh->header.numWeights, fp);

		for (int w = 0; w < mesh->header.numWeights; w++) {
			printf("%d %f (%f %f %f)\n",
				mesh->weights[w].jointIndex,
				mesh->weights[w].bias,
				mesh->weights[w].pos[X], mesh->weights[w].pos[Y], mesh->weights[w].pos[Z]
			);
		}
	}
}