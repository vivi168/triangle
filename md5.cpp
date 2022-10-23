#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cmath>
#include "md5.h"

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

	//printf("MD5 model\n%d, %d\n", model->header.numJoints, model->header.numMeshes);

	model->joints = (MD5Joint*)malloc(sizeof(MD5Joint) * model->header.numJoints);

	fread(model->joints, sizeof(MD5Joint), model->header.numJoints, fp);

	for (int i = 0; i < model->header.numJoints; i++) {
		MD5Joint* j = &model->joints[i];

		//printf("parent: %d (%f %f %f) (%f %f %f %f)\n",
		//	j->parent,
		//	j->pos[X], j->pos[Y], j->pos[Z],
		//	j->orient[X], j->orient[Y], j->orient[Z], j->orient[W]
		//);
	}

	// READ MESH
	model->meshes = (MD5Mesh*)malloc(sizeof(MD5Mesh) * model->header.numMeshes);

	for (int i = 0; i < model->header.numMeshes; i++) {
		MD5Mesh* mesh = &model->meshes[i];

		fread(&mesh->header, sizeof(MD5MeshHeader), 1, fp);

		// Verts
		mesh->vertices = (MD5Vertex*)malloc(sizeof(MD5Vertex) * mesh->header.numVerts);
		fread(mesh->vertices, sizeof(MD5Vertex), mesh->header.numVerts, fp);

		//for (int v = 0; v < mesh->header.numVerts; v++) {
		//	printf("(%f %f) %d %d\n",
		//		mesh->vertices[v].st[X], mesh->vertices[v].st[Y],
		//		mesh->vertices[v].startWeight, mesh->vertices[v].countWeight
		//		);
		//}

		// Tris
		mesh->indices = (int*)malloc(sizeof(int) * mesh->header.numTris * 3);
		fread(mesh->indices, sizeof(int) * 3, mesh->header.numTris, fp);

		//for (int t = 0; t < mesh->header.numTris; t++) {
		//	printf("%d %d %d\n",
		//		mesh->tris[t].vertIndices[0],
		//		mesh->tris[t].vertIndices[1],
		//		mesh->tris[t].vertIndices[2]
		//	);
		//}

		// Weights
		mesh->weights = (MD5Weight*)malloc(sizeof(MD5Weight) * mesh->header.numWeights);
		fread(mesh->weights, sizeof(MD5Weight), mesh->header.numWeights, fp);

		//for (int w = 0; w < mesh->header.numWeights; w++) {
		//	printf("%d %f (%f %f %f)\n",
		//		mesh->weights[w].jointIndex,
		//		mesh->weights[w].bias,
		//		mesh->weights[w].pos[X], mesh->weights[w].pos[Y], mesh->weights[w].pos[Z]
		//	);
		//}
	}
}

void read_md5anim(const char* filename, MD5Anim* anim)
{
	FILE* fp;
#ifdef _WIN32
	fopen_s(&fp, filename, "rb");
#else
	fp = fopen(filename, "rb");
#endif

	if (!fp)
		exit(EXIT_FAILURE);

	fread(&anim->header, sizeof(MD5AnimHeader), 1, fp);

	printf("MD5 anim\n%d, %d, %d\n", anim->header.numFrames, anim->header.numJoints, anim->header.frameRate);

	anim->frameJoints = (MD5Joint**)malloc(sizeof(MD5Joint*) * anim->header.numFrames);

	for (int i = 0; i < anim->header.numFrames; i++) {
		anim->frameJoints[i] = (MD5Joint*)malloc(sizeof(MD5Joint) * anim->header.numJoints);
		fread(anim->frameJoints[i], sizeof(MD5Joint), anim->header.numJoints, fp);

		for (int k = 0; k < anim->header.numJoints; k++) {
			MD5Joint* j = &anim->frameJoints[i][k];

			//printf("parent: %d (%f %f %f) (%f %f %f %f)\n",
			//	j->parent,
			//	j->pos[X], j->pos[Y], j->pos[Z],
			//	j->orient[X], j->orient[Y], j->orient[Z], j->orient[W]
			//);
		}
	}
}

void quat_normalize(quat q)
{
	float mag = sqrt((q[X] * q[X]) + (q[Y] * q[Y]) + (q[Z] * q[Z]) + (q[W] * q[W]));

	if (mag > 0.0f)
	{
		float one_over_mag = 1.0f / mag;

		q[X] *= one_over_mag;
		q[Y] *= one_over_mag;
		q[Z] *= one_over_mag;
		q[W] *= one_over_mag;
	}
}

void quat_mulvec(const quat q, const vec3 v, quat out)
{
	out[W] = -(q[X] * v[X]) - (q[Y] * v[Y]) - (q[Z] * v[Z]);
	out[X] =  (q[W] * v[X]) + (q[Y] * v[Z]) - (q[Z] * v[Y]);
	out[Y] =  (q[W] * v[Y]) + (q[Z] * v[X]) - (q[X] * v[Z]);
	out[Z] =  (q[W] * v[Z]) + (q[X] * v[Y]) - (q[Y] * v[X]);
}

void quat_mulquat(const quat qa, const quat qb, quat out)
{
	out[W] = (qa[W] * qb[W]) - (qa[X] * qb[X]) - (qa[Y] * qb[Y]) - (qa[Z] * qb[Z]);
	out[X] = (qa[X] * qb[W]) + (qa[W] * qb[X]) + (qa[Y] * qb[Z]) - (qa[Z] * qb[Y]);
	out[Y] = (qa[Y] * qb[W]) + (qa[W] * qb[Y]) + (qa[Z] * qb[X]) - (qa[X] * qb[Z]);
	out[Z] = (qa[Z] * qb[W]) + (qa[W] * qb[Z]) + (qa[X] * qb[Y]) - (qa[Y] * qb[X]);
}

void quat_rotate_point(const quat q, const vec3 in, vec3 out)
{
	quat tmp, inv, final;

	inv[X] = -q[X]; inv[Y] = -q[Y];
	inv[Z] = -q[Z]; inv[W] = q[W];

	quat_normalize(inv);

	quat_mulvec(q, in, tmp);
	quat_mulquat(tmp, inv, final);

	out[X] = final[X];
	out[Y] = final[Y];
	out[Z] = final[Z];
}

void prepare_vertices(const MD5Mesh* mesh, const MD5Joint* joints, Vertex** vertices, const int offset)
{
	for (int k = 0; k < mesh->header.numVerts; k++) {
		MD5Vertex* v = &mesh->vertices[k];
		vec3 finalPos = { 0, 0, 0 };

		for (int i = 0; i < v->countWeight; i++) {
			MD5Weight* w = &mesh->weights[v->startWeight + i];
			const MD5Joint* joint = &joints[w->jointIndex];

			vec3 wv;
			quat_rotate_point(joint->orient, w->pos, wv);

			finalPos[X] += (joint->pos[X] + wv[X]) * w->bias;
			finalPos[Y] += (joint->pos[Y] + wv[Y]) * w->bias;
			finalPos[Z] += (joint->pos[Z] + wv[Z]) * w->bias;
		}

		(*vertices)[k + offset].position.x = finalPos[X];
		(*vertices)[k + offset].position.y = finalPos[Z];
		(*vertices)[k + offset].position.z = -finalPos[Y];
		(*vertices)[k + offset].uv.x = v->st[X];
		(*vertices)[k + offset].uv.y = v->st[Y];
	}
}

// TODO: pass joint array to use instead of bind pose
void prepare_model(const MD5Model* model, const MD5Joint* joints, Vertex** vertices, int** indices, int* nv, int* nt)
{
	int numVerts = 0;
	int numTris = 0;
	for (int i = 0; i < model->header.numMeshes; i++) {
		numVerts += model->meshes[i].header.numVerts;
		numTris += model->meshes[i].header.numTris;
	}

	*vertices = (Vertex*)malloc(sizeof(Vertex) * numVerts);
	*indices = (int*)malloc(sizeof(int) * numTris * 3);

	int vertOffset = 0;
	int triOffset = 0;
	for (int i = 0; i < model->header.numMeshes; i++) {
		prepare_vertices(&model->meshes[i], joints, vertices, vertOffset);

		for (int t = 0; t < model->meshes[i].header.numTris * 3; t++) {
			(*indices)[triOffset + t] = model->meshes[i].indices[t] + vertOffset;
		}

		vertOffset += model->meshes[i].header.numVerts;
		triOffset += model->meshes[i].header.numTris * 3;
	}

	*nv = numVerts;
	*nt = numTris;
}