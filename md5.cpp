#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cmath>

#include <glm/gtc/type_ptr.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtx/quaternion.hpp>

#include "md5.h"

void MD5Model::read(const char* filename)
{
	FILE* fp;
#ifdef _WIN32
	fopen_s(&fp, filename, "rb");
#else
	fp = fopen(filename, "rb");
#endif

	assert(fp);

	fread(&header, sizeof(MD5ModelHeader), 1, fp);

	assert(header.numJoints < MAX_BONES);

	joints.resize(header.numJoints);

	fread(joints.data(), sizeof(MD5Joint), header.numJoints, fp);

	meshes.resize(header.numMeshes);

	for (auto& mesh : meshes) {
		// Header
		fread(&mesh.header, sizeof(MD5MeshHeader), 1, fp);

		// Verts
		mesh.vertices.resize(mesh.header.numVerts);
		fread(mesh.vertices.data(), sizeof(MD5Vertex), mesh.header.numVerts, fp);

		// Tris
		mesh.indices.resize(mesh.header.numTris * 3);
		fread(mesh.indices.data(), sizeof(int) * 3, mesh.header.numTris, fp);

		// Weights
		mesh.weights.resize(mesh.header.numWeights);
		fread(mesh.weights.data(), sizeof(MD5Weight), mesh.header.numWeights, fp);
	}
}

void MD5Anim::read(const char* filename)
{
	FILE* fp;
#ifdef _WIN32
	fopen_s(&fp, filename, "rb");
#else
	fp = fopen(filename, "rb");
#endif

	assert(fp);

	fread(&header, sizeof(MD5AnimHeader), 1, fp);

	printf("MD5 anim\n%d, %d, %d\n", header.numFrames, header.numJoints, header.frameRate);

	frameJoints = (MD5Joint**)malloc(sizeof(MD5Joint*) * header.numFrames);
	assert(frameJoints);

	for (int i = 0; i < header.numFrames; i++) {
		frameJoints[i] = (MD5Joint*)malloc(sizeof(MD5Joint) * header.numJoints);
		assert(frameJoints[i]);
		fread(frameJoints[i], sizeof(MD5Joint), header.numJoints, fp);

		for (int k = 0; k < header.numJoints; k++) {
			MD5Joint* j = &frameJoints[i][k];
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
	quat tmp, inv, qout;

	inv[X] = -q[X]; inv[Y] = -q[Y];
	inv[Z] = -q[Z]; inv[W] = q[W];

	quat_normalize(inv);

	quat_mulvec(q, in, tmp);
	quat_mulquat(tmp, inv, qout);

	out[X] = qout[X];
	out[Y] = qout[Y];
	out[Z] = qout[Z];
}

void prepare_vertices(const MD5Mesh& mesh, const std::vector<MD5Joint>& joints, SkinnedVertex** vertices, const int offset)
{
	for (int k = 0; k < mesh.header.numVerts; k++) {
		const MD5Vertex& v = mesh.vertices[k];
		vec3 finalPos = { 0, 0, 0 };

		assert(v.countWeight <= MAX_WEIGHTS);

		memset((*vertices)[k + offset].blend_idx, 0, sizeof(float) * MAX_WEIGHTS);
		memset((*vertices)[k + offset].blend_weights, 0, sizeof(float) * MAX_WEIGHTS);

		for (int i = 0; i < v.countWeight; i++) {
			const MD5Weight& w = mesh.weights[v.startWeight + i];
			const MD5Joint& joint = joints[w.jointIndex];

			vec3 wv;
			quat_rotate_point(joint.orient, w.pos, wv);

			finalPos[X] += (joint.pos[X] + wv[X]) * w.bias;
			finalPos[Y] += (joint.pos[Y] + wv[Y]) * w.bias;
			finalPos[Z] += (joint.pos[Z] + wv[Z]) * w.bias;

			(*vertices)[k + offset].blend_idx[i] = (float)w.jointIndex;
			(*vertices)[k + offset].blend_weights[i] = w.bias;
		}

		memcpy((*vertices)[k + offset].position, finalPos, sizeof(vec3));
		memcpy((*vertices)[k + offset].uv, v.st, sizeof(vec2));
	}
}

// TODO: pass joint array to use instead of bind pose
void MD5Model::prepare(SkinnedVertex** vertices, int** indices, int* nv, int* nt) const
{
	int numVerts = 0;
	int numTris = 0;
	for (int i = 0; i < header.numMeshes; i++) {
		numVerts += meshes[i].header.numVerts;
		numTris += meshes[i].header.numTris;
	}

	*vertices = (SkinnedVertex*)malloc(sizeof(SkinnedVertex) * numVerts);
	*indices = (int*)malloc(sizeof(int) * numTris * 3);
	assert(*vertices);
	assert(*indices);

	int vertOffset = 0;
	int triOffset = 0;
	for (int i = 0; i < header.numMeshes; i++) {
		prepare_vertices(meshes[i], joints, vertices, vertOffset);

		for (int t = 0; t < meshes[i].header.numTris * 3; t++) {
			(*indices)[triOffset + t] = meshes[i].indices[t] + vertOffset;
		}

		vertOffset += meshes[i].header.numVerts;
		triOffset += meshes[i].header.numTris * 3;
	}

	*nv = numVerts;
	*nt = numTris;
}

void animate(const MD5Anim* anim, MD5AnimInfo* animInfo, float dt)
{
	int maxFrames = anim->header.numFrames - 1;

	animInfo->time += dt;

	if (animInfo->time >= animInfo->frameDuration) {
		animInfo->currFrame++;
		animInfo->nextFrame++;
		animInfo->time = 0.0;

		if (animInfo->currFrame > maxFrames) animInfo->currFrame = 0;
		if (animInfo->nextFrame > maxFrames) animInfo->nextFrame = 0;
	}
}

std::vector<glm::mat4> MD5Model::inv_bindpose_matrices() const
{
	std::vector<glm::mat4> matrices;

	for (int i = 0; i < header.numJoints; i++) {
		const MD5Joint* joint = &joints[i];

		glm::mat4x4 transMat = glm::translate(glm::mat4(1.0f), glm::make_vec3(joint->pos));
		glm::mat4x4 rotMat = glm::toMat4(glm::make_quat(joint->orient));

		glm::mat4x4 matrix = glm::inverse(transMat * rotMat);

		matrices.push_back(matrix);
	}

	return matrices;
}

std::vector<glm::mat4> MD5Anim::bone_matrices(int frame)
{
	std::vector<glm::mat4> matrices;

	const int numJoints = header.numJoints;
	// TODO interpolate between two frameJoints
	const MD5Joint* joints = frameJoints[frame];

	// TODO should ensure model and anim file are compatibles
	for (int i = 0; i < numJoints; i++) {
		const MD5Joint* joint = &joints[i];

		glm::mat4x4 transMat = glm::translate(glm::mat4(1.0f), glm::make_vec3(joint->pos));
		glm::mat4 rotMat = glm::toMat4(glm::make_quat(joint->orient));

		glm::mat4 matrix = transMat * rotMat;
		matrices.push_back(matrix);
	}

	return matrices;
}
