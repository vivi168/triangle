#version 330 core

#define MAX_BONES 100

layout (location = 0) in vec3 in_position;
layout (location = 1) in vec2 in_uv;
layout (location = 2) in vec4 in_blend_idx;
layout (location = 3) in vec4 in_blend_weights;

out VertexData {
    vec2 texture_uv;
} o;

layout (std140) uniform PassUBO {
    mat4 pv;
};

layout (std140) uniform ModelUBO {
    mat4 model;
};

layout (std140) uniform SkinnedUBO {
    mat4 bones[MAX_BONES];
};

void main()
{
    mat4 transform  = bones[int(in_blend_idx.x)] * in_blend_weights.x;
    transform += bones[int(in_blend_idx.y)] * in_blend_weights.y;
    transform += bones[int(in_blend_idx.z)] * in_blend_weights.z;

    // Ensure the sum of all weights amounts to one
    float weight_w = 1.0f - (in_blend_weights.x + in_blend_weights.y + in_blend_weights.z);
    transform += bones[int(in_blend_idx.w)] * weight_w;

    vec4 position = transform * vec4(in_position, 1.0);

    gl_Position =  pv * model * position;

    o.texture_uv = in_uv;
}
