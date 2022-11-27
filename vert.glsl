#version 330 core

layout (location = 0) in vec3 in_position;
layout (location = 1) in vec2 in_uv;
layout (location = 2) in vec4 in_blend_idx;
layout (location = 3) in vec4 in_blend_weights;

out VertexData {
    vec4 color;
} o;

const int MAX_BONES = 100;
const int MAX_BONE_INFLUENCE = 4;

uniform mat4 mvp;
uniform mat4 bones[MAX_BONES];

void main()
{
    /*
    vec4 totalPosition = vec4(0.0f);

    for(int i = 0 ; i < MAX_BONE_INFLUENCE ; i++)
    {
        int bone_idx = int(in_blend_idx[i]);

        if(bone_idx < 0)
            continue;

        if(bone_idx >= MAX_BONES) {
            totalPosition = vec4(0.0f);
            break;
        }

        vec4 localPosition = bones[bone_idx] * vec4(in_position, 1.0f);
        totalPosition += localPosition * in_blend_weights[i];
    }

    gl_Position =  mvp * totalPosition;
    */

    
    mat4 transform  = bones[int(in_blend_idx.x)] * in_blend_weights.x;
    transform += bones[int(in_blend_idx.y)] * in_blend_weights.y;
    transform += bones[int(in_blend_idx.z)] * in_blend_weights.z;

    // Ensure sum of all weights amount to one
    float final_weight = 1.0f - (in_blend_weights.x + in_blend_weights.y + in_blend_weights.z);
    transform += bones[int(in_blend_idx.w)] * final_weight;

    vec4 position = transform * vec4(in_position, 1.0);

    gl_Position =  mvp * position;

    o.color = vec4(1.0f, 0.0f, 1.0f, 1.0f);
}