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

    vec4 totalPosition = vec4(0.0f);

    /*
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
    */

    int bone_idx = int(in_blend_idx[0]);
    totalPosition = bones[bone_idx] * in_blend_weights[0] * vec4(in_position, 1.0f);
    gl_Position =  mvp * totalPosition;

    // o.color = vec4(in_blend_idx.xyz / 5, 1.0f);
    // o.color = vec4(in_blend_idx[0] / 10, 0.0f, 0.0f , 1.0f);

    o.color = vec4(totalPosition.xyz, 1.0f);
}