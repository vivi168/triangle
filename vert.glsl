#version 330 core

layout (location = 0) in vec3 in_position;
layout (location = 1) in vec2 in_uv;

out VertexData {
    vec4 color;
} o;

uniform mat4 mvp;

void main()
{
    gl_Position = mvp * vec4(in_position, 1.0);

    o.color = vec4(in_uv.x, 0.0f, in_uv.y, 1.0f);
}
