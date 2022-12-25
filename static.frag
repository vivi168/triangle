#version 330 core

out vec4 out_color;

in VertexData {
    vec4 color;
} i;

void main()
{
    out_color = i.color;
}