#version 330 core

in VertexData {
    vec4 color;
} i;

out vec4 out_color;

void main()
{
    // out_color = vec4(1.0f, 0.0f, 1.0f, 1);

    out_color = i.color;
}
