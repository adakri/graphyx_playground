#version 460 core
// Standard frag shader with transparency
in vec3 fColor;

out vec4 FragColor;

void main()
{
    FragColor = vec4(fColor, 0.2);
}