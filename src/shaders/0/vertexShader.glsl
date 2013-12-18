//Image-aligned Squares
#version 400
uniform mat4 viewMatrix, projMatrix;
uniform float r; //Splat's radii

in  vec3 in_Position;
in  vec3 in_Color;
out vec3 ex_Color;

void main(void)
{
	gl_Position = projMatrix * viewMatrix * vec4(in_Position, 1.0);
	gl_PointSize = 2;
	ex_Color = in_Color;
}