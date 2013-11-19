#version 400
uniform mat4 viewMatrix, projMatrix;
uniform int h; //Height of the viewport
uniform float n; //Near parameter of the viewing frustum
uniform float t; //Top parameter of the viewing frustum
uniform float b; //Bottom parameter of the viewing frustum
uniform float r; //Splat's radii

in  vec3 in_Position;
in  vec3 in_Color;
out vec3 ex_Color;

vec4 ccPosition; //position in Camera Coordinates

void main(void)
{
	ex_Color = in_Color;
	ccPosition = viewMatrix * vec4(in_Position, 1.0);
	gl_Position = projMatrix * ccPosition;
	gl_PointSize = 2*r * (n / ccPosition.z) * (h / (t-b));
}