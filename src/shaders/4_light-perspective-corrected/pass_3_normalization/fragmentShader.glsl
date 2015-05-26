//Perspective Correct Rasterization
#version 400
uniform float n; //Near parameter of the viewing frustum
uniform float f; //Far parameter of the viewing frustum
uniform float t; //Top parameter of the viewing frustum
uniform float b; //Bottom parameter of the viewing frustum
uniform float r; //Right parameter of the viewing frustum
uniform float l; //Left parameter of the viewing frustum
uniform int h; 	 //Height of the viewport
uniform int w; 	 //Width of the viewport
uniform float radius;

in  vec3 ex_Color;
in 	vec3 ex_UxV;
in  vec3 normals;
in 	vec4 ccPosition;

out vec4 out_Color;

void main(void)
{
	out_Color = vec4(1, 0, 0, 0.5f);
}