//Perspective Correct Rasterization
#version 400
uniform mat4 viewMatrix, projMatrix;
uniform mat3 normalMatrix;
uniform int h; //Height of the viewport
uniform float n; //Near parameter of the viewing frustum
uniform float t; //Top parameter of the viewing frustum
uniform float b; //Bottom parameter of the viewing frustum
uniform float radius; //Splat's radii

in  vec3 in_Position;
in  vec3 in_Color;
in 	vec3 in_Normals;

out vec4 ccPosition; //position in Camera Coordinates
out vec3 normals;


void main(void)
{
	gl_Position = vec4(in_Position, 1.0);
}