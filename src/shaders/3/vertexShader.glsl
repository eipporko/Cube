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

out vec3 ex_Color;
out vec3 ex_UxV;
out vec3 ex_PxV;
out	vec3 ex_UxP;

vec4 ccPosition; //position in Camera Coordinates
vec3 normals;

void main(void)
{
	normals = normalize(normalMatrix * in_Normals);
	ex_PxV = vec3( -normals.y, normals.x, 0);
	ex_UxP = cross(normals, ex_PxV);
	ex_UxV = cross(ex_UxP, ex_PxV);

	//p. 277
	ccPosition = viewMatrix * vec4(in_Position, 1.0);
	gl_Position = projMatrix * ccPosition;
	gl_PointSize = 2*radius * (n / ccPosition.z) * (h / (t-b));

	ex_Color = in_Color;
}