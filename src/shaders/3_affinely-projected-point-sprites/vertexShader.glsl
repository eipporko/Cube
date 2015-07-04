//Affinely Projected Point Sprites
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
out vec3 ex_Normals;
out float ex_Pz; //z in Camera Coordinates

vec4 ccPosition; //position in Camera Coordinates

void main(void)
{
	ex_Normals = normalize(normalMatrix * in_Normals);

	if (abs(ex_Normals.z) <= 0.1)
		ex_Normals.z = 0.1;

	//p. 277
	ccPosition = viewMatrix * vec4(in_Position, 1.0);
	gl_Position = projMatrix * ccPosition;
	gl_PointSize = 2*radius * (n / ccPosition.z) * (h / (t-b));

	//Diffuse
	vec3 lightDirection = vec3(0.0,0.0,0.5);
	float dotValue = max(dot(ex_Normals, lightDirection), 0.0);
	ex_Color = vec3(dotValue) + in_Color;

	ex_Pz = ccPosition.z;
}