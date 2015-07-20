//Perspective Correct Rasterization, Gouraud Shading (Visibility Pass)
#version 400
uniform mat4 viewMatrix, projMatrix;
uniform mat3 normalMatrix;
uniform int h; //Height of the viewport
uniform float n; //Near parameter of the viewing frustum
uniform float t; //Top parameter of the viewing frustum
uniform float b; //Bottom parameter of the viewing frustum
uniform float userRadiusFactor; //Splat's radii
uniform bool automaticRadiusEnabled;

in  vec3 in_Position;
in  vec3 in_Color;
in 	vec3 in_Normals;
in  float in_Radius;

out vec3 ex_Color;
out float ex_Radius;

out vec4 ccPosition; //position in Camera Coordinates
out vec3 normals;


void main(void)
{
	if (automaticRadiusEnabled == true)
		ex_Radius = in_Radius * userRadiusFactor;
	else
		ex_Radius = userRadiusFactor;

	normals = normalize(normalMatrix * in_Normals);


	//p. 277
	ccPosition = viewMatrix * vec4(in_Position, 1.0);
	gl_Position = projMatrix * ccPosition;
	gl_PointSize = 2 * ex_Radius * (n / ccPosition.z) * (h / (t-b));
}