//Image-aligned Squares
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

vec4 ccPosition; //position in Camera Coordinates

void main(void)
{
	//p. 277
	ccPosition = viewMatrix * vec4(in_Position, 1.0);
	gl_Position = projMatrix * ccPosition;
	gl_PointSize = 2*radius * (n / ccPosition.z) * (h / (t-b));

	//Diffuse
	vec3 lightDirection = vec3(0.0,0.0,0.5);
	float dotValue = max(dot(normalize(normalMatrix * in_Normals), lightDirection), 0.0);
	ex_Color = vec3(dotValue) + in_Color;
	
	//ex_Color = in_Color;
}