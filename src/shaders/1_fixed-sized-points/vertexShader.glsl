//Image-aligned Squares
#version 400
uniform mat4 viewMatrix, projMatrix;
uniform mat3 normalMatrix;

in  vec3 in_Position;
in  vec3 in_Color;
in  vec3 in_Normals;

out vec3 ex_Color;

void main(void)
{
	gl_Position = projMatrix * viewMatrix * vec4(in_Position, 1.0);
	gl_PointSize = 2;

	//Diffuse
	vec3 lightDirection = vec3(0.0,0.0,0.5);
	float dotValue = max(dot(normalize(normalMatrix * in_Normals), lightDirection), 0.0);
	ex_Color = vec3(dotValue) + in_Color;

	//ex_Color = in_Color;
}