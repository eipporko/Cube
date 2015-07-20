//Fixed-sized-points
#version 400
uniform mat4 viewMatrix, projMatrix;
uniform mat3 normalMatrix;
uniform bool colorEnabled;

in  vec3 in_Position;
in  vec3 in_Color;
in  vec3 in_Normals;

out vec3 ex_Color;

void main(void)
{
	gl_Position = projMatrix * viewMatrix * vec4(in_Position, 1.0);
	gl_PointSize = 2;

	vec3 color = vec3 (0.0, 0.0f, 0.0f);

	//Diffuse
	if (colorEnabled == true) {
		vec3 lightDirection = vec3(0.0,0.0,1.0f);
		float dotValue = max(dot(normalize(normalMatrix * in_Normals), lightDirection), 0.0);
		ex_Color = vec3(dotValue) + color;
	}
	else
		ex_Color = in_Color;
}