//Affinely Projected Point Sprites
#version 400
uniform mat4 viewMatrix, projMatrix;
uniform mat3 normalMatrix;
uniform int h; //Height of the viewport
uniform float n; //Near parameter of the viewing frustum
uniform float t; //Top parameter of the viewing frustum
uniform float b; //Bottom parameter of the viewing frustum
uniform float userRadiusFactor; //Splat's radii
uniform bool colorEnabled;
uniform bool automaticRadiusEnabled;

in float in_Radius;
in  vec3 in_Position;
in  vec3 in_Color;
in 	vec3 in_Normals;

out float ex_Radius;
out vec3 ex_Color;
out vec3 ex_Normals;
out float ex_Pz; //z in Camera Coordinates

vec4 ccPosition; //position in Camera Coordinates

void main(void)
{
	ex_Normals = normalize(normalMatrix * in_Normals);

	if (abs(ex_Normals.z) <= 0.1)
		ex_Normals.z = 0.1;

	if (automaticRadiusEnabled == true)
		ex_Radius = in_Radius * userRadiusFactor;
	else
		ex_Radius = userRadiusFactor;

	//p. 277
	ccPosition = viewMatrix * vec4(in_Position, 1.0);
	gl_Position = projMatrix * ccPosition;
	gl_PointSize = 2*ex_Radius * (n / ccPosition.z) * (h / (t-b));

	//BackFace Culling
	if (dot (ccPosition.xyz, ex_Normals) > 0)
		gl_Position.w = 0;

	vec3 color = vec3 (0.0, 0.0f, 0.0f);

	//Diffuse
	if (colorEnabled == true) {
		vec3 lightDirection = vec3(0.0,0.0,1.0f);
		float dotValue = max(dot(normalize(normalMatrix * in_Normals), lightDirection), 0.0);
		ex_Color = vec3(dotValue) + color;
	}
	else
		ex_Color = in_Color;

	ex_Pz = ccPosition.z;
}