//Affinely Projected Point Sprites
#version 400
uniform float n; //Near parameter of the viewing frustum
uniform float f; //Far parameter of the viewing frustum

in  vec3 ex_Color;
in 	vec3 ex_Normals;
in float ex_Pz;

out vec4 out_Color;

vec3 test;
float zBuffer;

void main(void)
{
	//p. 278 -- ahorrar sqrt * 2, componentes.... > 1 -> discard
	test.x = gl_PointCoord.x - 0.5;
	test.y = gl_PointCoord.y - 0.5;
	test.z = -(ex_Normals.x/ex_Normals.z) * test.x - (ex_Normals.y/ex_Normals.z) * test.y;
	if (length(test) > 0.5)
		discard;
	
	//p. 279
	zBuffer = (ex_Pz + test.z);
	gl_FragDepth = ((1.0 / zBuffer) * ( (f * n) / (f - n) ) + ( f / (f - n) ));
	gl_FragDepth = gl_FragDepth;
	
	out_Color = vec4(ex_Color,1.0);
}