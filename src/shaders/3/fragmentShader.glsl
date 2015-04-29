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

in  vec3 ex_Color;
in 	vec3 ex_UxV;
in 	vec3 ex_PxV;
in 	vec3 ex_UxP;
out vec4 out_Color;

vec3 qn, q;
float zBuffer, lambda, u, v;

void main(void)
{
	//p. 280
	qn.x = (gl_PointCoord.x - 0.5) * ( (r - l)/w ) - ( (r - l)/2.0 );
	qn.y = (-1.0*(gl_PointCoord.y - 0.5)) * ( (t - b)/h ) - ( (t - b)/2.0 );
	qn.z = -n;

	//p. 281
	lambda = dot ( vec3(gl_FragCoord), ex_UxV ) / dot( qn, ex_UxV );
	u = dot ( qn, ex_PxV ) / dot ( qn, ex_UxV );
	v = dot ( qn, ex_UxP ) / dot ( qn, ex_UxV );

	//if (pow(u,2) + pow(v,2) > 1.0)
	//	discard;
	
	//p. 279
	//q = lambda * qn;
	//gl_FragDepth = ((1.0 / q.z) * ( (f * n) / (f - n) ) + ( f / (f - n) ));
	
	out_Color = vec4(ex_Color,1.0);
}