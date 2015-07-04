//Perspective Correct Rasterization, Gouraud Shading (Visibility Pass)
#version 400
uniform float n; //Near parameter of the viewing frustum
uniform float f; //Far parameter of the viewing frustum
uniform float t; //Top parameter of the viewing frustum
uniform float b; //Bottom parameter of the viewing frustum
uniform float r; //Right parameter of the viewing frustum
uniform float l; //Left parameter of the viewing frustum
uniform int h; 	 //Height of the viewport
uniform int w; 	 //Width of the viewport
uniform float radius;

in  vec3 ex_Color;
in 	vec3 ex_UxV;
in  vec3 normals;
in 	vec4 ccPosition;

out vec4 out_Color;

float LinearizeDepth(float depth)
{
    float near = n; 
    float far = f; 
    float z = depth; // Back to NDC 
    return (2.0 * near) / (far + near - z * (far - near));	
}

void main(void)
{
	//p. 280
	vec3 qn;

	qn.x = (gl_FragCoord.x ) *  ((r - l)/w ) - ( (r - l)/2.0 );
	qn.y = (gl_FragCoord.y ) *  ((b - t)/h ) - ( (b - t)/2.0 );
	qn.z = -n;

	float denom = dot (qn, normals);

	if (denom == 0.0)
		discard;


	//http://en.wikipedia.org/wiki/Line%E2%80%93plane_intersection
	float timef = dot (ccPosition.xyz, normals ) / denom;

	vec3 q = qn * timef;

	vec3 dist = (q - ccPosition.xyz);

	if ((dist.x * dist.x) + (dist.y * dist.y) + (dist.z * dist.z) > pow(radius, 2))
		discard;


	//p. 281
	//lambda = dot ( vec3(ccPosition.x, ccPosition.y, ccPosition.z) , ex_UxV ) / dot( qn, ex_UxV );

	
	//p. 279
	//q = lambda * qn;
	gl_FragDepth = ((1.0 / q.z) * ( (f * n) / (f - n) ) + ( f / (f - n) ));

}