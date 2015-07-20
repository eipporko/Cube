//Fixed-sized-points
#version 400
uniform float n; //Near parameter of the viewing frustum
uniform float f; //Far parameter of the viewing frustum

in  vec3 ex_Color;
out vec4 out_Color;

void main(void)
{	
	out_Color = vec4(ex_Color,1.0);
}