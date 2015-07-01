//Perspective Correct Rasterization
#version 400
in  vec3 in_Position;
in 	vec3 in_Color;

out vec4 out_Color;

void main(void)
{
	gl_Position = vec4(in_Position, 1.0);
}