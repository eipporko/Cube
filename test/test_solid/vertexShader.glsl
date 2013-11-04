#version 400
uniform mat4 viewMatrix, projMatrix;

in  vec3 in_Position;
in  vec3 in_Color;
out vec3 ex_Color;

void main(void)
{
	ex_Color = in_Color;
	gl_Position = projMatrix * viewMatrix * vec4(in_Position, 1.0);
    //gl_Position = vec4(in_Position, 1.0);

}