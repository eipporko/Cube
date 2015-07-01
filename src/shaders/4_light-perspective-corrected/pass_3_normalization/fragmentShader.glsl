//Perspective Correct Rasterization
#version 400
uniform sampler2DRect myTexture;

out vec4 out_Color;

void main(void)
{
	vec4 textureColor = texture(myTexture, gl_FragCoord.xy);

	if (textureColor.a <= 0.0f)
		discard;

	out_Color = vec4(textureColor.rgb/textureColor.a, 1.0f);
}