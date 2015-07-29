//Perspective Correct Rasterization, Gouraud Shading (Normalization Pass)
#version 410
uniform sampler2DRect blendTexture;

out vec4 out_Color;

void main(void)
{
	vec4 textureColor = texture(blendTexture, gl_FragCoord.xy);

	if (textureColor.a <= 0.0f)
		discard;

	out_Color = vec4(textureColor.rgb/textureColor.a, 1.0f);
}