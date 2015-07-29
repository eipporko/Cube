//Perspective Correct Rasterization, Deferred Shading (Normalization Pass)
#version 410
uniform sampler2DRect blendTexture;
uniform sampler2DRect normalTexture;

uniform bool colorEnabled;

out vec4 out_Color;

void main(void)
{
	vec4 textureColor = texture(blendTexture, gl_FragCoord.xy);

	if (textureColor.a <= 0.0f)
		discard;

	vec4 textureNormal = texture(normalTexture, gl_FragCoord.xy);

	vec4 normalizedColor = vec4(textureColor.rgb/textureColor.a, 1.0f);
	vec4 normalizedNormal = vec4(textureNormal.xyz/textureNormal.w, 1.0f);

	//Lightning with the resultant normalized textures
	if (colorEnabled == true) {
		vec3 lightDirection = vec3(0.0,0.0,1.0f);
		float dotValue = max(dot(normalize(normalizedNormal.xyz), lightDirection), 0.0);
		out_Color = vec4(vec3(dotValue) + normalizedColor.rgb, 1.0f);
	}
	else
		out_Color = normalizedColor;
}