//Perspective Correct Rasterization, Deferred Shading (Normalization Pass)
#version 410
uniform float n; //Near parameter of the viewing frustum
uniform float f; //Far parameter of the viewing frustum
uniform float t; //Top parameter of the viewing frustum
uniform float b; //Bottom parameter of the viewing frustum
uniform float r; //Right parameter of the viewing frustum
uniform float l; //Left parameter of the viewing frustum
uniform int h; 	 //Height of the viewport
uniform int w; 	 //Width of the viewport

uniform sampler2DRect blendTexture;
uniform sampler2DRect normalTexture;

uniform bool colorEnabled;

out vec4 out_Color;

void main(void)
{
	//Get Q
	vec3 qn;
	qn.x = (gl_FragCoord.x ) *  ((r - l)/w ) - ( (r - l)/2.0 );
	qn.y = (gl_FragCoord.y ) *  ((b - t)/h ) - ( (b - t)/2.0 );
	qn.z = -n;

	vec3 q;
	q.z = (f * n) / (-gl_FragCoord.z * f + gl_FragCoord.z * n + f);
	float timef = q.z / qn.z;
	q.xy = qn.xy * timef;

	vec4 textureColor = texture(blendTexture, gl_FragCoord.xy);

	if (textureColor.a <= 0.0f)
		discard;

	vec4 textureNormal = texture(normalTexture, gl_FragCoord.xy);

	vec4 normalizedColor = vec4(textureColor.rgb/textureColor.a, 1.0f);
	vec4 normalizedNormal = vec4(textureNormal.xyz/textureNormal.w, 1.0f);

	//Lightning with the resultant normalized textures
	if (colorEnabled == true) {
		vec3 lightPosition = vec3(0.0, 0.0, 1.0f);
		vec3 ligthToQ = normalize(lightPosition - q);
		float dotValue = max(dot(normalize(normalizedNormal.xyz), ligthToQ), 0.0);
		out_Color = vec4(vec3(dotValue) + normalizedColor.rgb, 1.0f);
	}
	else
		out_Color = vec4(q, 1.0f);
		//out_Color = normalizedColor;
}