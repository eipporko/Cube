//FXAA Filter
#version 400
uniform sampler2DRect myTexture;
uniform vec3 inverseTextureSize;

out vec4 out_Color;

layout(pixel_center_integer) in vec4 gl_FragCoord;

void main(void)
{
	float R_fxaaSpanMax  = 8.0f;
	float R_fxaaReduceMul = 1.0f/8.0f;
	float R_fxaaReduceMin = 1.0f/128.0f;

	vec3 luma = vec3(0.299, 0.587, 0.114);

	float lumaTL = dot(luma, texture(myTexture, gl_FragCoord.xy + ( vec2(-1.0f, -1.0f) * inverseTextureSize.xy )).xyz);
	float lumaTR = dot(luma, texture(myTexture, gl_FragCoord.xy + ( vec2(1.0f, -1.0f) * inverseTextureSize.xy )).xyz);
	float lumaBL = dot(luma, texture(myTexture, gl_FragCoord.xy + ( vec2(-1.0f, 1.0f) * inverseTextureSize.xy )).xyz);
	float lumaBR = dot(luma, texture(myTexture, gl_FragCoord.xy + ( vec2(1.0f, 1.0f) * inverseTextureSize.xy )).xyz);
	float lumaM = dot(luma, texture(myTexture, gl_FragCoord.xy).xyz);

	vec2 dir;
	dir.x = -( (lumaTL + lumaTR) - (lumaBL + lumaBR) );
	dir.y = ( (lumaTL + lumaBL) - (lumaTR + lumaBR) );

	float dirReduce = max((lumaTL + lumaTR + lumaBL + lumaBR) * (R_fxaaReduceMul * 0.25), R_fxaaReduceMin);
	float inverseDirAdjustment = 1.0/(min(abs(dir.x), abs(dir.y)) + dirReduce);
	
	dir = min(vec2(R_fxaaSpanMax, R_fxaaSpanMax), 
		max(vec2(-R_fxaaSpanMax, -R_fxaaSpanMax), dir * inverseDirAdjustment)) * inverseTextureSize.xy;

	vec3 result1 = (1.0/2.0) * (
		texture(myTexture,  gl_FragCoord.xy + (dir * vec2(1.0/3.0 - 0.5))).xyz +
		texture(myTexture,  gl_FragCoord.xy + (dir * vec2(2.0/3.0 - 0.5))).xyz);

	vec3 result2 = result1 * (1.0/2.0) + (1.0/4.0) * (
		texture(myTexture,  gl_FragCoord.xy + (dir * vec2(0.0/3.0 - 0.5))).xyz +
		texture(myTexture,  gl_FragCoord.xy + (dir * vec2(3.0/3.0 - 0.5))).xyz);

	float lumaMin = min(lumaM, min(min(lumaTL, lumaTR), min(lumaBL, lumaBR)));
	float lumaMax = max(lumaM, max(max(lumaTL, lumaTR), max(lumaBL, lumaBR)));
	float lumaResult2 = dot(luma, result2);
	
	if(lumaResult2 < lumaMin || lumaResult2 > lumaMax)
		out_Color = vec4(result1, 1.0);
	else
		out_Color = vec4(result2, 1.0);
}