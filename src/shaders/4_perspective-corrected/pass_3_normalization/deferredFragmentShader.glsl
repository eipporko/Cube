//Perspective Correct Rasterization, Deferred Shading (Normalization Pass)
/*
 *
 * CUBE
 *
 * Copyright (c) David Antunez Gonzalez 2013-2015 <dantunezglez@gmail.com> 
 * Copyright (c) Luis Omar Alvarez Mures 2013-2015 <omar.alvarez@udc.es> 
 * Copyright (c) Emilio Padron Gonzalez 2013-2015 <emilioj@gmail.com> 
 *
 * All rights reserved.
 *
 * This file is part of ToView.
 *
 * CUBE is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public
 * License as published by the Free Software Foundation; either
 * version 3 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU General Public
 * License along with this library.
 *
 */
 
#version 410
uniform mat4 viewMatrix;
uniform float n; //Near parameter of the viewing frustum
uniform float f; //Far parameter of the viewing frustum
uniform float t; //Top parameter of the viewing frustum
uniform float b; //Bottom parameter of the viewing frustum
uniform float r; //Right parameter of the viewing frustum
uniform float l; //Left parameter of the viewing frustum
uniform int h; 	 //Height of the viewport
uniform int w; 	 //Width of the viewport

uniform bool colorEnabled;
uniform int lightCount;
uniform vec3 lightPosition[16];
uniform vec3 lightColor[16];
uniform float lightIntensity[16];

uniform sampler2DRect blendTexture;
uniform sampler2DRect normalTexture;
uniform sampler2DRect positionTexture;

out vec4 out_Color;

void main(void)
{
	//Get Q
	//vec3 qn;
	//qn.x = (gl_FragCoord.x ) *  ((r - l)/w ) - ( (r - l)/2.0 );
	//qn.y = (gl_FragCoord.y ) *  ((b - t)/h ) - ( (b - t)/2.0 );
	//qn.z = -n;

	//vec3 q;
	//q.z = (f * n) / (-gl_FragCoord.z * f + gl_FragCoord.z * n + f);
	//float timef = q.z / qn.z;
	//q.xy = qn.xy * timef;

	vec4 textureColor = texture(blendTexture, gl_FragCoord.xy);

	if (textureColor.a <= 0.0f)
		discard;

	vec4 textureNormal = texture(normalTexture, gl_FragCoord.xy);

	vec4 normalizedColor = vec4(textureColor.rgb/textureColor.a, 1.0f);
	vec4 normalizedNormal = vec4(textureNormal.xyz/textureNormal.w, 1.0f);
	vec3 q = texture(positionTexture, gl_FragCoord.xy).xyz;

	vec3 color = normalizedColor.rgb;
	if (colorEnabled == true)
		color = vec3(0,0,0);

	//Lightning with the resultant normalized textures
	vec3 dotValue = vec3(0,0,0);
	for (int i = 0; i < lightCount; i++) {
		vec3 ccLightPosition = (viewMatrix * vec4(lightPosition[i], 1.0f)).xyz;
		vec3 ligthToQ = normalize(ccLightPosition - q);
		dotValue += vec3(max(dot(normalize(normalizedNormal.xyz), ligthToQ), 0.0)) * lightIntensity[i] * lightColor[i];;
	}

	out_Color = vec4(dotValue + color, 1.0f);
}