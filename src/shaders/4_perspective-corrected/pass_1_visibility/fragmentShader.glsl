//Perspective Correct Rasterization, Gouraud Shading (Visibility Pass)
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
uniform float n; //Near parameter of the viewing frustum
uniform float f; //Far parameter of the viewing frustum
uniform float t; //Top parameter of the viewing frustum
uniform float b; //Bottom parameter of the viewing frustum
uniform float r; //Right parameter of the viewing frustum
uniform float l; //Left parameter of the viewing frustum
uniform int h; 	 //Height of the viewport
uniform int w; 	 //Width of the viewport

in float ex_Radius;
in  vec3 normals;
in 	vec4 ccPosition;

layout (location = 1) out vec3 out_Position;

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

	if ((dist.x * dist.x) + (dist.y * dist.y) + (dist.z * dist.z) > pow(ex_Radius, 2))
		discard;

	
	out_Position = q;
	//p. 279
	gl_FragDepth = ((1.0 / q.z) * ( (f * n) / (f - n) ) + ( f / (f - n) ));

}