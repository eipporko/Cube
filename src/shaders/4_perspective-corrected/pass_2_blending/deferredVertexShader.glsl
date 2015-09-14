//Perspective Correct Rasterization, Gouraud Shading (Blending Pass)
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
uniform mat4 viewMatrix, projMatrix;
uniform mat3 normalMatrix;
uniform int h; //Height of the viewport
uniform float n; //Near parameter of the viewing frustum
uniform float t; //Top parameter of the viewing frustum
uniform float b; //Bottom parameter of the viewing frustum
uniform float userRadiusFactor; //Splat's radii
uniform bool automaticRadiusEnabled;

in float in_Radius;
in  vec3 in_Position;
in  vec3 in_Color;
in 	vec3 in_Normals;

out vec3 ex_Color;
out float ex_Radius;

out vec4 ccPosition; //position in Camera Coordinates
out vec3 normals;


void main(void)
{
	normals = normalize(normalMatrix * in_Normals);

	if (automaticRadiusEnabled == true)
		ex_Radius = in_Radius * userRadiusFactor;
	else
		ex_Radius = userRadiusFactor;

	//p. 277
	ccPosition = viewMatrix * vec4(in_Position, 1.0);
	gl_Position = projMatrix * ccPosition;
	gl_PointSize = 2*ex_Radius * (n / ccPosition.z) * (h / (t-b));

	//Backface Culling
	if (dot (ccPosition.xyz, normals) > 0)
		gl_Position.w = 0;


	ex_Color = in_Color;
}