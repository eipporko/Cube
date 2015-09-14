//Affinely Projected Point Sprites
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
 
#version 400
uniform float n; //Near parameter of the viewing frustum
uniform float f; //Far parameter of the viewing frustum

in  vec3 ex_Color;
in 	vec3 ex_Normals;
in float ex_Pz;

out vec4 out_Color;

vec3 test;
float zBuffer;

void main(void)
{
	//p. 278 -- ahorrar sqrt * 2, componentes.... > 1 -> discard
	test.x = gl_PointCoord.x - 0.5;
	test.y = gl_PointCoord.y - 0.5;
	test.z = -(ex_Normals.x/ex_Normals.z) * test.x - (ex_Normals.y/ex_Normals.z) * test.y;
	if (length(test) > 0.5)
		discard;
	
	//p. 279
	zBuffer = (ex_Pz + test.z * 0.1);
	gl_FragDepth = ((1.0 / zBuffer) * ( (f * n) / (f - n) ) + ( f / (f - n) ));
	
	out_Color = vec4(ex_Color,1.0);
}