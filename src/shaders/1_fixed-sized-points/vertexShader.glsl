//Fixed-sized-points
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
uniform mat4 viewMatrix, projMatrix;
uniform mat3 normalMatrix;
uniform bool colorEnabled;

in  vec3 in_Position;
in  vec3 in_Color;
in  vec3 in_Normals;

out vec3 ex_Color;

void main(void)
{
	gl_Position = projMatrix * viewMatrix * vec4(in_Position, 1.0);
	gl_PointSize = 2;

	vec3 color = vec3 (0.0, 0.0f, 0.0f);

	//Diffuse
	if (colorEnabled == true) {
		vec3 lightDirection = vec3(0.0,0.0,1.0f);
		float dotValue = max(dot(normalize(normalMatrix * in_Normals), lightDirection), 0.0);
		ex_Color = vec3(dotValue) + color;
	}
	else
		ex_Color = in_Color;
}