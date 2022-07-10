/*
Copyright (c) 2022 Peter Jamieson (jamieson.peter@gmail.com)
and Bryan Van Scoy

Permission is hereby granted, free of charge, to any person
obtaining a copy of this software and associated documentation
files (the "Software"), to deal in the Software without
restriction, including without limitation the rights to use,
copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the
Software is furnished to do so, subject to the following
conditions:

The above copyright notice and this permission notice shall be
included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES
OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
OTHER DEALINGS IN THE SOFTWARE.
*/ 

#include "types.h"

#ifndef COLLISSION_H
#define COLLISSION_H

short circles_collide( circle_t* a,  circle_t* b);
short circle_point_collide( circle_t* c,  vector_2D_t* p);
short circle_line_collide( circle_t* c,  line_t* l);
short circle_segment_collide( circle_t* c,  line_segment_t* s);
short circle_oriented_rectangle_collide( circle_t* c,  oriented_rectangle_t_t* r);
short rectangles_collide( rectangle_t* a,  rectangle_t* b);
short points_collide( vector_2D_t* a,  vector_2D_t* b);
short line_point_collide( line_t* l,  vector_2D_t* p);
short lines_collide( line_t* a,  line_t* b);
short line_segment_collide( line_t* l,  line_segment_t* s);
short point_segment_collide( vector_2D_t* p,  line_segment_t* s);
short segments_collide( line_segment_t* a,  line_segment_t* b);
short line_rectangle_collide( line_t* l,  rectangle_t* r);
short line_oriented_rectangle_collide( line_t* l,  oriented_rectangle_t_t* r);
short oriented_rectangles_collide( oriented_rectangle_t_t* a,  oriented_rectangle_t_t* b);
short point_rectangle_collide( vector_2D_t* p,  rectangle_t* r);
short oriented_rectangle_point_collide( oriented_rectangle_t_t* r,  vector_2D_t* p);
short oriented_rectangle_rectangle_collide( oriented_rectangle_t_t* orect,  rectangle_t* aar);;
short rectangle_segment_collide( rectangle_t* r,  line_segment_t* s);
short oriented_rectangle_segment_collide( oriented_rectangle_t_t* r,  line_segment_t* s);

short overlapping(double minA, double maxA, double minB, double maxB);
range_t project_segment( line_segment_t* s,  vector_2D_t* onto, short ontoIsUnit);
short on_one_side( line_t* axis,  line_segment_t* s);
double clamp_on_range(double x, double min, double max);
vector_2D_t clamp_on_rectangle( vector_2D_t* p,  rectangle_t* r);
short parallel_vectors( vector_2D_t* a,  vector_2D_t* b);
short equal_vectors( vector_2D_t* a,  vector_2D_t* b);
vector_2D_t rectangle_corner( rectangle_t* r, int nr);
vector_2D_t oriented_rectangle_corner( oriented_rectangle_t_t* r, int nr);
line_segment_t oriented_rectangle_edge( oriented_rectangle_t_t* r, int nr);
short separating_axis_for_oriented_rectangle( line_segment_t* axis,  oriented_rectangle_t_t* r);
short separating_axis_for_rectangle( line_segment_t* axis,  rectangle_t* r);
double minimum(double a, double b);
double maximum(double a, double b);
short equal_doubles(double a, double b);
double degrees_to_radian(double degrees);
double radian_to_degrees(double radian);
short overlapping_ranges( range_t* a,  range_t* b);
range_t range_hull( range_t* a,  range_t* b);
range_t sort_range( range_t* r);
vector_2D_t rectangle_center( rectangle_t* r);
vector_2D_t add_vector( vector_2D_t* a,  vector_2D_t* b);
vector_2D_t subtract_vector( vector_2D_t* a,  vector_2D_t* b);
vector_2D_t negate_vector( vector_2D_t* v);
vector_2D_t multiply_vector( vector_2D_t* v, double s);
vector_2D_t divide_vector( vector_2D_t* v, double d);
double dot_product( vector_2D_t* a,  vector_2D_t* b);
double vector_length( vector_2D_t* v);
vector_2D_t rotate_vector( vector_2D_t* v, double degrees);
vector_2D_t rotate_vector_90( vector_2D_t* v);
vector_2D_t rotate_vector_180( vector_2D_t* v);
vector_2D_t rotate_vector_270( vector_2D_t* v);
vector_2D_t unit_vector( vector_2D_t* v);
double enclosed_angle( vector_2D_t* a,  vector_2D_t* b);
vector_2D_t project_vector( vector_2D_t* project,  vector_2D_t* onto);
rectangle_t enlarge_rectangle_point( rectangle_t* r,  vector_2D_t* p);
rectangle_t oriented_rectangle_rectangle_hull( oriented_rectangle_t* r);

#endif
