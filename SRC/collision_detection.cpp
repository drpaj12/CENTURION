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
#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <string.h>
#include <math.h>

#include "globals.h"
#include "types.h"
#include "utils.h"

#include "collision_detection.h"

/* Collision detection functions taken code from:
 * 2d Game Collision Detection by Schwarzl */

#if 0
/*---------------------------------------------------------------------------------------------
 * (function: add_vector)
 *-------------------------------------------------------------------------------------------*/
vector_2D_t add_vector(vector_2D_t a, vector_2D_t b)
{
	vector_2D_t r;

	r.x = a.x + b.x;
	r.y = a.y + b.y;

	return r;
}
/*---------------------------------------------------------------------------------------------
 * (function: sub_vector)
 *-------------------------------------------------------------------------------------------*/
vector_2D_t subtract_vector(vector_2D_t a, vector_2D_t b)
{
	vector_2D_t r;
	r.x = a.x - b.x;
	r.y = a.y - b.y;
	return r;
}
/*---------------------------------------------------------------------------------------------
 * (function: negate_vector)
 *-------------------------------------------------------------------------------------------*/
vector_2D_t negate_vector(vector_2D_t v)
{
	vector_2D_t n;
	n.x = -v.x;
	n.y = -v.y;
	return n;
}

/*---------------------------------------------------------------------------------------------
 * (function: equal_doubles)
 * Rounding errors
 *-------------------------------------------------------------------------------------------*/
short equal_doubles(double a, double b)
{
	double threshold = 1.0f / 8192.0f;
	return fabsf(a - b) < threshold;
}
/*---------------------------------------------------------------------------------------------
 * (function: equal_vectors)
 *-------------------------------------------------------------------------------------------*/
short equal_vectors(vector_2D_t a, vector_2D_t b)
{
	if (equal_doubles(a.x, b.x) && equal_doubles(a.y, b.y)) 
		return TRUE;
	else
		return FALSE;
}

/*---------------------------------------------------------------------------------------------
 * (function: multiply_vector)
 *-------------------------------------------------------------------------------------------*/
vector_2D_t multiply_vector(vector_2D_t v, double scalar)
{
	vector_2D_t r;
	r.x = v.x * scalar;
	r.y = v.y * scalar;
	return r;
}

/*---------------------------------------------------------------------------------------------
 * (function: divide_vector)
 *-------------------------------------------------------------------------------------------*/
vector_2D_t divide_vector(vector_2D_t v, double divisor)
{
	vector_2D_t r;
	r.x = v.x / divisor;
	r.y = v.y / divisor;
	return r;
}

/*---------------------------------------------------------------------------------------------
 * (function: vector_length)
 * Cartesian Distance from origin
 *-------------------------------------------------------------------------------------------*/
double vector_length(vector_2D_t a)	
{
	return sqrt( a.x * a.x + a.y * a.y );
}

/*---------------------------------------------------------------------------------------------
 * (function: unit_vector)
 *-------------------------------------------------------------------------------------------*/
vector_2D_t unit_vector(vector_2D_t v)
{
	double length = vector_length(v);

	if(length > 0)
	{
		return divide_vector(v, length);
	}

	return v;
}

/*---------------------------------------------------------------------------------------------
 * (function: degrees_to_radian)
 *-------------------------------------------------------------------------------------------*/
double degrees_to_radian(double degrees)
{
	return degrees * PI / 180.0f;
}
/*---------------------------------------------------------------------------------------------
 * (function: radian_to_degrees)
 *-------------------------------------------------------------------------------------------*/
double radian_to_degrees(double radians)
{
	return radians * (180.0f / PI);
}

/*---------------------------------------------------------------------------------------------
 * (function: rotate_vector_degrees)
 *-------------------------------------------------------------------------------------------*/
vector_2D_t rotate_vector_degrees(vector_2D_t v, double degrees)
{
	double radian = degrees_to_radian(degrees);
	double sine = sin(radian);
	double cosine = cos(radian);
	vector_2D_t r;

	r.x = v.x * cosine - v.y * sine;
	r.y = v.x * sine + v.y * cosine;

	return r;
}

/*---------------------------------------------------------------------------------------------
 * (function: rotate_vector_radians)
 *-------------------------------------------------------------------------------------------*/
vector_2D_t rotate_vector_radians(vector_2D_t v, double radians)
{
	double sine = sin(radians);
	double cosine = cos(radians);
	vector_2D_t r;

	r.x = v.x * cosine - v.y * sine;
	r.y = v.x * sine + v.y * cosine;

	return r;
}

/*---------------------------------------------------------------------------------------------
 * (function: dot_product)
 *-------------------------------------------------------------------------------------------*/
double dot_product(vector_2D_t a, vector_2D_t b)
{
	return a.x * b.x + a.y * b.y;
}

/*---------------------------------------------------------------------------------------------
 * (function: enclosed_angle)
 * Returns the angle in degrees of the angle between two vectors
 *-------------------------------------------------------------------------------------------*/
double enclosed_angle_degrees(vector_2D_t a, vector_2D_t b)
{
	vector_2D_t ua = unit_vector(a);
	vector_2D_t ub = unit_vector(b);

	double dp = dot_product(ua, ub);

	return radian_to_degrees(acos(dp));
}
double enclosed_angle_radians(vector_2D_t a, vector_2D_t b)
{
	vector_2D_t ua = unit_vector(a);
	vector_2D_t ub = unit_vector(b);

	double dp = dot_product(ua, ub);

	return acos(dp);
}

/*---------------------------------------------------------------------------------------------
 * (function: project_vector)
 *
 *   project
 *   /
 *  /
 * /
 * -------------------- onto
 *
 * --- projection 
 *-------------------------------------------------------------------------------------------*/
vector_2D_t project_vector(vector_2D_t project, vector_2D_t onto)
{
	double dot_prod = dot_product(onto, onto);

	if(dot_prod > 0)
	{
		double dp = dot_product(project, onto);
		return multiply_vector(onto, dp / dot_prod);
	}

	return onto;
}

/*---------------------------------------------------------------------------------------------
 * (function: overlapping )
 *-------------------------------------------------------------------------------------------*/
short overlapping(double minA, double maxA, double minB, double maxB)
{
	return minB <= maxA && minA <= maxB;
}

/*---------------------------------------------------------------------------------------------
 * (function: )
 *-------------------------------------------------------------------------------------------*/
short rectangles_collide(rectangle_t a, rectangle_t b)
{
	double aLeft = a.origin.x;
	double aRight = aLeft + a.size.x;
	double aBottom = a.origin.y;
	double aTop = aBottom + a.size.y;

	double bBottom = b.origin.y;
	double bTop = bBottom + b.size.y;
	double bLeft = b.origin.x;
	double bRight = bLeft + b.size.x;
	
	return (
			overlapping(aLeft, aRight, bLeft, bRight) 
			&& overlapping(aBottom, aTop, bBottom, bTop)
	       );
}

/*---------------------------------------------------------------------------------------------
 * (function: circles_collide)
 *-------------------------------------------------------------------------------------------*/
short circles_collide(circle_t a, circle_t b)
{
	double radiusSum = a.radius + b.radius;

	vector_2D_t distance = subtract_vector(a.center, b.center);

	return (vector_length(distance) <= radiusSum);
}

/*---------------------------------------------------------------------------------------------
 * (function: points_collide)
 *-------------------------------------------------------------------------------------------*/
short points_collide(vector_2D_t a, vector_2D_t b)
{
	return equal_doubles(a.x, b.x) && equal_doubles(a.y, b.y);
}

/*---------------------------------------------------------------------------------------------
 * (function: rotate_vector_90)
 *-------------------------------------------------------------------------------------------*/
vector_2D_t rotate_vector_90(vector_2D_t v)
{
	vector_2D_t r;
	r.x = -v.y;
	r.y = v.x;
	return r;
}

/*---------------------------------------------------------------------------------------------
 * (function: parallel_vectors)
 *-------------------------------------------------------------------------------------------*/
short parallel_vectors(vector_2D_t a, vector_2D_t b)
{
	vector_2D_t na = rotate_vector_90(a);

	return equal_doubles(0, dot_product(na, b));
}

/*---------------------------------------------------------------------------------------------
 * (function: equaivalent_lines)
 *   Same line defined differently
 *-------------------------------------------------------------------------------------------*/
short equivalent_lines(line_t a, line_t b)
{
	if(!parallel_vectors(a.direction, b.direction))
		return FALSE;
	
	vector_2D_t d = subtract_vector(a.base, b.base);
	return parallel_vectors(d, a.direction);
}

/*---------------------------------------------------------------------------------------------
 * (function: lines_collide)
 * 
 * As defined above only non-parallel or equivalent lines collide. Function lines_collide() first checks if the lines are parallel. If they are it returns their equivalence. Otherwise the lines collide so yes gets returned.
 * Function parallel_vectors() uses a simple trick to test vectors for parallelism. We know from section Dot Product that two vectors enclose a right angle when their dot product is zero. So when two vectors are parallel we just need to rotate one of them by 90° and check their dot product. That's exactly what parallel_vectors() does.
 * Instead of utilizing rotate_vector(), which takes arbitrary angles, we use the specialized function rotate_vector_90(). It's faster due to a simple trick which is explained in section Rotating by Right Angles.
 * A prerequisite for explaining equivalent_lines() is to know what equivalence means in this regard. There's a slight difference between equal and equivalent. Equal lines have the same base point and the same direction. One could be the clone of the other. Equivalent lines, on the other hand, just need parallel direction vectors and must have their base point somewhere on the other line. So line base and direction can be different from the other line's base and direction. Only the outcome, the drawn lines if you will, must be identical.
 * Function equivalent_lines() first tests for parallelism. If the lines are parallel we need to find out if the base point of one line lies on the other line. This is the case if the distance vector between the two base points is parallel to any of the lines.
 *-------------------------------------------------------------------------------------------*/
short lines_collide(line_t a, line_t b)
{
	if(parallel_vectors(a.direction, b.direction))
		return equivalent_lines(a, b);
	else
		return TRUE;
}

/*---------------------------------------------------------------------------------------------
 * (function: on_one_side)
 *-------------------------------------------------------------------------------------------*/
short on_one_side(line_t axis, line_segment_t s)
{
	vector_2D_t d1 = subtract_vector(s.point1, axis.base);
	vector_2D_t d2 = subtract_vector(s.point2, axis.base);
	vector_2D_t n = rotate_vector_90(axis.direction);

	if (dot_product(n, d1) * dot_product(n, d2) > 0)
		return TRUE;
	else
		return FALSE;
}

/*---------------------------------------------------------------------------------------------
 * (function: sort_range)
 *-------------------------------------------------------------------------------------------*/
range_t sort_range(range_t r)
{
	range_t sorted = r;

	if(r.minimum > r.maximum)
	{
		sorted.minimum = r.maximum;
		sorted.maximum = r.minimum;
	}

	return sorted;
}

/*---------------------------------------------------------------------------------------------
 * (function: project_segment)
 *-------------------------------------------------------------------------------------------*/
range_t project_segment(line_segment_t s, vector_2D_t onto)
{
	range_t r;
	vector_2D_t ontoUnit = unit_vector(onto);

	r.minimum = dot_product(ontoUnit, s.point1);
	r.maximum = dot_product(ontoUnit, s.point2);

	r = sort_range(r);

	return r;
}

/*---------------------------------------------------------------------------------------------
 * (function: overlapping_ranges)
 *-------------------------------------------------------------------------------------------*/
short overlapping_ranges(range_t a, range_t b)
{
	return overlapping(a.minimum, a.maximum, b.minimum, b.maximum);
}

/*---------------------------------------------------------------------------------------------
 * (function: segments_collide)
 *-------------------------------------------------------------------------------------------*/
short segments_collide(line_segment_t a, line_segment_t b)
{
	line_t axisA, axisB;
	axisA.base = a.point1;
	axisA.direction = subtract_vector(a.point2, a.point1);

	if(on_one_side(axisA, b))
		return FALSE;

	axisB.base = b.point1;
	axisB.direction = subtract_vector(b.point2, b.point1);

	if(on_one_side(axisB, a))
		return FALSE;
	
	if(parallel_vectors(axisA.direction, axisB.direction))
	{
		range_t rangeA = project_segment(a, axisA.direction);
		range_t rangeB = project_segment(b, axisA.direction);
		return overlapping_ranges(rangeA, rangeB);
	}
	else
		return TRUE;
}

/*---------------------------------------------------------------------------------------------
 * (function: range_hull)
 *-------------------------------------------------------------------------------------------*/
range_t range_hull(range_t a, range_t b)
{
	range_t hull;

	hull.minimum = a.minimum < b.minimum ? a.minimum : b.minimum;
	hull.maximum = a.maximum > b.maximum ? a.maximum : b.maximum;

	return hull;
}

/*---------------------------------------------------------------------------------------------
 * (function: oriented_rectangle_edge)
 *-------------------------------------------------------------------------------------------*/
line_segment_t oriented_rectangle_edge(oriented_rectangle_t r, int nr)
{
	line_segment_t edge;

	vector_2D_t a = r.half_extend;
	vector_2D_t b = r.half_extend;
	
	switch(nr % 4)
	{
		case 0:
			a.x = -a.x;
			break;
		case 1:
			b.y = -b.y;
			break;
		case 2:
			a.y = -a.y;
			b = negate_vector(b);
			break;
		default:
			a = negate_vector(a);
			b.x = -b.x;
			break;
	}
	
	a = rotate_vector(a, r.rotation);
	a = add_vector(a, r.center);
	b = rotate_vector(b, r.rotation);
	b = add_vector(b, r.center);
	
	edge.point1 = a;
	edge.point2 = b;
	return edge;
}

/*---------------------------------------------------------------------------------------------
 * (function: separating_axis_for_oriented_rectangle)
 *-------------------------------------------------------------------------------------------*/
short separating_axis_for_oriented_rectangle( line_segment_t axis, oriented_rectangle_t r)
{
	range_t axisrange_t, r0range_t, r2range_t, rProjection;
	line_segment_t rEdge0 = oriented_rectangle_edge(r, 0);
	line_segment_t rEdge2 = oriented_rectangle_edge(r, 2);
	vector_2D_t n = subtract_vector(axis.point1, axis.point2);
	axisrange_t = project_segment(axis, n);
	r0range_t = project_segment(rEdge0, n);
	r2range_t = project_segment(rEdge2, n);
	rProjection = range_hull(r0range_t, r2range_t);
	
	return !overlapping_ranges(axisrange_t, rProjection);
}

/*---------------------------------------------------------------------------------------------
 * (function: oriented_rectangles_collide)
 *-------------------------------------------------------------------------------------------*/
short oriented_rectangles_collide( oriented_rectangle_t a, oriented_rectangle_t b)
{
	line_segment_t edge = oriented_rectangle_edge(a, 0);
	if(separating_axis_for_oriented_rectangle(edge, b))
		return FALSE;
	edge = oriented_rectangle_edge(a, 1);

	if(separating_axis_for_oriented_rectangle(edge, b))
		return FALSE;
	edge = oriented_rectangle_edge(b, 0);

	if(separating_axis_for_oriented_rectangle(edge, a))
		return FALSE;
	edge = oriented_rectangle_edge(b, 1);
	return !separating_axis_for_oriented_rectangle(edge, a);
}

/*---------------------------------------------------------------------------------------------
 * (function: circle_point_collide)
 *-------------------------------------------------------------------------------------------*/
short circle_point_collide(circle_t c, vector_2D_t p)
{
	vector_2D_t distance = subtract_vector(c.center, p);
	return vector_length(distance) <= c.radius;
}

/*---------------------------------------------------------------------------------------------
 * (function: circle_line_collide)
 *-------------------------------------------------------------------------------------------*/
short circle_line_collide(circle_t c, line_t l)
{
	vector_2D_t lc = subtract_vector(c.center, l.base);
	vector_2D_t p = project_vector(lc, l.direction);
	vector_2D_t nearest = add_vector(l.base, p);

	return circle_point_collide(c, nearest);
}

/*---------------------------------------------------------------------------------------------
 * (function: circle_segment_collide)
 *-------------------------------------------------------------------------------------------*/
short circle_segment_collide(circle_t c, line_segment_t s)
{
	if(circle_point_collide(c, s.point1))
		return yes;
	if(circle_point_collide(c, s.point2))
		return yes;
	
	vector_2D_t d = subtract_vector(s.point2, s.point1);
	vector_2D_t lc = subtract_vector(c.center, s.point1);
	vector_2D_t p = project_vector(lc, d);
	vector_2D_t nearest = add_vector(s.point1, p);
	
	return ( 
		circle_point_collide(c, nearest)
		&& vector_length(p) <= vector_length(d)
		&& 0 <= dot_product(p, d)
	       );
}

/*---------------------------------------------------------------------------------------------
 * (function: clamp_on_range)
 *-------------------------------------------------------------------------------------------*/
double clamp_on_range(double x, double min, double max)
{
	if(x < min)
		return min;
	else if(max < x)
		return max;
	else
		return x;
}

/*---------------------------------------------------------------------------------------------
 * (function: clamp_on_rectangle)
 *-------------------------------------------------------------------------------------------*/
vector_2D_t clamp_on_rectangle(vector_2D_t p, rectangle_t r)
{
	vector_2D_t clamp;

	clamp.x = clamp_on_range(p.x, r.origin.x, r.origin.x + r.size.x);
	clamp.y = clamp_on_range(p.y, r.origin.y, r.origin.y + r.size.y);

	return clamp;
}

/*---------------------------------------------------------------------------------------------
 * (function: circle_rectangle_collide)
 *-------------------------------------------------------------------------------------------*/
short circle_rectangle_collide(circle_t c, rectangle_t r)
{
	vector_2D_t clamped = clamp_on_rectangle(c.center, r);

	return circle_point_collide(c, clamped);
}

/*---------------------------------------------------------------------------------------------
 * (function: circle_oriented_rectangle_collide)
 *-------------------------------------------------------------------------------------------*/
short circle_oriented_rectangle_collide( circle_t c, oriented_rectangle_t r)
{
	rectangle_t lr;

	lr.origin.x = 0;
	lr.origin.y = 0;
	lr.size = multiply_vector(r.half_extend, 2);

	circle_t lc = {{0, 0}, c.radius};
	vector_2D_t distance = subtract_vector(c.center, r.center);
	distance = rotate_vector(distance, -r.rotation);
	lc.center = add_vector(distance, r.half_extend);

	return circle_rectangle_collide(lc, lr);
}

/*---------------------------------------------------------------------------------------------
 * (function: rectangle_segment_collide)
 *-------------------------------------------------------------------------------------------*/
short rectangle_segment_collide(rectangle_t r, line_segment_t s)
{
	Line sLine;
	range_t rRange = {r.origin.x, r->origin.x + r->size.x};
	range_t sRange = {s.point1.x, s->point2.x};
	srange_t = sort_range(&sRange);
	if(!overlapping_ranges(&rRange, &sRange))
		return FALSE;

	rRange.minimum = r.origin.y;
	rRange.maximum = r.origin.y + r->size.y;
	sRange.minimum = s.point1.y;
	sRange.maximum = s.point2.y;
	srange_t = sort_range(sRange);
	if(!overlapping_ranges(rRange, sRange))
		return FALSE;

	sLine.base = s.point1;
	sLine.direction = subtract_vector(s.point2, s->point1);

	return line_rectangle_collide(sLine, r);
}

/*---------------------------------------------------------------------------------------------
 * (function: oriented_rectangle_segment_collide)
 *-------------------------------------------------------------------------------------------*/
short oriented_rectangle_segment_collide(oriented_rectangle_t_t r, line_segment_t s)
{
	rectangle_t lr = {{0, 0}, {r.half_extend.x * 2, r.half_extend.y * 2}};

	line_segment_t ls;

	ls.point1 = subtract_vector(s.point1, r->center);
	ls.point1 = rotate_vector(ls.point1, -r.rotation);
	ls.point1 = add_vector(ls.point1, r.half_extend);
	ls.point2 = subtract_vector(s.point2, r->center);
	ls.point2 = rotate_vector(ls.point2, -r.rotation);
	ls.point2 = add_vector(ls.point2, r.half_extend);

	return rectangle_segment_collide(lr, ls);
}
#endif

short circles_collide( circle_t* a,  circle_t* b)
{
     double radiusSum = a->radius + b->radius;
	 vector_2D_t distance = subtract_vector(&(a->center), &(b->center));
    return dot_product(&distance, &distance) <= radiusSum * radiusSum;
}

short circle_point_collide( circle_t* c,  vector_2D_t* p)
{
	 vector_2D_t distance = subtract_vector(&(c->center), p);
	return dot_product(&distance, &distance) <= c->radius * c->radius;
}

short circle_line_collide( circle_t* c,  line_t* l)
{
     vector_2D_t lc = subtract_vector(&c->center, &l->base);
	vector_2D_t p = project_vector(&lc, &l->direction);
	p = add_vector(&l->base, &p);
    return circle_point_collide(c, &p);
}

short circle_rectangle_collide( circle_t* c,  rectangle_t* r)
{
     vector_2D_t clamped = clamp_on_rectangle(&c->center, r);
    return circle_point_collide(c, &clamped);
}

short circle_segment_collide( circle_t* c,  line_segment_t* s)
{
	if(circle_point_collide(c, &s->point1) || circle_point_collide(c, &s->point2))
		return TRUE;

	{
		 vector_2D_t d = subtract_vector(&s->point2, &s->point1);
		 vector_2D_t lc = subtract_vector(&c->center, &s->point1);
		 vector_2D_t p = project_vector(&lc, &d);
		 vector_2D_t nearest = add_vector(&s->point1, &p);
		return circle_point_collide(c, &nearest) && dot_product(&p, &p) <= dot_product(&d, &d) && 0 <= dot_product(&p, &d);
	}
}

short circle_oriented_rectangle_collide( circle_t* c,  oriented_rectangle_t_t* r)
{
	 rectangle_t lr = {{0, 0}, {r->halfExtend.x * 2, r->halfExtend.y * 2}};

	circle_t lc = {{0, 0}, c->radius};
    lc.center = subtract_vector(&c->center, &r->center);
	lc.center = rotate_vector(&lc.center, -r->rotation);
	lc.center = add_vector(&lc.center, &r->halfExtend);

	return circle_rectangle_collide(&lc, &lr);
}

short rectangles_collide( rectangle_t* a,  rectangle_t* b)
{
    return
		overlapping(a->origin.x, a->origin.x + a->size.x, b->origin.x, b->origin.x + b->size.x) &&
		overlapping(a->origin.y, a->origin.y + a->size.y, b->origin.y, b->origin.y + b->size.y);
}

short points_collide( vector_2D_t* a,  vector_2D_t* b)
{
	return equal_doubles(a->x, b->x) && equal_doubles(a->y, b->y);
}

short line_point_collide( line_t* l,  vector_2D_t* p)
{
     vector_2D_t lp = subtract_vector(p, &l->base);
	return (lp.x == 0 && lp.y == 0) || parallel_vectors(&lp, &l->direction);
}

short lines_collide( line_t* a,  line_t* b)
{
	return !parallel_vectors(&a->direction, &b->direction) || line_point_collide(a, &b->base);
}

short line_segment_collide( line_t* l,  line_segment_t* s)
{
	return !on_one_side(l, s);
}

short point_segment_collide( vector_2D_t* p,  line_segment_t* s)
{
     vector_2D_t d = subtract_vector(&s->point2, &s->point1);
     vector_2D_t lp = subtract_vector(p, &s->point1);
     vector_2D_t pr = project_vector(&lp, &d);
    return points_collide(&lp, &pr) && dot_product(&pr, &pr) <= dot_product(&d, &d) && 0 <= dot_product(&pr, &d);
}

short segments_collide( line_segment_t* a,  line_segment_t* b)
{
	line_t axisA, axisB;
	axisA.base = a->point1;
	axisA.direction = subtract_vector(&a->point2, &a->point1);
	if(0 == axisA.direction.x && 0 == axisA.direction.y)
		return point_segment_collide(&a->point1, b);
	else if(on_one_side(&axisA, b))
		return FALSE;

	axisB.base = b->point1;
	axisB.direction = subtract_vector(&b->point2, &b->point1);
	if(0 == axisB.direction.x && 0 == axisB.direction.y)
		return point_segment_collide(&b->point1, a);
	else if(on_one_side(&axisB, a))
		return FALSE;

	if(parallel_vectors(&axisA.direction, &axisB.direction))
	{
		 vector_2D_t d = unit_vector(&axisA.direction);
		 range_t rangeA = project_segment(a, &d, TRUE);
		 range_t rangeB = project_segment(b, &d, TRUE);
		return overlapping_ranges(&rangeA, &rangeB);
	}
	else
		return TRUE;
}

short line_rectangle_collide( line_t* l,  rectangle_t* r)
{
	 vector_2D_t n = rotate_vector_90(&l->direction);

	double dp1, dp2, dp3, dp4;

    vector_2D_t c1 = r->origin;
    vector_2D_t c2 = add_vector(&c1, &r->size);
	vector_2D_t c3 = {c2.x, c1.y};
	vector_2D_t c4 = {c1.x, c2.y};

    c1 = subtract_vector(&c1, &l->base);
    c2 = subtract_vector(&c2, &l->base);
    c3 = subtract_vector(&c3, &l->base);
    c4 = subtract_vector(&c4, &l->base);

    dp1 = dot_product(&n, &c1);
    dp2 = dot_product(&n, &c2);
    dp3 = dot_product(&n, &c3);
    dp4 = dot_product(&n, &c4);

	return (dp1 * dp2 <= 0) || (dp2 * dp3 <= 0) || (dp3 * dp4 <= 0);
}

short line_oriented_rectangle_collide( line_t* l,  oriented_rectangle_t_t* r)
{
	 rectangle_t lr = {{0, 0}, {r->halfExtend.x * 2, r->halfExtend.y * 2}};

	line_t ll;
	ll.base = subtract_vector(&l->base, &r->center);
	ll.base = rotate_vector(&ll.base, -r->rotation);
	ll.base = add_vector(&ll.base, &r->halfExtend);
	ll.direction = rotate_vector(&l->direction, -r->rotation);

    return line_rectangle_collide(&ll, &lr);
}

short oriented_rectangles_collide( oriented_rectangle_t_t* a,  oriented_rectangle_t_t* b)
{
	line_segment_t edge = oriented_rectangle_edge(a, 0);
	if(separating_axis_for_oriented_rectangle(&edge, b))
		return FALSE;

	edge = oriented_rectangle_edge(a, 1);
	if(separating_axis_for_oriented_rectangle(&edge, b))
		return FALSE;

	edge = oriented_rectangle_edge(b, 0);
	if(separating_axis_for_oriented_rectangle(&edge, a))
		return FALSE;

	edge = oriented_rectangle_edge(b, 1);
	return !separating_axis_for_oriented_rectangle(&edge, a);
}

short point_rectangle_collide( vector_2D_t* p,  rectangle_t* r)
{
     double left = r->origin.x;
     double right = left + r->size.x;
     double bottom = r->origin.y;
     double top = bottom + r->size.y;
    return left <= p->x && bottom <= p->y && p->x <= right && p->y <= top;
}

short oriented_rectangle_point_collide( oriented_rectangle_t_t* r,  vector_2D_t* p)
{
	 rectangle_t lr = {{0, 0}, {r->halfExtend.x * 2, r->halfExtend.y * 2}};
    vector_2D_t lp = subtract_vector(p, &r->center);
	lp = rotate_vector(&lp, -r->rotation);
	lp = add_vector(&lp, &r->halfExtend);

    return point_rectangle_collide(&lp, &lr);
}

short oriented_rectangle_rectangle_collide( oriented_rectangle_t_t* orect,  rectangle_t* aar)
{
	line_segment_t edge;
	 rectangle_t orHull = oriented_rectangle_rectangle_hull(orect);
	if(!rectangles_collide(&orHull, aar))
		return FALSE;

	edge = oriented_rectangle_edge(orect, 0);
	if(separating_axis_for_rectangle(&edge, aar))
		return FALSE;

	edge = oriented_rectangle_edge(orect, 1);
	return !separating_axis_for_rectangle(&edge, aar);
}

short rectangle_segment_collide( rectangle_t* r,  line_segment_t* s)
{
	line_t sline_t;
	range_t rrange_t = {r->origin.x, r->origin.x + r->size.x};
	range_t srange_t = {s->point1.x, s->point2.x};
	srange_t = sort_range(&srange_t);
	if(!overlapping_ranges(&rrange_t, &srange_t))
		return FALSE;

	rrange_t.minimum = r->origin.y;
	rrange_t.maximum = r->origin.y + r->size.y;
	srange_t.minimum = s->point1.y;
	srange_t.maximum = s->point2.y;
	srange_t = sort_range(&srange_t);
	if(!overlapping_ranges(&rrange_t, &srange_t))
		return FALSE;

	sline_t.base = s->point1;
	sline_t.direction = subtract_vector(&s->point2, &s->point1);
    return line_rectangle_collide(&sline_t, r);
}

short oriented_rectangle_segment_collide( oriented_rectangle_t_t* r,  line_segment_t* s)
{
	 rectangle_t lr = {{0, 0}, {r->halfExtend.x * 2, r->halfExtend.y * 2}};

	line_segment_t ls;
    ls.point1 = subtract_vector(&s->point1, &r->center);
    ls.point1 = rotate_vector(&ls.point1, -r->rotation);
    ls.point1 = add_vector(&ls.point1, &r->halfExtend);
    ls.point2 = subtract_vector(&s->point2, &r->center);
    ls.point2 = rotate_vector(&ls.point2, -r->rotation);
    ls.point2 = add_vector(&ls.point2, &r->halfExtend);

    return rectangle_segment_collide(&lr, &ls);
}

short overlapping(double minA, double maxA, double minB, double maxB)
{
    return minB <= maxA && minA <= maxB;
}

range_t project_segment( line_segment_t* s,  vector_2D_t* onto, short ontoIsUnit)
{
	 vector_2D_t ontoUnit = ontoIsUnit ? *onto : unit_vector(onto);

    range_t r;
	r.minimum = dot_product(&ontoUnit, &s->point1);
	r.maximum = dot_product(&ontoUnit, &s->point2);
	r = sort_range(&r);
    return r;
}

short on_one_side( line_t* axis,  line_segment_t* s)
{
	 vector_2D_t d1 = subtract_vector(&axis->base, &s->point1);
	 vector_2D_t d2 = subtract_vector(&axis->base, &s->point2);
	 vector_2D_t n = rotate_vector_90(&axis->direction);
	return dot_product(&n, &d1) * dot_product(&n, &d2) > 0;
}

double clamp_on_range(double x, double min, double max)
{
	return x < min ? min : (max < x ? max : x);
}

vector_2D_t clamp_on_rectangle( vector_2D_t* p,  rectangle_t* r)
{
	vector_2D_t clamp;
	clamp.x = clamp_on_range(p->x, r->origin.x, r->origin.x + r->size.x);
	clamp.y = clamp_on_range(p->y, r->origin.y, r->origin.y + r->size.y);
	return clamp;
}

short parallel_vectors( vector_2D_t* a,  vector_2D_t* b)
{
	 vector_2D_t na = rotate_vector_90(a);
	return !(0 == a->x && 0 == a->y) && !(0 == b->x && 0 == b->y) && equal_doubles(0, dot_product(&na, b));
}

short equal_vectors( vector_2D_t* a,  vector_2D_t* b)
{
	return equal_doubles(a->x, b->x) && equal_doubles(a->y, b->y);
}

vector_2D_t rectangle_corner( rectangle_t* r, int nr)
{
	vector_2D_t corner = r->origin;
	switch(nr % 4)
	{
	case 0:
		corner.x += r->size.x;
		break;
	case 1:
		corner = add_vector(&corner, &r->size);
		break;
	case 2:
		corner.y += r->size.y;
		break;
	default:
		/* corner = r->origin */
		break;
	}
	return corner;
}

vector_2D_t oriented_rectangle_corner( oriented_rectangle_t_t* r, int nr)
{
    vector_2D_t c = r->halfExtend;
    switch(nr % 4) {
        case 0:
            c.x = -c.x;
            break;
        case 1:
			/* c = r->halfExtend */
            break;
        case 2:
            c.y = -c.y;
            break;
        default:
            c = negate_vector(&c);
            break;
    }

	c = rotate_vector(&c, r->rotation);
    return add_vector(&c, &r->center);
}

line_segment_t oriented_rectangle_edge( oriented_rectangle_t_t* r, int nr)
{
	line_segment_t edge;
    vector_2D_t a = r->halfExtend;
    vector_2D_t b = r->halfExtend;

    switch(nr % 4) {
        case 0:/* top edge */
            a.x = -a.x;
            break;
        case 1:/* right edge */
            b.y = -b.y;
            break;
        case 2:/* bottom edge */
            a.y = -a.y;
            b = negate_vector(&b);
            break;
        default:/* left edge */
            a = negate_vector(&a);
            b.x = -b.x;
            break;
    }

	a = rotate_vector(&a, r->rotation);
    a = add_vector(&a, &r->center);

	b = rotate_vector(&b, r->rotation);
    b = add_vector(&b, &r->center);

	edge.point1 = a;
	edge.point2 = b;
	return edge;
}

short separating_axis_for_oriented_rectangle( line_segment_t* axis,  oriented_rectangle_t_t* r)
{
	range_t axisrange_t, r0range_t, r2range_t, rProjection;
	line_segment_t rEdge0 = oriented_rectangle_edge(r, 0);
	line_segment_t rEdge2 = oriented_rectangle_edge(r, 2);
	vector_2D_t n = subtract_vector(&axis->point1, &axis->point2);

	n = unit_vector(&n);
    axisrange_t = project_segment(axis, &n, TRUE);
    r0range_t = project_segment(&rEdge0, &n, TRUE);
    r2range_t = project_segment(&rEdge2, &n, TRUE);
    rProjection = range_hull(&r0range_t, &r2range_t);

    return !overlapping_ranges(&axisrange_t, &rProjection);
}

short separating_axis_for_rectangle( line_segment_t* axis,  rectangle_t* r)
{
	line_segment_t rEdgeA, rEdgeB;
	range_t axisrange_t, rEdgeArange_t, rEdgeBrange_t, rProjection;
	vector_2D_t n = subtract_vector(&axis->point1, &axis->point2);

	n = unit_vector(&n);
	rEdgeA.point1 = rectangle_corner(r, 0);
	rEdgeA.point2 = rectangle_corner(r, 1);
	rEdgeB.point1 = rectangle_corner(r, 2);
	rEdgeB.point2 = rectangle_corner(r, 3);
	rEdgeArange_t = project_segment(&rEdgeA, &n, TRUE);
	rEdgeBrange_t = project_segment(&rEdgeB, &n, TRUE);
	rProjection = range_hull(&rEdgeArange_t, &rEdgeBrange_t);

    axisrange_t = project_segment(axis, &n, TRUE);

    return !overlapping_ranges(&axisrange_t, &rProjection);
}

double minimum(double a, double b)
{
	return a < b ? a : b;
}

double maximum(double a, double b)
{
	return a > b ? a : b;
}

short equal_doubles(double a, double b)
{
	 double epsilon = 1.0f / 8192.0f;/* should be small eFALSEugh for 1.0f == pixel width */
	return fabsf(a - b) < epsilon;
}

double degrees_to_radian(double degrees)
{
    return degrees * PI / 180.0f;
}

double radian_to_degrees(double radian)
{
    return radian * 180.0f / PI;
}

/*typedef struct
{
	double minimum;
	double maximum;
} range_t;*/

short overlapping_ranges( range_t* a,  range_t* b)
{
    return b->minimum <= a->maximum && a->minimum <= b->maximum;
}

range_t range_hull( range_t* a,  range_t* b)
{
	range_t hull;
	hull.minimum = a->minimum < b->minimum ? a->minimum : b->minimum;
	hull.maximum = a->maximum > b->maximum ? a->maximum : b->maximum;
	return hull;
}

range_t sort_range( range_t* r)
{
	range_t sorted = *r;
    if(r->minimum > r->maximum)
	{
		sorted.minimum = r->maximum;
		sorted.maximum = r->minimum;
    }
	return sorted;
}

vector_2D_t rectangle_center( rectangle_t* r)
{
	vector_2D_t center;
	center.x = r->origin.x + r->size.x / 2;
	center.y = r->origin.y + r->size.y / 2;
	return center;
}

vector_2D_t add_vector( vector_2D_t* a,  vector_2D_t* b)
{
	 vector_2D_t r = {a->x + b->x, a->y + b->y};
    return r;
}

vector_2D_t subtract_vector( vector_2D_t* a,  vector_2D_t* b)
{
	 vector_2D_t r = {a->x - b->x, a->y - b->y};
    return r;
}

vector_2D_t negate_vector( vector_2D_t* v)
{
	 vector_2D_t n = {-v->x, -v->y};
    return n;
}

vector_2D_t multiply_vector( vector_2D_t* v, double s)
{
     vector_2D_t r = {v->x * s, v->y * s};
	return r;
}

vector_2D_t divide_vector( vector_2D_t* v, double d)
{
     vector_2D_t r = {v->x / d, v->y / d};
	return r;
}

double dot_product( vector_2D_t* a,  vector_2D_t* b)
{
	return a->x * b->x + a->y * b->y;
}

double vector_length( vector_2D_t* v)
{
	return sqrtf(dot_product(v, v));
}

vector_2D_t rotate_vector( vector_2D_t* v, double degrees)
{
     double radian = degrees_to_radian(degrees);
     double sine = sinf(radian);
     double cosine = cosf(radian);

	 vector_2D_t r = {v->x * cosine - v->y * sine, v->x * sine + v->y * cosine};
    return r;
}

vector_2D_t rotate_vector_90( vector_2D_t* v)
{
	 vector_2D_t r = {-(v->y), v->x};
    return r;
}

vector_2D_t rotate_vector_180( vector_2D_t* v)
{
	return negate_vector(v);
}

vector_2D_t rotate_vector_270( vector_2D_t* v)
{
	 vector_2D_t r = {v->y, -(v->x)};
    return r;
}

vector_2D_t unit_vector( vector_2D_t* v)
{
	 double length = vector_length(v);
	if(length != 0)
		return divide_vector(v, length);
	return *v;
}

double enclosed_angle( vector_2D_t* a,  vector_2D_t* b)
{
     vector_2D_t ua = unit_vector(a);
     vector_2D_t ub = unit_vector(b);
     double dp = dot_product(&ua, &ub);
    return radian_to_degrees(acosf(dp));
}

vector_2D_t project_vector( vector_2D_t* project,  vector_2D_t* onto)
{
     double d = dot_product(onto, onto);
	if(0 != d)
        return multiply_vector(onto, dot_product(project, onto) / d);
    return *onto;
}

rectangle_t enlarge_rectangle_point( rectangle_t* r,  vector_2D_t* p)
{
    rectangle_t enlarged;
    enlarged.origin.x = minimum(r->origin.x, p->x);
    enlarged.origin.y = minimum(r->origin.y, p->y);
    enlarged.size.x = maximum(r->origin.x + r->size.x, p->x);
    enlarged.size.y = maximum(r->origin.y + r->size.y, p->y);
	enlarged.size = subtract_vector(&enlarged.size, &enlarged.origin);
    return enlarged;
}

rectangle_t oriented_rectangle_rectangle_hull( oriented_rectangle_t* r)
{
	rectangle_t h = {{r->center.x, r->center.y}, {0, 0}};
	
	int nr;
	vector_2D_t corner;
	for(nr = 0; nr < 4; ++nr)
	{
		corner = oriented_rectangle_corner(r, nr);
		h = enlarge_rectangle_point(&h, &corner);
	}
    return h;
}
