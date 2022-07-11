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

/*---------------------------------------------------------------------------------------------
 * (function: clamp_on_rectangle)
 *-------------------------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------------------------
 * (function: circles_collide)
 *-------------------------------------------------------------------------------------------*/
short circles_collide( circle_t* a,  circle_t* b)
{
	double radiusSum = a->radius + b->radius;

	vector_2D_t distance = subtract_vector(&(a->center), &(b->center));

	return dot_product(&distance, &distance) <= radiusSum * radiusSum;
}

/*---------------------------------------------------------------------------------------------
 * (function: circle_point_collide)
 *-------------------------------------------------------------------------------------------*/
short circle_point_collide( circle_t* c,  vector_2D_t* p)
{
	vector_2D_t distance = subtract_vector(&(c->center), p);

	return dot_product(&distance, &distance) <= c->radius * c->radius;
}

/*---------------------------------------------------------------------------------------------
 * (function: circle_line_collide)
 *-------------------------------------------------------------------------------------------*/
short circle_line_collide( circle_t* c,  line_t* l)
{
	vector_2D_t lc = subtract_vector(&c->center, &l->base);
	vector_2D_t p = project_vector(&lc, &l->direction);
	p = add_vector(&l->base, &p);

	return circle_point_collide(c, &p);
}

/*---------------------------------------------------------------------------------------------
 * (function: circle_rectangle_collide)
 *-------------------------------------------------------------------------------------------*/
short circle_rectangle_collide( circle_t* c,  rectangle_t* r)
{
	vector_2D_t clamped = clamp_on_rectangle(&c->center, r);

	return circle_point_collide(c, &clamped);
}

/*---------------------------------------------------------------------------------------------
 * (function: circle_segment_collide)
 *-------------------------------------------------------------------------------------------*/
short circle_segment_collide( circle_t* c,  line_segment_t* s)
{
	if(circle_point_collide(c, &s->point1) || circle_point_collide(c, &s->point2))
		return TRUE;

	vector_2D_t d = subtract_vector(&s->point2, &s->point1);
	vector_2D_t lc = subtract_vector(&c->center, &s->point1);
	vector_2D_t p = project_vector(&lc, &d);
	vector_2D_t nearest = add_vector(&s->point1, &p);

	return circle_point_collide(c, &nearest) && dot_product(&p, &p) <= dot_product(&d, &d) && 0 <= dot_product(&p, &d);
}

/*---------------------------------------------------------------------------------------------
 * (function: circle_oriented_rectangle_collide)
 *-------------------------------------------------------------------------------------------*/
short circle_oriented_rectangle_collide( circle_t* c,  oriented_rectangle_t_t* r)
{
	rectangle_t lr = {{0, 0}, {r->halfExtend.x * 2, r->halfExtend.y * 2}};

	circle_t lc = {{0, 0}, c->radius};
	lc.center = subtract_vector(&c->center, &r->center);
	lc.center = rotate_vector(&lc.center, -r->rotation);
	lc.center = add_vector(&lc.center, &r->halfExtend);

	return circle_rectangle_collide(&lc, &lr);
}

/*---------------------------------------------------------------------------------------------
 * (function: rectangles_collide)
 *-------------------------------------------------------------------------------------------*/
short rectangles_collide( rectangle_t* a,  rectangle_t* b)
{
	return
		overlapping(a->origin.x, a->origin.x + a->size.x, b->origin.x, b->origin.x + b->size.x) &&
		overlapping(a->origin.y, a->origin.y + a->size.y, b->origin.y, b->origin.y + b->size.y);
}

/*---------------------------------------------------------------------------------------------
 * (function: points_collide)
 *-------------------------------------------------------------------------------------------*/
short points_collide( vector_2D_t* a,  vector_2D_t* b)
{
	return equal_doubles(a->x, b->x) && equal_doubles(a->y, b->y);
}

/*---------------------------------------------------------------------------------------------
 * (function: line_point_collide)
 *-------------------------------------------------------------------------------------------*/
short line_point_collide( line_t* l,  vector_2D_t* p)
{
	vector_2D_t lp = subtract_vector(p, &l->base);

	return (lp.x == 0 && lp.y == 0) || parallel_vectors(&lp, &l->direction);
}

/*---------------------------------------------------------------------------------------------
 * (function: lines_collide)
 *-------------------------------------------------------------------------------------------*/
short lines_collide( line_t* a,  line_t* b)
{
	return !parallel_vectors(&a->direction, &b->direction) || line_point_collide(a, &b->base);
}

/*---------------------------------------------------------------------------------------------
 * (function: line_segment_collide)
 *-------------------------------------------------------------------------------------------*/
short line_segment_collide( line_t* l,  line_segment_t* s)
{
	return !on_one_side(l, s);
}

/*---------------------------------------------------------------------------------------------
 * (function: point_segment_collide)
 *-------------------------------------------------------------------------------------------*/
short point_segment_collide( vector_2D_t* p,  line_segment_t* s)
{
	vector_2D_t d = subtract_vector(&s->point2, &s->point1);
	vector_2D_t lp = subtract_vector(p, &s->point1);
	vector_2D_t pr = project_vector(&lp, &d);
	return points_collide(&lp, &pr) && dot_product(&pr, &pr) <= dot_product(&d, &d) && 0 <= dot_product(&pr, &d);
}

/*---------------------------------------------------------------------------------------------
 * (function: segments_collide)
 *-------------------------------------------------------------------------------------------*/
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

/*---------------------------------------------------------------------------------------------
 * (function: line_rectangle_collide)
 *-------------------------------------------------------------------------------------------*/
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

/*---------------------------------------------------------------------------------------------
 * (function: line_oriented_rectangle_collide)
 *-------------------------------------------------------------------------------------------*/
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

/*---------------------------------------------------------------------------------------------
 * (function: oriented_rectangles_collide)
 *-------------------------------------------------------------------------------------------*/
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

/*---------------------------------------------------------------------------------------------
 * (function: point_rectangle_collide)
 *-------------------------------------------------------------------------------------------*/
short point_rectangle_collide( vector_2D_t* p,  rectangle_t* r)
{
	double left = r->origin.x;
	double right = left + r->size.x;
	double bottom = r->origin.y;
	double top = bottom + r->size.y;

	return left <= p->x && bottom <= p->y && p->x <= right && p->y <= top;
}

/*---------------------------------------------------------------------------------------------
 * (function: oriented_rectangle_point_collide)
 *-------------------------------------------------------------------------------------------*/
short oriented_rectangle_point_collide( oriented_rectangle_t_t* r,  vector_2D_t* p)
{
	rectangle_t lr = {{0, 0}, {r->halfExtend.x * 2, r->halfExtend.y * 2}};
	vector_2D_t lp = subtract_vector(p, &r->center);
	lp = rotate_vector(&lp, -r->rotation);
	lp = add_vector(&lp, &r->halfExtend);

	return point_rectangle_collide(&lp, &lr);
}

/*---------------------------------------------------------------------------------------------
 * (function: oriented_rectangle_rectangle_collide)
 *-------------------------------------------------------------------------------------------*/
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

/*---------------------------------------------------------------------------------------------
 * (function: rectangle_segment_collide)
 *-------------------------------------------------------------------------------------------*/
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

/*---------------------------------------------------------------------------------------------
 * (function: oriented_rectangle_segment_collide)
 *-------------------------------------------------------------------------------------------*/
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

/*---------------------------------------------------------------------------------------------
 * (function: overlapping)
 *-------------------------------------------------------------------------------------------*/
short overlapping(double minA, double maxA, double minB, double maxB)
{
	return minB <= maxA && minA <= maxB;
}

/*---------------------------------------------------------------------------------------------
 * (function: project_segment)
 *-------------------------------------------------------------------------------------------*/
range_t project_segment( line_segment_t* s,  vector_2D_t* onto, short ontoIsUnit)
{
	vector_2D_t ontoUnit = ontoIsUnit ? *onto : unit_vector(onto);

	range_t r;
	r.minimum = dot_product(&ontoUnit, &s->point1);
	r.maximum = dot_product(&ontoUnit, &s->point2);
	r = sort_range(&r);

	return r;
}

/*---------------------------------------------------------------------------------------------
 * (function: on_one_side)
 *-------------------------------------------------------------------------------------------*/
short on_one_side( line_t* axis,  line_segment_t* s)
{
	vector_2D_t d1 = subtract_vector(&axis->base, &s->point1);
	vector_2D_t d2 = subtract_vector(&axis->base, &s->point2);
	vector_2D_t n = rotate_vector_90(&axis->direction);

	return dot_product(&n, &d1) * dot_product(&n, &d2) > 0;
}

/*---------------------------------------------------------------------------------------------
 * (function: clamp_on_range)
 *-------------------------------------------------------------------------------------------*/
double clamp_on_range(double x, double min, double max)
{
	return x < min ? min : (max < x ? max : x);
}

/*---------------------------------------------------------------------------------------------
 * (function: clamp_on_rectangle)
 *-------------------------------------------------------------------------------------------*/
vector_2D_t clamp_on_rectangle( vector_2D_t* p,  rectangle_t* r)
{
	vector_2D_t clamp;
	clamp.x = clamp_on_range(p->x, r->origin.x, r->origin.x + r->size.x);
	clamp.y = clamp_on_range(p->y, r->origin.y, r->origin.y + r->size.y);

	return clamp;
}

/*---------------------------------------------------------------------------------------------
 * (function: parallel_vectors)
 *-------------------------------------------------------------------------------------------*/
short parallel_vectors( vector_2D_t* a,  vector_2D_t* b)
{
	vector_2D_t na = rotate_vector_90(a);

	return !(0 == a->x && 0 == a->y) && !(0 == b->x && 0 == b->y) && equal_doubles(0, dot_product(&na, b));
}

/*---------------------------------------------------------------------------------------------
 * (function: equal_vectors)
 *-------------------------------------------------------------------------------------------*/
short equal_vectors( vector_2D_t* a,  vector_2D_t* b)
{
	return equal_doubles(a->x, b->x) && equal_doubles(a->y, b->y);
}

/*---------------------------------------------------------------------------------------------
 * (function: rectangle_corner)
 *-------------------------------------------------------------------------------------------*/
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

/*---------------------------------------------------------------------------------------------
 * (function: oriented_rectangle_corner)
 *-------------------------------------------------------------------------------------------*/
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

/*---------------------------------------------------------------------------------------------
 * (function: oriented_rectangle_edge)
 *-------------------------------------------------------------------------------------------*/
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

/*---------------------------------------------------------------------------------------------
 * (function: separating_axis_for_oriented_rectangle)
 *-------------------------------------------------------------------------------------------*/
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

/*---------------------------------------------------------------------------------------------
 * (function: separating_axis_for_rectangle)
 *-------------------------------------------------------------------------------------------*/
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

/*---------------------------------------------------------------------------------------------
 * (function: minimum)
 *-------------------------------------------------------------------------------------------*/
double minimum(double a, double b)
{
	return a < b ? a : b;
}

/*---------------------------------------------------------------------------------------------
 * (function: maximum)
 *-------------------------------------------------------------------------------------------*/
double maximum(double a, double b)
{
	return a > b ? a : b;
}

/*---------------------------------------------------------------------------------------------
 * (function: equal_doubles)
 *-------------------------------------------------------------------------------------------*/
short equal_doubles(double a, double b)
{
	double epsilon = 1.0f / 8192.0f;/* should be small eFALSEugh for 1.0f == pixel width */

	return fabsf(a - b) < epsilon;
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
double radian_to_degrees(double radian)
{
	return radian * 180.0f / PI;
}

/*---------------------------------------------------------------------------------------------
 * (function: overlapping_ranges)
 *-------------------------------------------------------------------------------------------*/
short overlapping_ranges( range_t* a,  range_t* b)
{
	return b->minimum <= a->maximum && a->minimum <= b->maximum;
}

/*---------------------------------------------------------------------------------------------
 * (function: range_hull)
 *-------------------------------------------------------------------------------------------*/
range_t range_hull( range_t* a,  range_t* b)
{
	range_t hull;
	hull.minimum = a->minimum < b->minimum ? a->minimum : b->minimum;
	hull.maximum = a->maximum > b->maximum ? a->maximum : b->maximum;

	return hull;
}

/*---------------------------------------------------------------------------------------------
 * (function: sort_range)
 *-------------------------------------------------------------------------------------------*/
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

/*---------------------------------------------------------------------------------------------
 * (function: rectangle_center)
 *-------------------------------------------------------------------------------------------*/
vector_2D_t rectangle_center( rectangle_t* r)
{
	vector_2D_t center;
	center.x = r->origin.x + r->size.x / 2;
	center.y = r->origin.y + r->size.y / 2;

	return center;
}

/*---------------------------------------------------------------------------------------------
 * (function: add_vector)
 *-------------------------------------------------------------------------------------------*/
vector_2D_t add_vector( vector_2D_t* a,  vector_2D_t* b)
{
	vector_2D_t r = {a->x + b->x, a->y + b->y};

	return r;
}

/*---------------------------------------------------------------------------------------------
 * (function: subtract_vector)
 *-------------------------------------------------------------------------------------------*/
vector_2D_t subtract_vector( vector_2D_t* a,  vector_2D_t* b)
{
	vector_2D_t r = {a->x - b->x, a->y - b->y};

	return r;
}

/*---------------------------------------------------------------------------------------------
 * (function: negate_vector)
 *-------------------------------------------------------------------------------------------*/
vector_2D_t negate_vector( vector_2D_t* v)
{
	vector_2D_t n = {-v->x, -v->y};

	return n;
}

/*---------------------------------------------------------------------------------------------
 * (function: multiply_vector)
 *-------------------------------------------------------------------------------------------*/
vector_2D_t multiply_vector( vector_2D_t* v, double s)
{
	vector_2D_t r = {v->x * s, v->y * s};

	return r;
}

/*---------------------------------------------------------------------------------------------
 * (function: divide_vector)
 *-------------------------------------------------------------------------------------------*/
vector_2D_t divide_vector( vector_2D_t* v, double d)
{
	vector_2D_t r = {v->x / d, v->y / d};

	return r;
}

/*---------------------------------------------------------------------------------------------
 * (function: dot_product)
 *-------------------------------------------------------------------------------------------*/
double dot_product( vector_2D_t* a,  vector_2D_t* b)
{
	return a->x * b->x + a->y * b->y;
}

/*---------------------------------------------------------------------------------------------
 * (function: vector_length)
 *-------------------------------------------------------------------------------------------*/
double vector_length( vector_2D_t* v)
{
	return sqrtf(dot_product(v, v));
}

/*---------------------------------------------------------------------------------------------
 * (function: rotate_vector)
 *-------------------------------------------------------------------------------------------*/
vector_2D_t rotate_vector( vector_2D_t* v, double degrees)
{
	double radian = degrees_to_radian(degrees);
	double sine = sinf(radian);
	double cosine = cosf(radian);

	vector_2D_t r = {v->x * cosine - v->y * sine, v->x * sine + v->y * cosine};

	return r;
}

/*---------------------------------------------------------------------------------------------
 * (function: rotate_vector_90)
 *-------------------------------------------------------------------------------------------*/
vector_2D_t rotate_vector_90( vector_2D_t* v)
{
	vector_2D_t r = {-(v->y), v->x};

	return r;
}

/*---------------------------------------------------------------------------------------------
 * (function: rotate_vector_180)
 *-------------------------------------------------------------------------------------------*/
vector_2D_t rotate_vector_180( vector_2D_t* v)
{
	return negate_vector(v);
}

/*---------------------------------------------------------------------------------------------
 * (function: rotate_vector_270)
 *-------------------------------------------------------------------------------------------*/
vector_2D_t rotate_vector_270( vector_2D_t* v)
{
	vector_2D_t r = {v->y, -(v->x)};

	return r;
}

/*---------------------------------------------------------------------------------------------
 * (function: unit_vector)
 *-------------------------------------------------------------------------------------------*/
vector_2D_t unit_vector( vector_2D_t* v)
{
	double length = vector_length(v);

	if(length != 0)
		return divide_vector(v, length);

	return *v;
}

/*---------------------------------------------------------------------------------------------
 * (function: enclosed_angle)
 *-------------------------------------------------------------------------------------------*/
double enclosed_angle( vector_2D_t* a,  vector_2D_t* b)
{
	vector_2D_t ua = unit_vector(a);
	vector_2D_t ub = unit_vector(b);
	double dp = dot_product(&ua, &ub);

	return radian_to_degrees(acosf(dp));
}

/*---------------------------------------------------------------------------------------------
 * (function: project_vector)
 *-------------------------------------------------------------------------------------------*/
vector_2D_t project_vector( vector_2D_t* project,  vector_2D_t* onto)
{
	double d = dot_product(onto, onto);

	if(0 != d)
		return multiply_vector(onto, dot_product(project, onto) / d);

	return *onto;
}

/*---------------------------------------------------------------------------------------------
 * (function: enlarge_rectangle_point)
 *-------------------------------------------------------------------------------------------*/
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

/*---------------------------------------------------------------------------------------------
 * (function: oriented_rectangle_rectangle_hull)
 *-------------------------------------------------------------------------------------------*/
rectangle_t oriented_rectangle_rectangle_hull( oriented_rectangle_t* r)
{
	int nr;
	rectangle_t h = {{r->center.x, r->center.y}, {0, 0}};

	vector_2D_t corner;

	for(nr = 0; nr < 4; ++nr)
	{
		corner = oriented_rectangle_corner(r, nr);
		h = enlarge_rectangle_point(&h, &corner);
	}

	return h;
}
 
/*---------------------------------------------------------------------------------------------
 * (function: square)
 *-------------------------------------------------------------------------------------------*/
double square(double x) 
{
	return x * x;
}
 
/*---------------------------------------------------------------------------------------------
 * (function: helper_point_within_rectangle)
 *-------------------------------------------------------------------------------------------*/
short helper_point_within_rectangle(double x1, double y1, double x2, double y2, double x, double y) 
{
	double epsilon = 1.0f / 8192.0f;/* should be small eFALSEugh for 1.0f == pixel width */

	double d1 = sqrt(square(x2 - x1) + square(y2 - y1));	// distance between end-points
	double d2 = sqrt(square(x - x1) + square(y - y1));	  // distance from point to one end
	double d3 = sqrt(square(x2 - x) + square(y2 - y));	  // distance from point to other end
	double delta = d1 - d2 - d3;

	return fabs(delta) < epsilon;   // true if delta is less than a small tolerance
}
 
double helper_function_fx(double A, double B, double C, double x) 
{
	return -(A * x + C) / B;
}
double helper_function_fy(double A, double B, double C, double y) 
{
	return -(B * y + C) / A;
}
 
/*---------------------------------------------------------------------------------------------
 * (function: segment_intersects_circle_at) 
 * returns NULL if no intersection
 *
 * Prints the intersection points (if any) of a circle, center 'cp' with radius , 
 * and either a segment drawn between those points.  
 * CODE from : https://rosettacode.org/wiki/Line_circle_intersection#C
 *-------------------------------------------------------------------------------------------*/
points_t *segment_intersects_circle_at(line_segment_t *line_segment, circle_t *circle)
{
	double epsilon = 1.0f / 8192.0f;/* should be small eFALSEugh for 1.0f == pixel width */
	double x0 = circle->center.x;
       	double y0 = circle->center.y;
	double x1 = line_segment->point1.x;
	double y1 = line_segment->point1.y;
	double x2 = line_segment->point2.x;
       	double y2 = line_segment->point2.y;

	double A = y2 - y1; // length ydir
	double B = x1 - x2; // length xdir
	double C = x2 * y1 - x1 * y2; //
	double a = square(A) + square(B); // length vector
	double b, c, d;
	short bnz = TRUE;
	int cnt = 0;

	points_t *return_points = NULL;
 
	if (fabs(B) >= epsilon) 
	{
		// if B isn't zero or close to it
		b = 2 * (A * C + A * B * y0 - square(B) * x0);
		c = square(C) + 2 * B * C * y0 - square(B) * (square(circle->radius) - square(x0) - square(y0));
	} 
	else 
	{
		b = 2 * (B * C + A * B * x0 - square(A) * y0);
		c = square(C) + 2 * A * C * x0 - square(A) * (square(circle->radius) - square(x0) - square(y0));
		bnz = false;
	}
	d = square(b) - 4 * a * c; // discriminant
	if (d < 0) 
	{
		// line & circle don't intersect
		return return_points;
	}
 
	if (d == 0) 
	{
		// line is tangent to circle, so just one intersect at most
		
		return_points = (points_t*)malloc(sizeof(points_t));
		return_points->num_points = 1;
		return_points->points = (vector_2D_t**)malloc(sizeof(vector_2D_t*));
		return_points->points[0] = (vector_2D_t*)malloc(sizeof(vector_2D_t));

		if (bnz) 
		{
			double x = -b / (2 * a);
			double y = helper_function_fx(A, B, C, x);

			if (helper_point_within_rectangle(x1, y1, x2, y2, x, y))
			{
				return_points->points[0]->x = x;	
				return_points->points[0]->y = y;	
			}
			else
			{
				return_points->num_points = 0;
				free(return_points->points[0]);
				free(return_points->points);
				free(return_points);
				return_points = NULL;
			}
		} 
		else 
		{
			double y = -b / (2 * a);
			double x = helper_function_fy(A, B, C, y);

			if (helper_point_within_rectangle(x1, y1, x2, y2, x, y))
			{
				return_points->points[0]->x = x;	
				return_points->points[0]->y = y;	
			}
			else
			{
				return_points->num_points = 0;
				free(return_points->points[0]);
				free(return_points->points);
				free(return_points);
				return_points = NULL;
			}
		}
	} 
	else 
	{
		// two intersects at most
		return_points = (points_t*)malloc(sizeof(points_t));
		return_points->num_points = 2;
		return_points->points = (vector_2D_t**)malloc(sizeof(vector_2D_t*)*2);
		return_points->points[0] = (vector_2D_t*)malloc(sizeof(vector_2D_t));
		return_points->points[1] = (vector_2D_t*)malloc(sizeof(vector_2D_t));
		cnt = 1;

		d = sqrt(d);
		if (bnz) 
		{
			double x = (-b + d) / (2 * a);
			double y = helper_function_fx(A, B, C, x);

			if (helper_point_within_rectangle(x1, y1, x2, y2, x, y))
			{
				return_points->points[0]->x = x;	
				return_points->points[0]->y = y;	
			}
			else
			{
				return_points->num_points --;
				free(return_points->points[1]);
				cnt--;
			}
 
			x = (-b - d) / (2 * a);
			y = helper_function_fx(A, B, C, x);

			if (helper_point_within_rectangle(x1, y1, x2, y2, x, y))
			{
				return_points->points[cnt]->x = x;	
				return_points->points[cnt]->y = y;	
			}
			else if (cnt == 1)
			{
				return_points->num_points --;
				free(return_points->points[cnt]);
				cnt--;
			}
			else if (cnt == 0)
			{
				return_points->num_points = 0;
				free(return_points->points[0]);
				free(return_points->points);
				free(return_points);
				return_points = NULL;
			}
		} 
		else 
		{
			double y = (-b + d) / (2 * a);
			double x = helper_function_fy(A, B, C, y);

			if (helper_point_within_rectangle(x1, y1, x2, y2, x, y))
			{
				return_points->points[0]->x = x;	
				return_points->points[0]->y = y;	
			}
 
			y = (-b - d) / (2 * a);
			x = helper_function_fy(A, B, C, y);

			if (helper_point_within_rectangle(x1, y1, x2, y2, x, y))
			{
				return_points->points[cnt]->x = x;	
				return_points->points[cnt]->y = y;	
			}
			else if (cnt == 1)
			{
				return_points->num_points --;
				free(return_points->points[cnt]);
				cnt--;
			}
			else if (cnt == 0)
			{
				return_points->num_points = 0;
				free(return_points->points[0]);
				free(return_points->points);
				free(return_points);
				return_points = NULL;
			}
		}
	}
 
	return return_points;
}


/*---------------------------------------------------------------------------------------------
 * (function: line_segments_intersect_at) 
 *
 * From : http://paulbourke.net/geometry/pointlineplane/pdb.c
   Determine the intersection point of two line segments
   Return FALSE if the lines don't intersect
*/
short line_segments_intersect_at(line_segment_t *A, line_segment_t *B, vector_2D_t *intersection) 
{
	double mua,mub;
	double denom,numera,numerb;
	double epsilon = 1.0f / 8192.0f;/* should be small eFALSEugh for 1.0f == pixel width */

	denom  = (B->point2.y-B->point1.y) * (A->point2.x-A->point1.x) - (B->point2.x-B->point1.x) * (A->point2.y-A->point1.y);
	numera = (B->point2.x-B->point1.x) * (A->point1.y-B->point1.y) - (B->point2.y-B->point1.y) * (A->point1.x-B->point1.x);
	numerb = (A->point2.x-A->point1.x) * (A->point1.y-B->point1.y) - (A->point2.y-A->point1.y) * (A->point1.x-B->point1.x);

	/* Are the line coincident? */
	if (abs(numera) < epsilon && abs(numerb) < epsilon && abs(denom) < epsilon) 
	{
		intersection->x = (A->point1.x + A->point2.x) / 2;
		intersection->y = (A->point1.y + A->point2.y) / 2;
		return(TRUE);
	}

	/* Are the line parallel */
	if (abs(denom) < epsilon) 
	{
		intersection->x = 0;
		intersection->y = 0;
		return(FALSE);
	}

	/* Is the intersection along the the segments */
	mua = numera / denom;
	mub = numerb / denom;
	if (mua < 0 || mua > 1 || mub < 0 || mub > 1) 
	{
		intersection->x = 0;
		intersection->y = 0;
		return(FALSE);
	}
	intersection->x = A->point1.x + mua * (A->point2.x - A->point1.x);
	intersection->y = A->point1.y + mua * (A->point2.y - A->point1.y);
	return(TRUE);
}

/*---------------------------------------------------------------------------------------------
 * (function: segment_intersects_oriented_rectangle_at) 
 *-------------------------------------------------------------------------------------------*/
points_t *segment_intersects_oriented_rectangle_at(line_segment_t *line_segment, oriented_rectangle_t *rectangle) 
{
	/* rectangles points    a b
	 * 			c d */
	vector_2D_t a;
	a.x = rectangle->center.x - rectangle->halfExtend.x;
	a.y = rectangle->center.y + rectangle->halfExtend.y;
	vector_2D_t b;
	a.x = rectangle->center.x + rectangle->halfExtend.x;
	a.y = rectangle->center.y + rectangle->halfExtend.y;
	vector_2D_t c;
	a.x = rectangle->center.x - rectangle->halfExtend.x;
	a.y = rectangle->center.y - rectangle->halfExtend.y;
	vector_2D_t d;
	a.x = rectangle->center.x + rectangle->halfExtend.x;
	a.y = rectangle->center.y - rectangle->halfExtend.y;

	/* create line segments of rectangle */
	line_segment_t a_b;
	a_b.point1.x = a.x;
	a_b.point1.y = a.y;
	a_b.point2.x = b.x;
	a_b.point2.y = b.y;
	line_segment_t a_c;
	a_c.point1.x = a.x;
	a_c.point1.y = a.y;
	a_c.point2.x = c.x;
	a_c.point2.y = c.y;
	line_segment_t b_d;
	b_d.point1.x = b.x;
	b_d.point1.y = b.y;
	b_d.point2.x = d.x;
	b_d.point2.y = d.y;
	line_segment_t c_d;
	c_d.point1.x = c.x;
	c_d.point1.y = c.y;
	c_d.point2.x = d.x;
	c_d.point2.y = d.y;

	points_t *return_points;	
	return_points = (points_t *)malloc(sizeof(points_t));
	return_points->points = (vector_2D_t **)malloc(sizeof(vector_2D_t*)*2);
	return_points->num_points = 0;

	vector_2D_t intersection_point;

	if (line_segments_intersect_at(&a_b, line_segment, &intersection_point))
	{
		return_points->points[return_points->num_points] = (vector_2D_t *)malloc(sizeof(vector_2D_t));
		return_points->points[return_points->num_points]->x = intersection_point.x;
		return_points->points[return_points->num_points]->y = intersection_point.y;
		return_points->num_points ++;
	}
	if (line_segments_intersect_at(&a_c, line_segment, &intersection_point))
	{
		return_points->points[return_points->num_points] = (vector_2D_t *)malloc(sizeof(vector_2D_t));
		return_points->points[return_points->num_points]->x = intersection_point.x;
		return_points->points[return_points->num_points]->y = intersection_point.y;
		return_points->num_points ++;
	}
	if (line_segments_intersect_at(&b_d, line_segment, &intersection_point))
	{
		return_points->points[return_points->num_points] = (vector_2D_t *)malloc(sizeof(vector_2D_t));
		return_points->points[return_points->num_points]->x = intersection_point.x;
		return_points->points[return_points->num_points]->y = intersection_point.y;
		return_points->num_points ++;
	}
	if (line_segments_intersect_at(&c_d, line_segment, &intersection_point))
	{
		return_points->points[return_points->num_points] = (vector_2D_t *)malloc(sizeof(vector_2D_t));
		return_points->points[return_points->num_points]->x = intersection_point.x;
		return_points->points[return_points->num_points]->y = intersection_point.y;
		return_points->num_points ++;
	}

	oassert(return_points->num_points <= 2);

	if(return_points->num_points == 0)
	{
		/* clean up if no intersections */
		free(return_points->points);
		free(return_points);
		return_points = NULL;
	}

	return return_points;
}
