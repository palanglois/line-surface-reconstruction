#ifndef LINE_BASED_RECONS_REFACTO_CGALTYPES_H
#define LINE_BASED_RECONS_REFACTO_CGALTYPES_H

//STL
#include <vector>

//CGAL
#include <CGAL/Exact_predicates_exact_constructions_kernel.h>
#if defined(_OPENMP)
#include <CGAL/gmpxx.h>
#endif
#include <CGAL/intersections.h>

/* Default 3D Kernel */
#if defined(_OPENMP)
//This Kernel is slower than Epeck, but it is thread-safe
struct Kernel : CGAL::Filtered_kernel_adaptor<CGAL::Type_equality_wrapper<CGAL::Simple_cartesian<mpq_class>::Base<Kernel>::Type, Kernel>,true> {};
#else
// This is the fastest kernel for exact construction but as of today (3 Sept 2018), it is not thread-safe.
// See https://github.com/CGAL/cgal/issues/2685
typedef CGAL::Exact_predicates_exact_constructions_kernel Kernel;
#endif

/* Typedefs for the geometrical primitives */
typedef typename Kernel::FT Scalar;
typedef typename Kernel::Point_3 Point;
typedef typename Kernel::Vector_3 Vector;
typedef typename Kernel::Segment_3 Segment;
typedef typename Kernel::Line_3 LineCgal;
typedef typename Kernel::Plane_3 Plane;
typedef typename Kernel::Direction_3 Direction;
typedef typename Kernel::Triangle_3 Triangle;
typedef typename Kernel::Intersect_3 Intersect;

#endif //LINE_BASED_RECONS_REFACTO_CGALTYPES_H
