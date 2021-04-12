#ifndef POLYHEDRAL_COMPLEX_3_ARRANGEMENT_3_HPP
#define POLYHEDRAL_COMPLEX_3_ARRANGEMENT_3_HPP

#include<algorithm>
#include<cassert>
#include<iostream>
#include<queue>
#include<vector>
#include<set>

#include <CGAL/basic.h>
#include <CGAL/Bbox_3.h>

#include <CGAL/Exact_predicates_exact_constructions_kernel.h>

#include <CGAL/Homogeneous_d.h>
#include <CGAL/predicates_d.h>

#include <CGAL/squared_distance_3.h>

namespace Polyhedral_complex_3 {

template<class K,class Fh>
class Arrangement_face_base_3;

template<class K,class Fh>
std::ostream & operator<<(std::ostream &os, const Arrangement_face_base_3<K,Fh> & face);
template<class K,class Fh>
std::istream & operator>>(std::istream &is,  Arrangement_face_base_3<K,Fh> & face);

template<class K,
class Fh>
class Arrangement_face_base_3 {
public:
	typedef K Kernel;
	typedef typename Kernel::Point_3 Point;
	typedef typename Kernel::Line_3 Line;
	typedef typename Kernel::Ray_3 Ray;
	typedef typename Kernel::Segment_3 Segment;

	typedef Fh Face_handle;

	typedef std::vector<Face_handle> Superface_list;
	typedef typename Superface_list::iterator Superfaces_iterator;
	typedef typename Superface_list::const_iterator
			Superfaces_const_iterator;
	typedef std::vector<Face_handle> Subface_list;
	typedef typename Subface_list::iterator Subfaces_iterator;
	typedef typename Subface_list::const_iterator
			Subfaces_const_iterator;


public:
	enum {VERTEX, EDGE, FACET, CELL};

	int type;

	bool is_bbox_adj; 
	bool to_draw;
	std::string ifcType;

	double facet_nbr_ray;
	double facet_nbr_pts;
	double facet_nbr_ray_vis_point;
	double facet_nbr_ray_vis_line;
	double facet_nbr_ray_data_point;
	double facet_nbr_ray_data_line;
	
	double cell_value_void_points;
	double cell_value_full_points;
	double cell_value_void_view_points;

	float dist_to_points;


	int cell_type;
	int void_volume_nb; 
	
	double area;
	double solid_angle;
	double density;

	Arrangement_face_base_3();

public:


	const Point& point() const;

	Point& point();

	const int& info() const;

	int& info();

	void insert_superface(Face_handle fh);

	void remove_superface(Face_handle fh);

	Superfaces_const_iterator superfaces_begin() const;

	Superfaces_iterator superfaces_begin();

	Superfaces_const_iterator superfaces_end() const;

	Superfaces_iterator superfaces_end();

	int number_of_superfaces() const;

	bool has_superface(Face_handle fh) const;

	Face_handle superface(int i) const;

	Face_handle& superface(int i);

	const Superface_list& superfaces() const;

	Superface_list& superfaces();

	void clear_superfaces();

	void insert_subface(Face_handle fh);

	void remove_subface(Face_handle fh);

	Subfaces_const_iterator subfaces_begin() const;

	Subfaces_iterator subfaces_begin();

	Subfaces_const_iterator subfaces_end() const;

	Subfaces_iterator subfaces_end();

	int number_of_subfaces() const;

	bool has_subface(Face_handle fh) const;

	Face_handle subface(int i) const;

	Face_handle& subface(int i);

	const Subface_list& subfaces() const;

	Subface_list& subfaces();

	void clear_subfaces();

	bool is_segment() const;

	bool is_ray() const;

protected:
	Superfaces_const_iterator _find_superface(Face_handle fh) const;

	Superfaces_iterator _find_superface(Face_handle fh);

	Subfaces_const_iterator _find_subface(Face_handle fh) const;

	Subfaces_iterator _find_subface(Face_handle fh);

protected:
	Subface_list _subfaces;
	Superface_list _superfaces;

	Point _point;
	int _info; 

public:

	friend std::ostream& operator<< <K,Fh>(std::ostream &os, const Arrangement_face_base_3<K,Fh> & face);
	friend std::istream& operator>> <K,Fh>(std::istream &is, Arrangement_face_base_3<K,Fh> & face);

};


template<class K,class Fh>
std::ostream & operator<<(std::ostream &os, const Arrangement_face_base_3<K,Fh> & face);


template<class K,class Fh>
std::istream & operator>>(std::istream &is, Arrangement_face_base_3<K,Fh> & face);

template<class K>
class Arrangement_predicates_3 {
    public:
        typedef K Kernel;

        typedef typename Kernel::Line_3 Line;
        typedef typename Kernel::Object_3 Object;
        typedef typename Kernel::Plane_3 Plane;
        typedef typename Kernel::Point_3 Point;
        typedef typename Kernel::Ray_3 Ray;
        typedef typename Kernel::Segment_3 Segment;

    public:
        static int oriented_side(const Plane& pl, const Point& p);

        static bool do_intersect_cl(const Plane& pl, const Ray& r);

        static bool do_intersect_cl(const Plane& pl, const Segment& s);

        static bool do_intersect(const Plane& pl, const Ray& r);

        static bool do_intersect(const Plane& pl, const Segment& s);

        static bool do_intersect(const Plane& pl, const Line& l);

        static bool do_contain(const Plane& pl, const Point& p);

        static bool do_contain(const Plane& pl, const Ray& r);

        static bool do_contain(const Plane& pl, const Segment& s);

        static bool is_parallel(const Plane& pl, const Line& l);
};

template<class K>
class Arrangement_constructions_3 {
    public:
        typedef K Kernel;

        typedef typename Kernel::FT FT;

        typedef typename Kernel::Line_3 Line;
        typedef typename Kernel::Object_3 Object;
        typedef typename Kernel::Plane_3 Plane;
        typedef typename Kernel::Point_3 Point;

    public:
        static FT squared_distance(const Plane& pl, const Point& p);

        static Point intersection(const Plane& pl, const Line& l);

        static Line intersection(const Plane& pl1, const Plane& pl2);

        static Point intersection(const Plane& pl1, const Plane& pl2, const Plane& pl3);
};




template<class K = CGAL::Exact_predicates_exact_constructions_kernel,
		class K_d = CGAL::Homogeneous_d<typename K::FT> >
class Arrangement_3 {
public:
	typedef K Kernel;

	typedef typename Kernel::FT FT;
	typedef typename Kernel::RT RT;

	typedef typename Kernel::Object_3 Object;
	typedef typename Kernel::Point_3 Point;
	typedef typename Kernel::Vector_3 Vector;
	typedef typename Kernel::Line_3 Line;
	typedef typename Kernel::Ray_3 Ray;
	typedef typename Kernel::Segment_3 Segment;
	typedef typename Kernel::Plane_3 Plane;
	typedef typename Kernel::Iso_cuboid_3 Iso_cuboid;

	struct Plane_handle;

public:
	struct Face_handle {
	public:



		Face_handle();

		Face_handle(int i);

		bool operator==(Face_handle fh) const;

		operator int() const;


	protected:
		int _i;
	};




public:
	typedef Arrangement_face_base_3<K,
			Face_handle> Face;

protected:
	typedef typename Kernel::Aff_transformation_3 Aff_transformation;

	typedef Arrangement_constructions_3<K> Constructions;
	typedef Arrangement_predicates_3<K> Predicates;

	enum { _dimension = 3 };
	enum { _number_of_face_lists = _dimension + 3 };
	enum { _number_of_bbox_planes = 2 * _dimension };
	enum { _number_of_bbox_vertices = 8 };

	typedef std::vector<Plane> Plane_list;
	typedef std::vector<Face> Face_list;

public:
	typedef typename Plane_list::const_iterator Planes_const_iterator;

	typedef typename Face_list::iterator Faces_iterator;
	typedef typename Face_list::const_iterator Faces_const_iterator;

	typedef typename Face::Superfaces_iterator Superfaces_iterator;
	typedef typename Face::Superfaces_const_iterator
			Superfaces_const_iterator;

	typedef typename Face::Subfaces_iterator Subfaces_iterator;
	typedef typename Face::Subfaces_const_iterator
			Subfaces_const_iterator;

public:
	struct Plane_handle {
	public:
		Plane_handle();

		Plane_handle(int i);

		bool operator==(Plane_handle plh) const;

		operator int() const;




	protected:
		int _i;
	};

protected:
	typedef std::vector<Face_handle> Face_handle_list;

	typedef typename Face::Superface_list Superface_list;
	typedef typename Face::Subface_list Subface_list;

	typedef std::vector<Plane_handle> Plane_handle_list;

protected:
	Face_list _face_lists[_number_of_face_lists];
	Plane_handle_list _plane_handles;
	Plane_list _planes;

	bool _has_bbox;

	std::set<Face_handle> _split_ch; 
public:

	int plane_number_before_ghosts;
	std::vector<bool> consistent_orientation;
	std::vector<double> ref_a;
	std::vector<double> ref_b;
	std::vector<double> ref_c;
	std::vector<double> ref_d;

	bool save(const std::string& filename);

	bool load(const std::string& filename);
public:
	void set_bbox(const Point& p, const Vector& u, const Vector& v, const Vector& w);

	void set_bbox(const CGAL::Bbox_3& bbox);

	void set_bbox(double xmin, double ymin, double zmin, double xmax, double ymax, double zmax);

	Arrangement_3();

	bool insert(const Plane& pl);

	template<class InputIterator>
	int insert(InputIterator first, InputIterator last);

	static int dimension();

	static int number_of_bbox_planes();

	static int number_of_bbox_vertices();

	void clear();

	static int maximum_number_of_faces(int k, int n);

	bool has_bbox() const;


	void insert_splittable_cell(Face_handle ch);
	void clear_splittable_cell();

protected:
	void _split_cell_begin(Face_handle ch);

	void _split_cell_end();

	bool _is_face_splittable(int k, Face_handle fh) const;

	void _clear_face_lists();

	void _clear_plane_handles();

	void _clear_planes();

	static int _binomial(int n, int k);

protected:
	enum Color {
		WHITE = 0, 
		PINK = 1, 
		RED = 2, 
		CRIMSON = 3, 
		GREEN = 4, 
		GREY = 5, 
		BLACK = 6, 
	};

protected:
	static const char* _color_to_str(Color color);

public:

	Faces_iterator faces_begin(int k);

	Faces_const_iterator faces_begin(int k) const;

	Faces_iterator faces_end(int k);

	Faces_const_iterator faces_end(int k) const;

	int number_of_faces(int k) const;

	const Face& face(int k, int i) const;

	Face& face(int k, int i);

	Face_handle face_handle(int k, const Face& f) const;

	Faces_const_iterator cells_begin() const;

	Faces_iterator cells_begin();

	Faces_const_iterator cells_end() const;

	Faces_iterator cells_end();

	int number_of_cells() const;

	const Face& cell(int i) const;

	Face& cell(int i);

	Face_handle cell_handle(const Face& c) const;

	Face_handle cell_neighbor(const Face& c, int i) const;

	Face_handle cell_neighbor(Face_handle ch, int i) const;

	Faces_const_iterator facets_begin() const;

	Faces_iterator facets_begin();

	Faces_const_iterator facets_end() const;

	Faces_iterator facets_end();

	int number_of_facets() const;

	const Face& facet(int i) const;

	Face& facet(int i);

	Face_handle facet_handle(const Face& f) const;

	Plane_handle facet_plane(int i) const;

	Plane_handle facet_plane(const Face& f) const;

	Faces_const_iterator edges_begin() const;

	Faces_iterator edges_begin();

	Faces_const_iterator edges_end() const;

	Faces_iterator edges_end();

	int number_of_edges() const;

	const Face& edge(int i) const;

	Face& edge(int i);

	Face_handle edge_handle(const Face& e) const;

	Faces_const_iterator vertices_begin() const;

	Faces_iterator vertices_begin();

	Faces_const_iterator vertices_end() const;

	Faces_iterator vertices_end();

	int number_of_vertices() const;

	const Face& vertex(int i) const;

	Face& vertex(int i);

	Face_handle vertex_handle(const Face& v) const;

	Planes_const_iterator planes_begin() const;

	Planes_const_iterator planes_end() const;

	const Plane& plane(Plane_handle plh) const;

	Plane_handle plane_handle(const Plane& pl) const;

	int number_of_planes() const;

	bool is_bbox_facet(Face_handle fh) const;

	bool is_bbox_facet(const Face& f) const;

protected:
	enum { _empty_face_handle = 0 };

	enum { _complete_face_handle = 0 };

protected:
	const Face_list& _face_list(int k) const;

	Face_list& _face_list(int k);

	const Face& _face(int k, int i) const;

	Face& _face(int k, int i);

	const Face& _empty_face() const;

	Face& _empty_face();

	const Face& _complete_face() const;

	Face& _complete_face();

	Planes_const_iterator _find_plane(const Plane& pl) const;

	void _insert_bbox_planes(const CGAL::Bbox_3& bbox);

	void _insert_bbox_planes(double xmin, double ymin, double zmin,
			double xmax, double ymax, double zmax);

	void _insert_bbox_planes(const Point& p,
			const Vector& u,
			const Vector& v,
			const Vector& w);

	virtual bool _insert_plane(const Plane& pl);

	bool _has_bbox_vertices_on_both_sides(const Plane& pl);

	template<class InputIterator>
	int _insert_planes(InputIterator first, InputIterator last);

	void _update_arrangement(int m);

	void _compute_initial_arrangement(const Plane_handle* plhs);

	static int _count_mismatch(int n, const int* h1, const int* h2);

	static int _count_non_zero(int n, const int* h);

	static int _find_zero(int n, const int* h);

	static int _find_non_zero(int n, const int* h);

public:
	const Point& point(const Face& f) const;

	const Point& point(Face_handle vh) const;

	Line line(const Face& f) const;

	Line line(Face_handle eh) const;

	Ray ray(const Face& f) const;

	Ray ray(Face_handle eh) const;

	Segment segment(const Face& f) const;

	Segment segment(Face_handle eh) const;

	bool is_face_bounded(int k, const Face& f) const;

	bool is_cell_bounded(const Face& f) const;

	bool is_cell_bounded(Face_handle ch) const;

	bool is_facet_bounded(const Face& f) const;

	bool is_facet_bounded(Face_handle fh) const;

	bool is_edge_bounded(const Face& f) const;

	bool is_edge_bounded(Face_handle eh) const;

	template<class OutputIterator>
	bool facet_to_polygon(const Face& f, OutputIterator it) const;

	void dump_incidence_graph(std::ostream& stream) const;

protected:
	FT _squared_distance(const Plane& pl, const Face& v) const;

	bool _do_contain_vertex(const Plane& pl, const Face& v) const;

	bool _do_contain_vertex(const Plane& pl, Face_handle vh) const;

	bool _do_intersect_edge_cl(const Plane& pl, const Face& e) const;

	bool _do_intersect_edge_cl(const Plane& pl, Face_handle eh) const;

	bool _do_intersect_edge(const Plane& pl, const Face& e) const;

	bool _do_intersect_edge(const Plane& pl, Face_handle eh) const;

	bool _do_contain_edge(const Plane& pl, const Face& e) const;

	bool _do_contain_edge(const Plane& pl, Face_handle eh) const;

	bool _is_parallel(const Plane& pl, const Face& e) const;

	bool _is_parallel(const Plane& pl, Face_handle eh) const;

	const Face& _find_incident_edge_not_parallel(const Face& v,
			const Plane& pl) const;

	const Face& _find_intersecting_edge(const Plane& pl) const;

	Face_handle _find_incident_facet(const Face& e) const;

	void _mark_intersecting_0_1_faces(const Plane& pl, const Face& e0,
			Face_handle_list* L);

	int _oriented_side(const Plane& pl, const Face& f) const;

	bool _has_pink_subfaces_on_both_sides(int k, const Face& f,
			const Plane& pl) const;

	bool _has_a_red_subface(int k, const Face& f) const;

	bool _has_all_subfaces_crimson(int k, const Face& f) const;

	void _mark_intersecting_2_d_faces(const Plane& pl,
			Face_handle_list* L);

	void _mark_intersecting_faces(const Plane& pl, const Face& e0,
			Face_handle_list* L);

	bool _is_face_in_cell_cl(int k, Face_handle fh, Face_handle ch) const;

	void _update_marked_faces(Plane_handle plh, Face_handle_list* L);

	void _remove_g(int k, Face_handle gh, Face& g,
			Superface_list& g_superfaces,
			Subface_list& g_subfaces);

	Face_handle _replace_g_by_gm_and_gp(int k, Face_handle gh);

	Face_handle _create_f_and_connect_f_with_gm_and_gp(
			int k,
			Face_handle gmh, Face& gm,
			Face_handle gph, Face& gp,
			Plane_handle plh
	);

	void _connect_g_superfaces_with_gm_and_gp(
			int k,
			const Superface_list& g_superfaces,
			Face_handle gmh, Face& gm,
			Face_handle gph, Face& gp
	);

	void _connect_g_subfaces_with_gm_or_gp(
			int k,
			const Subface_list& g_subfaces,
			const Plane& pl,
			Face_handle gmh, Face& gm,
			Face_handle gph, Face& gp
	);

	void _connect_f_with_empty_face(Face_handle fh, Face& f);

	void _compute_gm_gp_points_segment(const Plane& pl,
			const Point& p,
			const Point& p1,
			const Point& p2,
			Face& gm,
			Face& gp);

	void _compute_gm_gp_points_ray(const Plane& pl,
			const Point& p,
			const Point& p1,
			const Point& p2,
			Face& gm,
			Face& gp);

	void _compute_f_gm_gp_points(const Plane& pl,
			const Line& g_line,
			const Point& g_point,
			const Subface_list& g_subfaces,
			Face& f,
			Face& gm,
			Face& gp);

	void _connect_f_with_subfaces(int k,
			Face_handle fh,
			Face& f,
			const Subface_list& g_subfaces);

	void _compute_f_point_ray(Face& f,
			const Plane& pl, Plane_handle plh,
			const Point& g_point,
			const Subface_list& g_subfaces);

	void _split_g(Plane_handle plh, int k, Face_handle gh,
			Face_handle_list* L,
			typename Face_handle_list::iterator& it, int i);

	void _unmark_faces(Face_handle_list* L);

	void _increment_arrangement(Plane_handle plh);

	void _dump_marked_faces(const Face_handle_list* L) const;

public:
	bool check_incidence_graph() const;

protected:
	bool _choose_initial_planes(Plane_handle* plhs);

	Point _compute_face_point(int k, int i) const;

protected:
	typedef K_d Kernel_d;
	typedef typename Kernel_d::Vector_d Vector_d;

protected:
	int _linear_rank(int n, const Plane_handle* plhs) const;
};

template<class T = double>
class Mesh_3 {
    public:
        typedef T value_type;
        typedef int info_type;

    public:
        struct Tuple_3 {
            public:
                Tuple_3();
     
                Tuple_3(value_type x,
                        value_type y,
                        value_type z);

                Tuple_3 operator+(const Tuple_3& v) const;

                Tuple_3 operator-(const Tuple_3& v) const;

                Tuple_3 operator/(value_type f) const;

                static Tuple_3 cross(const Tuple_3& v1, const Tuple_3& v2);

                static value_type dot(const Tuple_3& v1, const Tuple_3& v2);

            public:
                value_type x, y, z;
        };

        struct Facet {
            public:
                typedef int Vertex_handle;

                typedef std::vector<Vertex_handle>::const_iterator
                    Vertex_handles_const_iterator;
                typedef std::vector<Vertex_handle>::iterator
                    Vertex_handles_iterator;

            public:
                Facet();

                Facet(info_type info);

                bool is_bounded() const;

                bool& is_bounded();

                int number_of_vertex_handles() const;

                int number_of_vertices() const;

                Vertex_handles_const_iterator vertex_handles_begin() const;

                Vertex_handles_iterator vertex_handles_begin();

                Vertex_handles_const_iterator vertex_handles_end() const;

                Vertex_handles_iterator vertex_handles_end();

                Vertex_handle vertex_handle(int i) const;

                Vertex_handle& vertex_handle(int i);

                void insert(Vertex_handle vh);

                bool has_vertex_handle(Vertex_handle vh) const;

                void flip_normal();

                info_type info() const;

                info_type& info();

            protected:
                std::vector<Vertex_handle> _vertex_handles;
                info_type _info;
                bool _is_bounded;
        };

    public:
        typedef typename std::vector<Tuple_3>::const_iterator
            Vertices_const_iterator;
        typedef typename std::vector<Tuple_3>::iterator
            Vertices_iterator;

        typedef typename Facet::Vertex_handle Vertex_handle;
        typedef typename Facet::Vertex_handles_const_iterator
            Vertex_handles_const_iterator;
        typedef typename Facet::Vertex_handles_iterator
            Vertex_handles_iterator;
        typedef int Facet_handle;

        typedef typename std::vector<Facet>::const_iterator
            Facets_const_iterator;
        typedef typename std::vector<Facet>::iterator
            Facets_iterator;

    public:
        Vertex_handle insert(const Tuple_3& v);

        Facet_handle insert(const Facet& f);

        void clear();

        Vertices_const_iterator vertices_begin() const;

        Vertices_iterator vertices_begin();

        Vertices_const_iterator vertices_end() const;

        Vertices_iterator vertices_end();

        int number_of_vertices() const;

        const Tuple_3& vertex(Vertex_handle vh) const;

        Tuple_3& vertex(Vertex_handle vh);

        const Tuple_3& vertex(Facet_handle fh, int i) const;

        Tuple_3& vertex(Facet_handle fh, int i);

        Vertex_handle vertex_handle(const Tuple_3& v) const;

        Facets_const_iterator facets_begin() const;

        Facets_iterator facets_begin();

        Facets_const_iterator facets_end() const;

        Facets_iterator facets_end();

        int number_of_facets() const;

        const Facet& facet(Facet_handle fh) const;

        Facet& facet(Facet_handle fh);

        Facet_handle facet_handle(const Facet& f) const;

        Tuple_3 compute_facet_normal(const Facet& f) const;

        Tuple_3 compute_facet_normal(Facet_handle fh) const;

        void flip_facet_normal(Facet& f);

        void flip_facet_normal(Facet_handle fh);

    protected:
        std::vector<Tuple_3> _vertices;
        std::vector<Facet> _facets;
};

struct Cells_binary_labeler_tag { };

struct Facets_binary_labeler_tag { };

struct Cells_multi_labeler_tag { };

struct Facets_multi_labeler_tag { };

template<class Comp,
         class Mesh>
class Mesh_extractor_3 {
    protected:
        typedef typename Comp::Face Face;
        typedef typename Comp::Face_handle Face_handle;

    protected:
        typedef typename Comp::Faces_const_iterator Faces_const_iterator;
        typedef typename Comp::Plane_handle Plane_handle;
        typedef typename Comp::Plane Plane;
        typedef typename Comp::Point Point;
        typedef typename Comp::Vector Vector;

        typedef typename Mesh::Tuple_3 Tuple_3;
        typedef typename Mesh::Facet Mesh_facet;
        typedef typename Mesh::Vertex_handle Mesh_vertex_handle;
        typedef typename Mesh::Facet_handle Mesh_facet_handle;
        typedef typename Mesh::info_type info_type;

    public:
        Mesh_extractor_3(const Comp& comp);

        void extract(Mesh& mesh,
                     bool keep_bounded_bbox_facet = false) const;

        template<class Labeler>
        void extract(Mesh& mesh,
                     const Labeler& labeler,
                     bool keep_bounded_bbox_facet = false) const;

    protected:
        void _extract(Mesh& mesh,
                      bool keep_bounded_bbox_facet) const;

        template<class Labeler>
        void _extract(Mesh& mesh,
                      const Labeler& labeler,
                      bool keep_bounded_bbox_facet,
                      Cells_binary_labeler_tag tag) const;

        template<class Labeler>
        void _extract(Mesh& mesh,
                      const Labeler& labeler,
                      bool keep_bounded_bbox_facet,
                      Facets_multi_labeler_tag tag) const;

        template<class Labeler>
        void _extract(Mesh& mesh,
                      const Labeler& labeler,
                      bool keep_bounded_bbox_facet,
                      Facets_binary_labeler_tag tag) const;

    protected:
        const Comp& _comp;
};

template<class Mesh>
class Mesh_3_writer {
    protected:
        typedef typename Mesh::value_type value_type;

        typedef typename Mesh::Tuple_3 Tuple_3;
        typedef typename Mesh::Vertices_const_iterator
            Vertices_const_iterator;
        typedef typename Mesh::Vertex_handle Vertex_handle;

        typedef typename Mesh::Facet Facet;
        typedef typename Mesh::Facets_const_iterator
            Facets_const_iterator;
        typedef typename Mesh::Facet_handle Facet_handle;

        typedef typename Mesh::Vertex_handles_const_iterator
            Vertex_handles_const_iterator;

    public:
        Mesh_3_writer(const Mesh& mesh,
                      bool triangulate = false);

        void write(std::ostream& stream) const;

    protected:
        virtual void _write_header(std::ostream& stream, int number_of_vertices, int number_of_facets) const = 0;

        virtual void _write_begin(std::ostream& stream) const;

        virtual void _write_vertices_begin(std::ostream& stream, int number_of_vertices) const;

        virtual void _write_vertex_begin(std::ostream& stream, Vertex_handle vh) const;

        virtual void _write_vertex(std::ostream& stream, Vertex_handle vh, value_type x, value_type y, value_type z) const;

        virtual void _write_vertex_end(std::ostream& stream, Vertex_handle vh) const;

        virtual void _write_vertices_end(std::ostream& stream) const;

        virtual void _write_facets_begin(std::ostream& stream, int number_of_facets) const;

        virtual void _write_facet_begin(std::ostream& stream, Facet_handle fh, int number_of_indices) const;

        virtual void _write_facet_vertex_index(std::ostream& stream, Facet_handle fh, int index) const;

        virtual void _write_facet_end(std::ostream& stream, Facet_handle fh) const;

        virtual void _write_facets_end(std::ostream& stream) const;

        virtual void _write_end(std::ostream& stream) const;

    protected:
        const Mesh& _mesh;
        const bool _triangulate;
};

template<class Mesh,
         class Colormap>
class Mesh_with_facet_info_3_writer_PLY : public Mesh_3_writer<Mesh> {
    protected:
        typedef Mesh_3_writer<Mesh> Base;
        typedef typename Base::Facet Facet;
        typedef typename Base::Vertex_handle Vertex_handle;
        typedef typename Base::Facet_handle Facet_handle;
        typedef typename Mesh::info_type info_type;

        typedef typename Colormap::Color Color;

    public:
        Mesh_with_facet_info_3_writer_PLY(const Mesh& mesh, const Colormap& colormap);

    protected:
        void _write_header(std::ostream& stream, int number_of_vertices, int number_of_facets) const;

        void _write_facet_begin(std::ostream& stream, Facet_handle fh, int number_of_vertices) const;

        void _write_facet_end(std::ostream& stream, Facet_handle fh) const;

    protected:
        const Colormap& _colormap;
};

template<class Mesh>
class Mesh_3_writer_PLY : public Mesh_3_writer<Mesh> {
    protected:
        typedef Mesh_3_writer<Mesh> Base;
        typedef typename Base::Facet_handle Facet_handle;

    public:
        Mesh_3_writer_PLY(const Mesh& mesh);

    protected:
        void _write_header(std::ostream& stream, int number_of_vertices, int number_of_facets) const;

        void _write_facet_begin(std::ostream& stream, Facet_handle fh, int number_of_vertices) const;
};

template<class Mesh>
void print_mesh_PLY(std::ostream& stream,
                    const Mesh& mesh);

template<class Mesh,
         class Colormap>
void print_mesh_with_facet_color_PLY(std::ostream& stream,
                                     const Mesh& mesh,
                                     const Colormap& colormap);

template<class Comp>
typename Comp::Face_handle find_bounded_cell(const Comp& comp);

template<class Comp>
typename Comp::Face_handle find_unbounded_cell(const Comp& comp);

template<class Comp>
bool do_intersect_facet_cl(const Comp& comp,
                           const typename Comp::Segment& s,
                           typename Comp::Face_handle fh);

template<class K>
int find_max_coord(const typename K::Plane_3& pl);

template<class Comp>
bool do_intersect_facet_cl2(const Comp& comp,
                            const typename Comp::Segment& s,
                            typename Comp::Face_handle fh);

template<class Comp>
bool is_inside(const Comp& comp,
               const typename Comp::Point& q,
               const typename Comp::Face& c);

template<class Comp>
typename Comp::Face_handle
find_containing_cell(const Comp& comp,
                     const typename Comp::Point& q,
                     typename Comp::Face_handle ch0 =
                         typename Comp::Face_handle());

template<class Comp>
typename Comp::Face_handle
find_facet(const Comp& comp, const typename Comp::Face& c, typename Comp::Plane_handle plh);

} 

#endif /* POLYHEDRAL_COMPLEX_3_ARRANGEMENT_3_HPP */

