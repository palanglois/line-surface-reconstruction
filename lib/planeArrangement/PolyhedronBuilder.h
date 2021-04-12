#ifndef LINE_BASED_RECONS_REFACTO_POLYHEDRONBUILDER_H
#define LINE_BASED_RECONS_REFACTO_POLYHEDRONBUILDER_H

#include <cgalTypes.h>
#include <map>

#include<CGAL/Polyhedron_incremental_builder_3.h>
#include<CGAL/Polyhedron_3.h>
#include<CGAL/IO/Polyhedron_iostream.h>
#include <CGAL/Polyhedron_items_with_id_3.h>

typedef CGAL::Polyhedron_3<Kernel, CGAL::Polyhedron_items_with_id_3> Polyhedron;
typedef Polyhedron::HalfedgeDS HDS;

class PolyhedronBuilder : public CGAL::Modifier_base<HDS>
{
// A modifier creating a triangle with the incremental builder.
public:
    PolyhedronBuilder(const std::map<int, Point>& _allVertice,
                      const std::vector<std::vector<size_t>>& _facetsAsVertexIndice) :
            allVertice (_allVertice), facetsAsVertexIndice(_facetsAsVertexIndice)
    {

    }

    void operator()(HDS &hds) override {
        // Create a cgal incremental builder
        CGAL::Polyhedron_incremental_builder_3<HDS> B(hds, true);
        B.begin_surface(allVertice.size(), facetsAsVertexIndice.size());

        //Add the vertice
        for (const auto &i : facetsAsVertexIndice)
            for (const size_t & j : i)
                B.add_vertex(allVertice.at(j));

        //Add the polyhedron
        size_t accumulator = 0;
        for (const auto &i : facetsAsVertexIndice) {
            B.begin_facet();
            for (size_t j = 0; j < i.size(); j++)
            {
                B.add_vertex_to_facet(accumulator);
                accumulator++;
            }
            B.end_facet();
        }

        // Finish up the surface
        B.end_surface();
    }
private:
    const std::map<int, Point>& allVertice;
    const std::vector<std::vector<size_t>>& facetsAsVertexIndice;

};


#endif //LINE_BASED_RECONS_REFACTO_POLYHEDRONBUILDER_H
