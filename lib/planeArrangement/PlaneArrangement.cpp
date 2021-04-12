#include "PlaneArrangement.h"

using namespace std;

PlaneArrangement::PlaneArrangement(const CGAL::Bbox_3 &_bbox, const PrimitiveSet &_myPrimitiveSet, double _voxelSize,
                                   bool _verbose) :
        myPrimitiveSet(_myPrimitiveSet),
        bbox(_bbox),
        voxelSize(_voxelSize),
        verbose(_verbose),
        plane_handle_to_prim(map<int, int>()),
        prim_to_plane_handle(map<int, int>()),
        facetToCells(map<int, OrientedCellsHandles>()),
        intersectionTree(nullptr),
        polyhedron(nullptr)
{
    // Set the bounding box to the internal set
    set_bbox(bbox);

    //Insert the voxel planes
    insertVoxelPlanes();

    //Insert the primitives
    insertPlanes(myPrimitiveSet);

    if (verbose) {
        cout << "Constructed plane arrangement bounding box : " << bbox;
        if (voxelSize != 0.)
            cout << " and with voxels of size : " << voxelSize;
        cout << endl;
    }
}

PlaneArrangement::~PlaneArrangement() {
    delete intersectionTree;
    delete polyhedron;
}

void PlaneArrangement::insertVoxelPlanes() {
    // Inserting the Voxel planes
    if (voxelSize <= 0)
        return;
    if (verbose)
        cout << "Beginning insertion of the voxels." << endl;
    // Insert along the x axis
    for (double x = bbox.xmin() + voxelSize; x < bbox.xmax(); x += voxelSize)
        insert(Plane(1, 0, 0, -x));
    // Insert along the y axis
    for (double y = bbox.ymin() + voxelSize; y < bbox.ymax(); y += voxelSize)
        insert(Plane(0, 1, 0, -y));
    // Insert along the z axis
    for (double z = bbox.zmin() + voxelSize; z < bbox.zmax(); z += voxelSize)
        insert(Plane(0, 0, 1, -z));
    if (verbose)
        cout << "Insertion of the voxels finished." << endl;

}

void PlaneArrangement::insertPlanes(const PrimitiveSet &myPrimitiveSet) {
    if (verbose)
        cout << "Beginning insertion of the primitives." << endl;
    for (size_t i = 0; i < myPrimitiveSet.size(); i++) {
        //Actually insert the plane
        insert(myPrimitiveSet[i].toCgalPlane());

        //Get the plane handle and update the maps
        Plane_handle plh((planes_end() - planes_begin()) - 1);
        plane_handle_to_prim[plh] = static_cast<int>(i);
        prim_to_plane_handle[i] = plh;
        if(verbose)
            cout << "Inserted " << i + 1 << " primitives out of " << myPrimitiveSet.size() << "." << endl;
    }
    if (verbose) {
        cout << "Insertion of the primitives finished." << endl;
        cout << "Building the intersection tree..." << endl;
    }
    buildTree(); // Build the intersection tree;
    if(verbose)
        cout << "Intersection tree computed !" << endl;
}

void PlaneArrangement::saveArrangementAsPly(string fileName) {
    // Mark all facet to be drawn
    for (Faces_iterator itf = facets_begin(); itf != facets_end(); itf++) {
        Face &f = *itf;
        f.to_draw = false;
        if (!is_facet_bounded(f)) { continue; }
        f.to_draw = true;
    }

    // Save the drawn faces
    typedef Polyhedral_complex_3::Arrangement_3<Kernel> RawComplex;
    typedef Polyhedral_complex_3::Mesh_3<> Mesh;
    typedef Polyhedral_complex_3::Mesh_extractor_3<RawComplex, Mesh> Extractor;
    Mesh meshGC;
    Extractor extractorGC(*this);
    extractorGC.extract(meshGC, false);
    {
        std::ofstream stream(fileName.c_str());
        if (!stream.is_open())
            return;
        Polyhedral_complex_3::print_mesh_PLY(stream, meshGC);
        stream.close();
    }

    if (verbose)
        cout << "Saved the plane arrangement at path : " << fileName << endl;
}

void PlaneArrangement::buildTree() {
    // Retrieve all the facets
    facetToCells.clear();
    vector<size_t> numberOfTrianglesPerFacet;
    vector<int> indexPerFacet;
    vector<vector<size_t>> facetsAsVertexIndice;
    map<int, Point> allVertice;
    for (auto itf = facets_begin(); itf != facets_end(); itf++) {
        PlaneArrangement::Face &f = *itf;

        PlaneArrangement::Face_handle fh = facet_handle(f);

        // Test if facet on the bounding box
        if (!is_facet_bounded(fh)) { continue; }

        // Build references from facets id to the positive neighbour cell and the negative neighbour cell
        assert(f.number_of_superfaces() == 2);
        PlaneArrangement::Face &cell0 = cell(f.superface(0));
        PlaneArrangement::Face &cell1 = cell(f.superface(1));
        if (!is_cell_bounded(f.superface(0)) || !is_cell_bounded(f.superface(1))) continue;
        const Primitive &prim = myPrimitiveSet[plane_handle_to_prim[facet_plane(facet_handle(f))]];
        // Check which cell is on the positive side of the current facet
        bool orientationPos0 = (cell0.point() - prim.getInlier()) * (prim.getNormal()) >= 0;
        assert((orientationPos0 && !((cell1.point() - prim.getInlier()) * (prim.getNormal()) >= 0)) ||
               (!orientationPos0 && ((cell1.point() - prim.getInlier()) * (prim.getNormal()) >= 0)));
        OrientedCellsHandles currentOrientation = {false, f.superface(1), f.superface(0), {-1, -1}, fh};
        if (orientationPos0) {
            currentOrientation.positive = f.superface(0);
            currentOrientation.negative = f.superface(1);
        }
        facetToCells[fh] = currentOrientation;

        // get the vertices of the facet
        std::vector<PlaneArrangement::Face_handle> vertices;
        facet_to_polygon(f, std::back_inserter(vertices));
        unsigned long num_hs = vertices.size();
        numberOfTrianglesPerFacet.push_back(num_hs - 2);

        vector<size_t> currentFacetIndice(vertices.begin(), vertices.end());
        for (auto &vertice : vertices) {
            int curIndex = vertice;
            if (allVertice.find(curIndex) == allVertice.end()) {
                // not found
                allVertice[curIndex] = point(vertice);
            }
        }
        //Triangulate the facet
        vector<vector<size_t>> trianglesFromCurrentFacet;
        for (size_t i = 1; i < currentFacetIndice.size() - 1; i++) {
            vector<size_t> currentTriangle = {currentFacetIndice[0], currentFacetIndice[i], currentFacetIndice[i + 1]};
            trianglesFromCurrentFacet.push_back(currentTriangle);
        }
        facetsAsVertexIndice.insert(facetsAsVertexIndice.end(),
                                    trianglesFromCurrentFacet.begin(), trianglesFromCurrentFacet.end());
        indexPerFacet.push_back(facet_handle(f));
    }

    // Delete former intersection tree
    delete intersectionTree;
    delete polyhedron;

    // Build a polyhedron from the loaded arrays
    polyhedron = new Polyhedron();
    PolyhedronBuilder builder(allVertice, facetsAsVertexIndice);
    polyhedron->delegate(builder);    // assign id field for each facet

    // Label the triangles according to the original facet they come from
    int facesIndex = 0;
    size_t curFaceIt = 0;
    for (Polyhedron::Facet_iterator facet_it = polyhedron->facets_begin();
         facet_it != polyhedron->facets_end(); ++facet_it) {
        facet_it->id() = indexPerFacet[facesIndex];

        curFaceIt++;
        if (curFaceIt == numberOfTrianglesPerFacet[facesIndex]) {
            facesIndex++;
            curFaceIt = 0;
        }


    }

    // constructs AABB tree
    intersectionTree = new Tree(faces(*polyhedron).first, faces(*polyhedron).second, *polyhedron);
    intersectionTree->accelerate_distance_queries();

}

set<int> PlaneArrangement::getListOfIntersectedFacets(Segment segmentQuery) const
{
    //Make intersections
    std::list<Segment_intersection> intersections;
    intersectionTree->all_intersections(segmentQuery, std::back_inserter(intersections));

    set<int> facetIds;
    for (auto &segmentIntersection : intersections) {
        if (Segment *facet_segment = boost::get<Segment>(&(segmentIntersection)->first)) {
            facetIds.insert(segmentIntersection->second->id());
        }
    }
    return facetIds;
}

IntersectionsOutput PlaneArrangement::intersectWithSegment(Segment segmentQuery) const {
    //Make intersections
    std::list<Segment_intersection> intersections;
    intersectionTree->all_intersections(segmentQuery, std::back_inserter(intersections));
    return postProcessIntersections<Segment_intersection>(intersections);
}

IntersectionsOutput PlaneArrangement::intersectWithTriangle(Triangle triangleQuery) const {
    //Make intersections
    std::list<Triangle_intersection> intersections;
    intersectionTree->all_intersections(triangleQuery, std::back_inserter(intersections));
    return postProcessIntersections<Triangle_intersection>(intersections, false);
}

template<class T>
IntersectionsOutput PlaneArrangement::postProcessIntersections(std::list<T> intersections, bool processPoints) const {
    // Differenciate Point intersections and Segment intersections
    IntersectionsOutput intersectionsOutput;
    vector<int> segmentIds;
    vector<int> pointIds;
    for (auto &pt_intersection : intersections) {
        if (Point *facet_point = boost::get<Point>(&(pt_intersection)->first)) {
            pointIds.push_back(pt_intersection->second->id());
            intersectionsOutput.points.push_back(*facet_point);
        } else if (Segment *facet_segment = boost::get<Segment>(&(pt_intersection)->first)) {
            OrientedCellsHandles neighbourCells = facetToCells.at(pt_intersection->second->id());
            segmentIds.push_back(pt_intersection->second->id());
            intersectionsOutput.segments.push_back({*facet_segment, neighbourCells});
        }
    }
    mergeSegments(intersectionsOutput.segments, segmentIds);
    if (processPoints) mergePoints(intersectionsOutput.points);
    return intersectionsOutput;
}

template<class T, class U>
void PlaneArrangement::removeDuplicates(vector<T> &array) const {
    // Merge points with same coordinates and same ids
    unordered_map<T, vector<size_t>, U> elemsIdsToIndex;
    for (size_t i = 0; i < array.size(); i++) {
        if (elemsIdsToIndex.find(array[i]) == elemsIdsToIndex.end()) {
            // not found
            elemsIdsToIndex[array[i]] = {i};
        } else {
            // found
            elemsIdsToIndex[array[i]].push_back(i);
        }
    }
    vector<T> newArray;
    newArray.reserve(array.size());
    for (auto &it: elemsIdsToIndex)
        newArray.push_back(it.first);
    array = newArray;
}

void PlaneArrangement::removeDuplicatesSegment(vector<SegmentWithNeighbourCells> &array) const {
    // Instead of having several instances of the same segment for each facet, we make one instance with a link
    // to the neighbour cells
    // For each segment instance, we store all the neighbours
    unordered_map<SegmentWithNeighbourCells, vector<OrientedCellsHandles>, SegmentWithNeighbourCellsHash> elemsIdsToIndex;
    for (auto &segmentWithNeighbour : array) {
        if (elemsIdsToIndex.find(segmentWithNeighbour) == elemsIdsToIndex.end()) {
            // not found
            elemsIdsToIndex[segmentWithNeighbour] = {segmentWithNeighbour.neighbourCells};
        } else {
            // found
            elemsIdsToIndex[segmentWithNeighbour].push_back(segmentWithNeighbour.neighbourCells);
        }
    }
    vector<SegmentWithNeighbourCells> newArray;
    newArray.reserve(array.size());
    for (auto &it: elemsIdsToIndex) {
        // We discard intersections of more than 2 planes
        if(it.second.size() > 4)
        {
            cout << "INFO : intersection of more than 2 planes detected." << endl;
            continue;
        }
        assert(it.second.size() == 1 || it.second.size() == 4);
        if (it.second.size() == 4) {
            // The segment is on the edge of a facet
            // We compute the neighbour cells :
            // There is a ++ cell, a -- cell and 2 +- cells
            map<int, pair<int, int>> facetToPosNeg; // facet -> <nbTimesPositive, nbTimesNegative>
            for (auto &cells: it.second) {
                if (facetToPosNeg.find(cells.positive) == facetToPosNeg.end())
                    facetToPosNeg[cells.positive] = {1, 0};
                else
                    facetToPosNeg[cells.positive].first += 1;
                if (facetToPosNeg.find(cells.negative) == facetToPosNeg.end())
                    facetToPosNeg[cells.negative] = {0, 1};
                else
                    facetToPosNeg[cells.negative].second += 1;
            }
            OrientedCellsHandles neighbours = {true, -1, -1, {-1, -1}, -1};
            vector<int> neutralNeighbour;
            for (auto &facet: facetToPosNeg) {
                if (facet.second.first == 2)
                    neighbours.positive = facet.first;
                else if (facet.second.second == 2)
                    neighbours.negative = facet.first;
                else
                    neutralNeighbour.push_back(facet.first);
            }
            assert(neutralNeighbour.size() == 2);
            neighbours.neutral[0] = neutralNeighbour[0];
            neighbours.neutral[1] = neutralNeighbour[1];
            newArray.push_back({it.first.segment, neighbours});
        } else {
            // The segment is inside a facet
            newArray.push_back(it.first);
        }
    }
    array = newArray;
}

void PlaneArrangement::mergePoints(vector<Point> &points) const {
    removeDuplicates<Point, PointHash>(points);
}

void PlaneArrangement::mergeSegments(vector<SegmentWithNeighbourCells> &segments, const vector<int> &segmentIds) const {

    //Store each segment by its facet ID
    map<int, vector<SegmentWithNeighbourCells>> facetIdToSegments;
    for (size_t i = 0; i < segments.size(); i++) {
        if (facetIdToSegments.find(segmentIds[i]) == facetIdToSegments.end()) {
            // not found
            facetIdToSegments[segmentIds[i]] = {segments[i]};
        } else {
            // found
            // Add the segment only if it is not yet in the vector
            vector<SegmentWithNeighbourCells>& curDictArray = facetIdToSegments[segmentIds[i]];
            if (find(curDictArray.begin(), curDictArray.end(), segments[i]) == curDictArray.end())
                curDictArray.push_back(segments[i]);
        }
    }
    vector<SegmentWithNeighbourCells> mergedSegments;
    for (auto &facetIdToSegment : facetIdToSegments) {
        vector<SegmentWithNeighbourCells> &facetSegments = facetIdToSegment.second;

        // If there is only 1 segment in the current facet, it stays as is
        if (facetSegments.size() == 1) {
            mergedSegments.push_back(facetSegments[0]);
            continue;
        }

        // Otherwise, we merge them
        map<Point, int> nbOfPointOccurences;
        vector<Point> allPoints;
        // Gathering all points
        for (auto &curSegment: facetSegments) {
            allPoints.push_back(curSegment.segment.source());
            allPoints.push_back(curSegment.segment.target());
        }
        // Finding the number of occurences for each of them
        for (auto &point: allPoints) {
            if (nbOfPointOccurences.find(point) == nbOfPointOccurences.end()) {
                // not found
                nbOfPointOccurences[point] = 1;
            } else {
                // found
                nbOfPointOccurences[point]++;
            }
        }
        vector<Point> segEndPoints(2);
        // The points that appear only once are the segment's endpoints
        int cursor = 0;
        for (auto &nbOfPointOccurence : nbOfPointOccurences) {
            if (nbOfPointOccurence.second == 1) {
                assert(cursor < 2);
                segEndPoints[cursor] = nbOfPointOccurence.first;
                cursor++;
            }
        }
        assert(cursor == 2);
        mergedSegments.push_back({Segment(segEndPoints[0], segEndPoints[1]), facetSegments[0].neighbourCells});
    }
    removeDuplicatesSegment(mergedSegments);
    segments = mergedSegments;
}

const Polyhedron &PlaneArrangement::getPolyhedron() const {
    assert(polyhedron);
    return *polyhedron;
}

/** \brief compute area of a facet in the arrangement*/
double PlaneArrangement::facet_area(const PlaneArrangement::Face& f) const
{

    std::vector<PlaneArrangement::Face_handle> vertices;
    if ( !facet_to_polygon(f,std::back_inserter(vertices)) )
        return 0; // Area is undefined for infinite facets

    double area = 0;

    if ( vertices.size() < 3 )
        return area;

    std::vector<PlaneArrangement::Face_handle>::const_iterator vhi = vertices.begin();
    PlaneArrangement::Point first = point(*vhi);
    ++vhi;
    PlaneArrangement::Point second = point(*vhi);
    ++vhi;
    while ( vhi != vertices.end() )
    {
        PlaneArrangement::Point third = point(*vhi);
        area += sqrt( CGAL::to_double(CGAL::squared_area(first,second,third) ));
        second = third;
        ++vhi;
    }
    return area;
}

int PlaneArrangement::getPlaneHandleFromPrimitiveId(int i) const
{
    assert(prim_to_plane_handle.find(i) != prim_to_plane_handle.end());
    return prim_to_plane_handle.at(i);
}

void PlaneArrangement::savePlyFromLabel(const string &filename, map<int, int> &fh_to_node, const vector<size_t> &labels)
{

    for(auto itf = facets_begin(); itf != facets_end(); itf++){
        PlaneArrangement::Face& f = *itf;
        itf->to_draw = false;
        if(! is_facet_bounded(f)){continue;}
        PlaneArrangement::Face_handle ch0 = f.superface(0), ch1 = f.superface(1);
        //if(!(is_cell_bounded(ch0) && is_cell_bounded(ch1))){continue;}
        if(fh_to_node.count((int)ch0) ==0 || fh_to_node.count((int)ch1) == 0){continue;}
        if(labels[fh_to_node[int(ch0)]] != labels[fh_to_node[int(ch1)]]){
            f.to_draw = true;
        }
    }

    typedef Polyhedral_complex_3::Arrangement_3<Kernel> RawComplex;
    typedef Polyhedral_complex_3::Mesh_3<> Mesh;
    typedef Polyhedral_complex_3::Mesh_extractor_3<RawComplex,Mesh> Extractor;
    Mesh meshGC;
    Extractor extractorGC(*this);
    extractorGC.extract(meshGC,false);
    {
        std::ofstream stream(filename.c_str());
        if (!stream.is_open())
            return ;
        Polyhedral_complex_3::print_mesh_PLY(stream, meshGC);
        stream.close();
    }
}

void PlaneArrangement::saveArrPlyFullCell(const string &filename)
{

    // mark all facet to be drawn
    for(auto itf = facets_begin(); itf != facets_end(); itf++){
        PlaneArrangement::Face& f = *itf;
        f.to_draw = false;
        if(! is_facet_bounded(f)){continue;}
        for(int i=0; i<2; i++){
            if (cell(f.superface(i)).cell_value_full_points > cell(f.superface(i)).cell_value_void_points) {
                f.to_draw = true;
                f.info() = int(floor(2 * (double(rand()) / double(RAND_MAX))));
                break;
            }
        }
    }

    typedef Polyhedral_complex_3::Arrangement_3<Kernel> RawComplex;
    typedef Polyhedral_complex_3::Mesh_3<> Mesh;
    typedef Polyhedral_complex_3::Mesh_extractor_3<RawComplex, Mesh> Extractor;
    Mesh meshGC;
    Extractor extractorGC(*this);
    extractorGC.extract(meshGC,false);
    {
        std::ofstream stream(filename.c_str());
        if (!stream.is_open())
            return ;
		Polyhedral_complex_3::print_mesh_PLY(stream, meshGC);
        stream.close();
    }
}


void PlaneArrangement::insertTexturialCost(pair<int, int> textCells, double cost)
{
    if(texturialCosts.find(textCells) != texturialCosts.end())
        texturialCosts[textCells] += cost;
    else
        texturialCosts[textCells] = cost;
}

const map<pair<int, int>, double>& PlaneArrangement::getTexturialCosts() const
{
    return texturialCosts;
}

void PlaneArrangement::insertStructuralCost(const TripletCell &structCells, double cost)
{
    if(structuralCosts.find(structCells) != structuralCosts.end())
        structuralCosts[structCells] += cost;
    else
        structuralCosts[structCells] = cost;
}

const map<TripletCell, double>& PlaneArrangement::getStructuralCosts() const
{
    return structuralCosts;
}
