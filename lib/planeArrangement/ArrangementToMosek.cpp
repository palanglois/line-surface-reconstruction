#include "ArrangementToMosek.h"

using namespace std;

ArrangementToMosek::ArrangementToMosek(const PlaneArrangement &_arrangement, double cost_primitive,
                                       double cost_visibility, double cost_area,
                                       double cost_edge, double cost_corner, bool _force_filled_bbox,
                                       bool _force_empty_bbox, bool _verbose) :
        Model("Linear Program"),
        arrangement(_arrangement), force_filled_bbox(_force_filled_bbox), force_empty_bbox(_force_empty_bbox),
        cp(cost_primitive), cv(cost_visibility), ca(cost_area), ce(cost_edge), cc(cost_corner), verbose(_verbose)
{

    // Check that given conditions are not contradictory
    if(force_empty_bbox && force_filled_bbox) cerr << "WARNING : you should not try to force the bounding box to "
                                                      "be simultaneously full and empty !"<< endl;

    // Set verbosity to maximum if required
    if(verbose) setLogHandler([=](const string & msg) { cout << msg << flush; } );
    if(verbose) cout << "Computing cell to LP variables mapping..." << endl;

    // Initializing the number of variables and the number of constraints
    nbVariables = arrangement.number_of_cells() // Actual variables related to cell fullness/emptiness
            + (int)arrangement.getTexturialCosts().size() // Additional variables related to texturial lines
            + (int)arrangement.getStructuralCosts().size() // Additional variables related to structural lines
            + 2*arrangement.number_of_facets() // Additional variables related to visibility and area penalization
            + arrangement.number_of_edges() // Additional variables related to edge length penalization
            + arrangement.number_of_vertices(); // Additional variables related to corner penalization
    nbConstraints = 2*nbVariables;

    // Initializing the variables and constraints
    variablesWeights = new ndarray<double, 1>(nbVariables, 0.);
    constraintsCoorAndWeights = vector<vector<pair<size_t, double>>>(0);
    constraintsCoorAndWeights.reserve((size_t)nbConstraints);
    constraintsUpperBounds = vector<double>(0);
    constraintsUpperBounds.reserve((size_t)nbConstraints);

    // Initializing mappings
    //  cells <-> variables
    int var_it = 0;
    for(auto cell_it = arrangement.cells_begin(); cell_it != arrangement.cells_end(); cell_it++)
    {
        cellToVariable[arrangement.cell_handle(*cell_it)] = var_it;
        variableToCell[var_it] = arrangement.cell_handle(*cell_it);
        var_it++;
    }
    // texturial pairs <-> variables
    for(auto pair_it: arrangement.getTexturialCosts())
    {
        texturialPairToVariable[pair_it.first] = var_it;
        variableToTexturialPair[var_it] = pair_it.first;
        var_it++;
    }
    // structural pairs <-> variables
    for(auto triplet_it: arrangement.getStructuralCosts())
    {
        structuralTripletToVariable[triplet_it.first] = var_it;
        variableToStructuralTriplet[var_it] = triplet_it.first;
        var_it++;
    }
    //  facets <-> variables (visibility, area)
    for(auto facet_it = arrangement.facets_begin(); facet_it != arrangement.facets_end(); facet_it++)
    {
        facetVisToVariable[arrangement.facet_handle(*facet_it)] = var_it;
        variableToFacetVis[var_it] = arrangement.facet_handle(*facet_it);
        var_it++;

        facetAreaToVariable[arrangement.facet_handle(*facet_it)] = var_it;
        variableTofacetArea[var_it] = arrangement.facet_handle(*facet_it);
        var_it++;
    }
    //  edges <-> variables (length)
    for(auto edge_it = arrangement.edges_begin(); edge_it != arrangement.edges_end(); edge_it++)
    {
        edgeToVariable[arrangement.edge_handle(*edge_it)] = var_it;
        variableToEdge[var_it] = arrangement.edge_handle(*edge_it);
        var_it++;
    }
    //  vertices <-> variables (corners)
    for(auto vert_it = arrangement.vertices_begin(); vert_it != arrangement.vertices_end(); vert_it++)
    {
        verticeToVariable[arrangement.vertex_handle(*vert_it)] = var_it;
        variableToVertice[var_it] = arrangement.vertex_handle(*vert_it);
        var_it++;
    }
    assert(var_it == nbVariables);

    //Setting the cells variables range
    for(auto cell_it = arrangement.cells_begin(); cell_it != arrangement.cells_end(); cell_it++)
    {
        // Constraint 1 : cellVariable <= 1.
        // Constraint 2 : -cellVariable <= 0.
        int cellVariable = cellToVariable[arrangement.cell_handle(*cell_it)];
        vector<pair<size_t, double>> curConstraint1 = {pair<size_t, double>(cellVariable, 1.)};
        constraintsCoorAndWeights.push_back(curConstraint1);
        constraintsUpperBounds.push_back(1.);
        vector<pair<size_t, double>> curConstraint2 = {pair<size_t, double>(cellVariable, -1.)};
        constraintsCoorAndWeights.push_back(curConstraint2);
        constraintsUpperBounds.push_back(0.);
    }
    if(verbose) cout << "Cell to LP variables mapping computed !" << endl <<
                     "Propagating the nu coefficient..." << endl;

    // Compute the nu sign for each cell
    computeNu();
    if(verbose) cout << "Nu coefficient propagated !" << endl;
}

ArrangementToMosek::~ArrangementToMosek()
{
    dispose();
}

/* Associating a number 1 or -1 to each cell. Each pair of cells sharing a facet must have
 * been associated to a different number.*/
void ArrangementToMosek::computeNu()
{
    int currentNu = 1;
    set<PlaneArrangement::Face_handle> currentCells;

    //We start with a bounded cell
    auto firstCell = arrangement.cells_begin();
    while(!arrangement.is_cell_bounded(*firstCell)) firstCell++;
    currentCells.insert(arrangement.cell_handle(*firstCell));

    while(!currentCells.empty())
    {
        // Filling the nu values for the current step
        for(const auto &cellHandle: currentCells)
            cellToNu[cellHandle] = currentNu;

        // Next step, gathering neighbour cells of current cells
        currentNu *= -1;
        set<PlaneArrangement::Face_handle> nextCells;
        for(const auto &cell_handle: currentCells)
        {
            auto &cell = arrangement.cell(cell_handle);
            for(auto facetIt = cell.subfaces_begin(); facetIt != cell.subfaces_end(); facetIt++)
            {
                PlaneArrangement::Face_handle cell0 = arrangement.facet(*facetIt).superface(0);
                PlaneArrangement::Face_handle cell1 = arrangement.facet(*facetIt).superface(1);
                // We find the cell that is not the current cell
                auto candidateCell = (cell0 == cell_handle) ? cell1: cell0;
                if(!arrangement.is_cell_bounded(arrangement.cell(candidateCell))) continue;
                if (cellToNu.find(candidateCell) == cellToNu.end())
                    nextCells.insert(candidateCell);
            }
        }
        currentCells = nextCells;
    }
}

void ArrangementToMosek::computeBoundaryTerm()
{
    for(auto cells_it = arrangement.cells_begin(); cells_it != arrangement.cells_end(); cells_it++)
    {
        // Retrieving the computed weight for the point of views
        double curCellCost = cells_it->cell_value_void_view_points - cells_it->cell_value_full_points;

        // Updated the weights in the objective function
        (*variablesWeights)[cellToVariable[arrangement.cell_handle(*cells_it)]] += curCellCost;

	    // Forcing the unbounded cell to be full
        if(!arrangement.is_cell_bounded(*cells_it))
            (*variablesWeights)[cellToVariable[arrangement.cell_handle(*cells_it)]] -= 1e7;
    }

}

void ArrangementToMosek::computeTexturialTerm()
{
    for(const auto& textPairAndWeight: arrangement.getTexturialCosts())
    {
        // Retrieve the 3 involved variables
        auto &textPair = textPairAndWeight.first;
        double cost = textPairAndWeight.second;
        int textPairVar = texturialPairToVariable[textPair];
        int textCell1 = cellToVariable[textPair.first];
        int textCell2 = cellToVariable[textPair.second];

        // Updating the objective weight
        (*variablesWeights)[textPairVar] = cost*cp;

        // Updating the constraints

        // constraint 1 : textCell1 - textCell2 - textPairVar <= 0
        vector<pair<size_t, double>> curConstraint1 = {pair<size_t, double>(textPairVar, -1.),
                                                      pair<size_t, double>(textCell1, 1.),
                                                      pair<size_t, double>(textCell2, -1.)};
        constraintsCoorAndWeights.push_back(curConstraint1);
        constraintsUpperBounds.push_back(0.);

        // constraint 2 : -textCell1 + textCell2 - textPairVar <= 0
        vector<pair<size_t, double>> curConstraint2 = {pair<size_t, double>(textPairVar, -1.),
                                                       pair<size_t, double>(textCell1, -1.),
                                                       pair<size_t, double>(textCell2, 1.)};
        constraintsCoorAndWeights.push_back(curConstraint2);
        constraintsUpperBounds.push_back(0.);
    }
}

void ArrangementToMosek::computeStructuralTerm()
{
    for(const auto& structTripletAndWeight: arrangement.getStructuralCosts())
    {
        // Retrieve the 4 involved variables
        auto &structTriplet = structTripletAndWeight.first;
        double cost = structTripletAndWeight.second;
        int structTripletVar = structuralTripletToVariable[structTriplet];
        int structCell1 = cellToVariable[get<0>(structTriplet)];
        int structCell2 = cellToVariable[get<1>(structTriplet)];
        int structCell3 = cellToVariable[get<2>(structTriplet)];

        // Update the objective weight
        (*variablesWeights)[structTripletVar] += cost*cp;

        // Updating the constraints

        // constraint 1 : -structCell1 - structCell2 - structCell3 - structTripletVar <= -1
        vector<pair<size_t, double>> curConstraint1 = {pair<size_t, double>(structTripletVar, -1.),
                                                       pair<size_t, double>(structCell1, -1.),
                                                       pair<size_t, double>(structCell2, -1.),
                                                       pair<size_t, double>(structCell3, -1.)};
        constraintsCoorAndWeights.push_back(curConstraint1);
        constraintsUpperBounds.push_back(-1);

        // constraint 2 : -structTripletVar <= 0
        vector<pair<size_t , double>> curConstraint2 = {pair<size_t , double>(structTripletVar, -1)};
        constraintsCoorAndWeights.push_back(curConstraint2);
        constraintsUpperBounds.push_back(0.);
    }
}

void ArrangementToMosek::computeVisibilityAndAreaTerms()
{
    for(auto facets_it = arrangement.facets_begin(); facets_it != arrangement.facets_end(); facets_it++)
    {
        // COST RELATED TO VISIBILITY
        double curFacetVisCost = facets_it->facet_nbr_ray_vis_point*cv;

        // Finding the index of the two cells variables related to this facet
        int var1 = cellToVariable[facets_it->superface(0)];
        int var2 = cellToVariable[facets_it->superface(1)];

        // Finding the index of the additional variable for this facet's visibility cost
        int facetVisVar = facetVisToVariable[arrangement.facet_handle(*facets_it)];

        // Updating the objective weight
        (*variablesWeights)[facetVisVar] += curFacetVisCost;

        // Updating the constraints

        //  constraint 1 : var1 - var2 - facetVisVar <= 0
        vector<pair<size_t, double>> curConstraint1 = {pair<size_t, double>(facetVisVar, -1.),
                                                       pair<size_t, double>(var1, 1.),
                                                       pair<size_t, double>(var2, -1.)};
        constraintsCoorAndWeights.push_back(curConstraint1);
        constraintsUpperBounds.push_back(0.);

        //  constraint 2 : -var1 + var2 - facetVisVar <= 0
        vector<pair<size_t, double>> curConstraint2 = {pair<size_t, double>(facetVisVar, -1.),
                                                       pair<size_t, double>(var1, -1.),
                                                       pair<size_t, double>(var2, 1.)};
        constraintsCoorAndWeights.push_back(curConstraint2);
        constraintsUpperBounds.push_back(0.);

        // COST RELATED TO AREA
        double curFacetAreaCost = arrangement.facet_area(*facets_it)*ca;

        // Finding the index of the additional variable for this facet's area cost
        int facetAreaVar = facetAreaToVariable[arrangement.facet_handle(*facets_it)];

        // Updating the objective weight
        (*variablesWeights)[facetAreaVar] += curFacetAreaCost;

        // Updating the constraints

        //  constraint 1 : var1 - var2 - facetAreaVar <= 0
        vector<pair<size_t, double>> curConstraint3 = {pair<size_t, double>(facetAreaVar, -1.),
                                                       pair<size_t, double>(var1, 1.),
                                                       pair<size_t, double>(var2, -1.)};
        constraintsCoorAndWeights.push_back(curConstraint3);
        constraintsUpperBounds.push_back(0.);

        //  constraint 2 : -var1 + var2 - facetAreaVar <= 0
        vector<pair<size_t, double>> curConstraint4 = {pair<size_t, double>(facetAreaVar, -1.),
                                                       pair<size_t, double>(var1, -1.),
                                                       pair<size_t, double>(var2, 1.)};
        constraintsCoorAndWeights.push_back(curConstraint4);
        constraintsUpperBounds.push_back(0.);

	    // BOUNDING BOX
        // Forcing the bounding box to be either full or empty if required
        if (force_filled_bbox)
            {
            PlaneArrangement::Face_handle fh = arrangement.facet_handle(*facets_it);
                if (arrangement.facet_plane(fh) >= 6) { continue; } //if not a plane of bounding box skip

                // Finding the index of the additional variable for this facet's area cost
                int cellVar0 = cellToVariable[facets_it->superface(0)];
                int cellVar1 = cellToVariable[facets_it->superface(1)];

            // Updating the objective weight
            (*variablesWeights)[cellVar0] += -1e7;
            (*variablesWeights)[cellVar1] += -1e7;
        }
        else if (force_empty_bbox)
        {
            PlaneArrangement::Face_handle fh = arrangement.facet_handle(*facets_it);
                if (arrangement.facet_plane(fh) >= 6) { continue; } //if not a plane of bounding box skip

                // Finding the index of the additional variable for this facet's area cost
                int cellVar0 = cellToVariable[facets_it->superface(0)];
                int cellVar1 = cellToVariable[facets_it->superface(1)];

            // Updating the objective weight
            (*variablesWeights)[cellVar0] += 1e7;
            (*variablesWeights)[cellVar1] += 1e7;
        }
    }
}

void ArrangementToMosek::computeEdgeTerm()
{
    for(auto edge_it = arrangement.edges_begin(); edge_it != arrangement.edges_end(); edge_it++)
    {
        // Gathering neighbour cells of current edge and discarding if any of them is not bounded
        set<PlaneArrangement::Face_handle> neighbourCells;
        bool isSurroundedByBoundedCells = true;
        for(auto facet_it = edge_it->superfaces_begin(); facet_it != edge_it->superfaces_end(); facet_it++)
        {
            PlaneArrangement::Face_handle cell0 = arrangement.facet(*facet_it).superface(0);
            if(!arrangement.is_cell_bounded(cell0))
            {
                isSurroundedByBoundedCells = false;
                break;
            }
            PlaneArrangement::Face_handle cell1 = arrangement.facet(*facet_it).superface(1);
            if(!arrangement.is_cell_bounded(cell1))
            {
                isSurroundedByBoundedCells = false;
                break;
            }
            neighbourCells.insert(cell0);
            neighbourCells.insert(cell1);
        }
        if(!isSurroundedByBoundedCells) continue;
        assert(neighbourCells.size() >= 4);
        assert(neighbourCells.size() % 2 == 0);

        // Computing the weight
        Point vert0 = arrangement.vertex(edge_it->subface(0)).point();
        Point vert1 = arrangement.vertex(edge_it->subface(1)).point();
        double curEdgeCost = sqrt(CGAL::to_double((vert0 - vert1).squared_length()))*ce;

        // Finding the index of the additional variable for this facet's edge cost
        int edgeVar = edgeToVariable[arrangement.edge_handle(*edge_it)];

        // Updating the objective weight
        (*variablesWeights)[edgeVar] += curEdgeCost;

        // Updating the constraints

        //  constraint 1 : sum_neighbourCellsVar - edgeVar <= 0
        //  constraint 2 : -sum_neighbourCellsVar - edgeVar <= 0
        vector<pair<size_t, double>> curConstraint1(0);
        vector<pair<size_t, double>> curConstraint2(0);
        for(auto cell: neighbourCells)
        {
            auto curCellVar = cellToVariable[cell];
            auto curCellNu = cellToNu[cell];
            curConstraint1.emplace_back(curCellVar, 1.*curCellNu);
            curConstraint2.emplace_back(curCellVar, -1.*curCellNu);
        }
        curConstraint1.emplace_back(edgeVar, -1.);
        curConstraint2.emplace_back(edgeVar, -1.);
        constraintsCoorAndWeights.push_back(curConstraint1);
        constraintsCoorAndWeights.push_back(curConstraint2);
        constraintsUpperBounds.push_back(0.);
        constraintsUpperBounds.push_back(0.);
    }
}

void ArrangementToMosek::computeCornerTerm()
{
    for(auto vert_it = arrangement.vertices_begin(); vert_it != arrangement.vertices_end(); vert_it++)
    {
        // Gathering neighbour cells of current vertex and discarding if any of them is not bounded
        set<PlaneArrangement::Face_handle> neighbourCells;
        bool isSurroundedByBoundedCells = true;
        for(auto edge_it = vert_it->superfaces_begin(); edge_it != vert_it->superfaces_end(); edge_it++)
        {
            const auto &curEdge = arrangement.edge(*edge_it);
            for(auto facet_it = curEdge.superfaces_begin(); facet_it != curEdge.superfaces_end(); facet_it++)
            {
                PlaneArrangement::Face_handle cell0 = arrangement.facet(*facet_it).superface(0);
                if(!arrangement.is_cell_bounded(cell0))
                {
                    isSurroundedByBoundedCells = false;
                    break;
                }
                PlaneArrangement::Face_handle cell1 = arrangement.facet(*facet_it).superface(1);
                if(!arrangement.is_cell_bounded(cell1))
                {
                    isSurroundedByBoundedCells = false;
                    break;
                }
                neighbourCells.insert(cell0);
                neighbourCells.insert(cell1);

            }
            if(!isSurroundedByBoundedCells) break;
        }
        if(!isSurroundedByBoundedCells) continue;
        assert(neighbourCells.size() >= 8);
        assert(neighbourCells.size() % 2 == 0);

        // Computing the cost for the current vertex
        double curVertexCost = cc;

        // Finding the index of the additional variable for this vertex's edge cost
        int vertexVar = verticeToVariable[arrangement.vertex_handle(*vert_it)];

        // Updating the objective weight
        (*variablesWeights)[vertexVar] += curVertexCost;

        // Updating the constraints

        //  constraint 1 : sum_neighbourCellsVar - vertexVar <= 0
        //  constraint 2 : -sum_neighbourCellsVar - vertexVar <= 0
        vector<pair<size_t, double>> curConstraint1(0);
        vector<pair<size_t, double>> curConstraint2(0);
        for(auto cell: neighbourCells)
        {
            auto curCellVar = cellToVariable[cell];
            auto curCellNu = cellToNu[cell];
            curConstraint1.emplace_back(curCellVar, 1.*curCellNu);
            curConstraint2.emplace_back(curCellVar, -1.*curCellNu);
        }
        curConstraint1.emplace_back(vertexVar, -1.);
        curConstraint2.emplace_back(vertexVar, -1.);
        constraintsCoorAndWeights.push_back(curConstraint1);
        constraintsCoorAndWeights.push_back(curConstraint2);
        constraintsUpperBounds.push_back(0.);
        constraintsUpperBounds.push_back(0.);
    }
}

pair<vector<size_t>, map<int, int>> ArrangementToMosek::setAndSolve()
{
    //Computing the costs
    if(verbose) cout << "Computing boundary term..." << endl;
    computeBoundaryTerm();
    if(verbose) cout << "Boundary term computed !" << endl;
    if(verbose) cout << "Computing structural term..." << endl;
    computeStructuralTerm();
    if(verbose) cout << "Structural term computed !" << endl << "Computing visibility and area terms..." << endl;
    computeVisibilityAndAreaTerms();
    if(verbose) cout << "Visibility and area terms computed !" << endl << "Computing edge term..." << endl;
    computeEdgeTerm();
    if(verbose) cout << "Edge term computed !" << endl << "Computing corner term..." << endl;
    computeCornerTerm();
    if(verbose) cout << "Corner term computed !" << endl << "Forming linear program..." << endl;

    // Create the variable
    variables = new Variable::t(variable("x", nbVariables, Domain::integral(Domain::greaterThan(0.0))));

    // Create the constraints
    assert(constraintsCoorAndWeights.size() == constraintsUpperBounds.size());
    for (size_t i = 0; i < constraintsCoorAndWeights.size(); i++)
    {
        auto varWeights = new ndarray<double, 1>(constraintsCoorAndWeights[i].size(), 0.);
        auto varCoords = new ndarray<int, 1>(constraintsCoorAndWeights[i].size(), 0.);
        for(size_t j=0; j < constraintsCoorAndWeights[i].size(); j++)
        {
            (*varCoords)[j] = (int)constraintsCoorAndWeights[i][j].first;
            (*varWeights)[j] = constraintsCoorAndWeights[i][j].second;
        }
        constraint("", Expr::dot(shared_ptr<ndarray<double, 1>>(varWeights),
                                 (*variables)->pick(shared_ptr<ndarray<int, 1>>(varCoords))),
                   Domain::lessThan(constraintsUpperBounds.at(i)));

    }

    // DEBUG copy the variable weights
    vector<double> weightsCopy(nbVariables);
    for(size_t i = 0; i < nbVariables; i++)
        weightsCopy[i] = (*variablesWeights)[i];

    // Create the objective
    objective("obj", ObjectiveSense::Minimize,
              Expr::dot(shared_ptr<ndarray<double, 1>>(variablesWeights), *variables));
    if(verbose) cout << "Linear program formed !" << endl << "Starting optimization" << endl;

    // Set the solver's parameters
    setSolverParam("optimizer", "intpnt");
    setSolverParam("intpntBasis", "never");
    setSolverParam("logInfeasAna", 100000);
    setSolverParam("anaSolInfeasTol", 0.0001);
    setSolverParam("logFile", 100000);

    // Solve the linear program
    solve();

    // Retrieve the results
    auto sol = (*variables)->level();
    
    vector<size_t> roundedResult((size_t)arrangement.number_of_cells());
    map<int, int> cellToVar;
    for(const auto& pairCellVar: cellToVariable)
    {
        roundedResult[pairCellVar.second] = (size_t)round(fabs((*sol)[pairCellVar.second]));
        cellToVar[pairCellVar.first] = pairCellVar.second;
    }

    return pair<vector<size_t>, map<int, int>>(roundedResult, cellToVar);
}

int ArrangementToMosek::getNu(PlaneArrangement::Face_handle cell_handle)
{
    assert(cellToNu.find(cell_handle) != cellToNu.end());
    return cellToNu[cell_handle];
}

void ArrangementToMosek::checkCellFromInlierPoint(Point query, shared_ptr<ndarray<double,1>> sol, vector<double> weightsCopy)
{
    // DEBUG check one variable's weight and constraint
    //Point query(-0.80943525, 3.15, 0.45861575);
    auto hf = find_containing_cell((Polyhedral_complex_3::Arrangement_3<Kernel>) arrangement, query);
    int varNumber = cellToVariable[hf];
    cout << "Var weight : " << weightsCopy[varNumber] << endl;
    for (size_t i=0; i < constraintsCoorAndWeights.size(); i++)
    {
	vector<int> allPresentVar(0);
	for(const auto& coorAndWeight: constraintsCoorAndWeights[i])
	    allPresentVar.push_back(coorAndWeight.first);
	if(find(allPresentVar.begin(), allPresentVar.end(), varNumber) != allPresentVar.end())
	{
	    double accu(0);
	    for(size_t j=0; j < constraintsCoorAndWeights[i].size(); j++)
	    {
                accu += constraintsCoorAndWeights[i][j].second * 
			(*sol)[constraintsCoorAndWeights[i][j].first];
                cout << " + " << constraintsCoorAndWeights[i][j].second 
			<< "*var_" << constraintsCoorAndWeights[i][j].first << "(" 
			<< (*sol)[constraintsCoorAndWeights[i][j].first] << ")" ;
	    }
	    cout << " = " << accu << " <= " << constraintsUpperBounds.at(i) << endl;
	}
    }
    
}

void ArrangementToMosek::printEachTermMagnitude(vector<double> weightsCopy)
{
    // DEBUG computing and printing the contribution of each term

    double contribTextuStructu(0);
    double contribVis(0);
    double contribStructuNeutral(0);
    double contribArea(0);
    double contribEdge(0);
    double contribCorner(0);

    for(const auto& pairCellVar: cellToVariable)
        contribTextuStructu += weightsCopy[pairCellVar.second];

    for(const auto& pairFacetVar: facetVisToVariable)
        contribVis += weightsCopy[pairFacetVar.second];

    for(const auto& pairFacetVar: facetAreaToVariable)
        contribArea += weightsCopy[pairFacetVar.second];

    for(const auto& pairEdgeVar: edgeToVariable)
        contribEdge += weightsCopy[pairEdgeVar.second];

    for(const auto& pairVerticeVar: verticeToVariable)
        contribCorner += weightsCopy[pairVerticeVar.second];

    for(const auto& pairNeutralCellsVar: neutralCellsToVariable)
        contribStructuNeutral += weightsCopy[pairNeutralCellsVar.second];

    cout << "Primitive term (without extra structural term) : " << contribTextuStructu << endl;
    cout << "Visibility term : " << contribVis << endl;
    cout << "Area term : " << contribArea << endl;
    cout << "Edge term : " << contribEdge << endl;
    cout << "Corner term : " << contribCorner << endl;
    cout << "Structural neutral cells term : " << contribStructuNeutral << endl;

}

