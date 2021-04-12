#include "RANSAC.h"

using namespace std;
using Json = nlohmann::json;
using namespace Eigen;

RANSAC::RANSAC(std::vector<LineRansac> _lineSet, double _parallelThreshold, double _epsilon, int _maxDrawIterations,
               int _nbRANSACIterations, int _maxRefitIterations, int _maxNbOfPlanes, bool _verbose) :
        parallelThreshold(_parallelThreshold), epsilon(_epsilon),
        maxDrawIterations(_maxDrawIterations),
        nbRANSACIterations(_nbRANSACIterations),
        maxRefitIterations(_maxRefitIterations),
        maxNbOfPlanes(_maxNbOfPlanes),
        verbose(_verbose),
        lineSet(move(_lineSet)),
        detectedPlanes(vector<Plane>(0)),
        isNotClassifiedTwice(vector<int>(0)),
        drawnWithoutPlaneI(vector<vector<int>>(0)),
        nbFoundPlanes(0) {
    // At the beginning, every index is classifiable
    for (size_t i = 0; i < lineSet.size(); i++)
        isNotClassifiedTwice.push_back(i);
}

pair<bool, Plane> RANSAC::drawModel() const {
    for (int i = 0; i < maxDrawIterations; i++) {
        if (isNotClassifiedTwice.empty())
            continue;
        // Draw first line in isNotClassifiedTwice
        const LineRansac &firstLine = lineSet[isNotClassifiedTwice[int(
                double(rand()) / double(RAND_MAX) * double(isNotClassifiedTwice.size() - 1))]];

        // Draw second line in the set of lines which has not been classified in a same plane than firstLine
        const vector<int> &secondLineDrawingSet = (firstLine.getNbTimesClassified() == 1)
                                                  ? drawnWithoutPlaneI[firstLine.getFirstCluster()]
                                                  : isNotClassifiedTwice;
        if (secondLineDrawingSet.empty())
            continue;
        const LineRansac &secondLine = lineSet[secondLineDrawingSet[int(
                double(rand()) / double(RAND_MAX) * double(secondLineDrawingSet.size() - 1))]];

        // Check that lines are not parallel
        Vec3d crossProduct = firstLine.getDirection().cross(secondLine.getDirection());
        if (crossProduct.norm() < parallelThreshold)
            continue;

        //Check that the lines are in the model they define
        Vec3d planeNormal = crossProduct.normalized();
        Vec3d planeInlier = getMidPointSegment(firstLine, secondLine);
        Plane testPlane = Plane(planeNormal, planeInlier);
        if (computeDistanceLineToPlane(firstLine, testPlane) < epsilon &&
            computeDistanceLineToPlane(secondLine, testPlane) < epsilon)
            return pair<bool, Plane>(true, testPlane);
    }
    return pair<bool, Plane>(false, Plane());
}

void RANSAC::evaluateModel(const Plane &planeModel, vector<int> &inliers) const {
    assert(inliers.empty());
    for (int j : isNotClassifiedTwice) {
        const LineRansac &currentLine = lineSet[j];
        if (currentLine.getNbTimesClassified() == 0) {
            //If the line has not been classified yet we just test that it belongs to the plane
            if (computeDistanceLineToPlane(currentLine, planeModel) < epsilon)
                inliers.push_back(j);
        } else {
            //Otherwise we test that it belongs to the intersection of currentPlane and the
            //plane it already belongs to
            assert(currentLine.getNbTimesClassified() == 1);
            if (distanceToIntersection(currentLine, planeModel,
                                       detectedPlanes[currentLine.getFirstCluster()]) < epsilon)
                inliers.push_back(j);
        }
    }
}

bool RANSAC::haveSameConfiguration(const vector<int> &oldInliers, const Plane &oldModel,
                                   const vector<int> &newInliers, const Plane &newModel) const {
    //Check size
    if (oldInliers.size() != newInliers.size())
        return false;
    //Get references to the planes
    const Vec3d &newPlaneInlier = newModel.getPlaneInlier();
    const Vec3d &newPlaneNormal = newModel.getPlaneNormal();
    const Vec3d &oldPlaneInlier = oldModel.getPlaneInlier();
    const Vec3d &oldPlaneNormal = oldModel.getPlaneNormal();
    for (size_t i = 0; i < oldInliers.size(); i++) {
        //Check that the elements are the same
        if (newInliers[i] != oldInliers[i])
            return false;

        //Check that the elements are on the same side of their plane
        bool areOnSameSideFirst = (lineSet[newInliers[i]].getFirstPoint() - newPlaneInlier).dot(newPlaneNormal) *
                                  (lineSet[oldInliers[i]].getFirstPoint() - oldPlaneInlier).dot(oldPlaneNormal) >= 0.;
        bool areOnSameSideSecond = (lineSet[newInliers[i]].getSecondPoint() - newPlaneInlier).dot(newPlaneNormal) *
                                   (lineSet[oldInliers[i]].getSecondPoint() - oldPlaneInlier).dot(oldPlaneNormal) >= 0.;
        if (!areOnSameSideFirst || !areOnSameSideSecond)
            return false;

    }
    return true;
}

pair<Vec3d, Vec3d> RANSAC::makePlaneFromInliers(const vector<int> &inliers) const {

    // Compute weights and point cloud
    double weightAccumulator = 0;
    MatrixXd pointCloud = MatrixXd::Zero(2 * inliers.size(), 3);
    MatrixXd weights = MatrixXd::Zero(2 * inliers.size(), 2 * inliers.size());
    for (size_t i = 0; i < inliers.size(); i++) {
        const LineRansac &curLine = lineSet[inliers[i]];
        double currentWeight = curLine.getNorm();
        pointCloud.row(2 * i) = curLine.getFirstPoint().transpose();
        pointCloud.row(2 * i + 1) = curLine.getSecondPoint().transpose();
        weights(2 * i, 2 * i) = currentWeight;
        weights(2 * i + 1, 2 * i + 1) = currentWeight;
        weightAccumulator += 2 * currentWeight;
    }
    weights /= weightAccumulator;

    // Compute the point cloud center
    MatrixXd cloudCenter = (weights * pointCloud).colwise().sum();

    // Compute the centered point cloud
    MatrixXd centeredPointCloud = pointCloud - cloudCenter.replicate(2 * inliers.size(), 1);

    // Compute the plane normal
    Vec3d normal = (weights * centeredPointCloud).jacobiSvd(ComputeThinU | ComputeThinV).matrixV().col(2);

    return pair<Vec3d, Vec3d>(normal, Vec3d(Map<Vec3d>(cloudCenter.data(), cloudCenter.cols() * cloudCenter.rows())));

}


void RANSAC::refitting(Plane &plane, vector<int> &inliers) const {
    assert(!inliers.empty());
    vector<int> oldInliers;
    Plane oldModel;
    int nbIter = 0;
    const unsigned long oldNbInliers = inliers.size();
    while (nbIter < maxRefitIterations && !haveSameConfiguration(oldInliers, oldModel, inliers, plane)) {
        // Saving the current model
        oldModel = plane;
        oldInliers = inliers;

        // Compute weights and point cloud
        double weightAccumulator = 0;
        MatrixXd pointCloud = MatrixXd::Zero(2 * inliers.size(), 3);
        MatrixXd weights = MatrixXd::Zero(2 * inliers.size(), 2 * inliers.size());
        for (size_t i = 0; i < inliers.size(); i++) {
            const LineRansac &curLine = lineSet[inliers[i]];
            double currentWeight = curLine.getNorm();
            if (lineCrossesPlane(curLine, plane)) currentWeight /= 2;
            pointCloud.row(2 * i) = curLine.getFirstPoint().transpose();
            pointCloud.row(2 * i + 1) = curLine.getSecondPoint().transpose();
            weights(2 * i, 2 * i) = currentWeight;
            weights(2 * i + 1, 2 * i + 1) = currentWeight;
            weightAccumulator += 2 * currentWeight;
        }
        weights /= weightAccumulator;

        // Compute the point cloud center
        MatrixXd cloudCenter = (weights * pointCloud).colwise().sum();

        // Compute the centered point cloud
        MatrixXd centeredPointCloud = pointCloud - cloudCenter.replicate(2 * inliers.size(), 1);

        // Compute the plane normal
        Vec3d normal = (weights * centeredPointCloud).jacobiSvd(ComputeThinU | ComputeThinV).matrixV().col(2);

        // Evaluate new model
        Plane newModel(Plane(normal, Vec3d(Map<Vec3d>(cloudCenter.data(), cloudCenter.cols() * cloudCenter.rows()))));
        vector<int> newInliers(0);
        evaluateModel(newModel, newInliers);

        // If not better, we stop refitting here
        if (newInliers.size() < inliers.size()) break;

        // We save the new model
        plane = newModel;
        inliers = newInliers;
        nbIter++;
    }
    if (!verbose) return;
    cout << "Nb of refitting operations : " << nbIter << endl;
    cout << " nb of inliers before refitting : " << oldNbInliers << endl;
    cout << " nb of inliers after refitting : " << inliers.size() << endl;
}

pair<bool, Plane> RANSAC::extractPlane() {
    unsigned int bestNbOfInliers = 0;
    vector<int> bestInliers(0);
    Vec3d bestNormal, bestInlier = Vec3d(0., 0., 0.);
#pragma omp parallel for
    for (int i = 0; i < nbRANSACIterations; i++) {
        // Draw a random plane
        pair<bool, Plane> drawnModelPair = drawModel();
        if (!drawnModelPair.first)
            continue;

        // Find the model's inliers
        vector<int> currentInliers(0);
        evaluateModel(drawnModelPair.second, currentInliers);

        //Test if we have a better model
#pragma omp flush(bestNbOfInliers)
        if (currentInliers.size() > bestNbOfInliers) {
#pragma omp critical
            {
                bestNbOfInliers = static_cast<int>(currentInliers.size());
                bestInliers = currentInliers;
                bestNormal = drawnModelPair.second.getPlaneNormal();
                bestInlier = drawnModelPair.second.getPlaneInlier();
            }
        }
    }

    //If we have less than 3 inliers, the model is considered to be wrong
    if (bestNbOfInliers < 3)
        return pair<bool, Plane>(false, Plane());

    //Refitting the plane
    Plane finalPlane = Plane(bestNormal, bestInlier);
    refitting(finalPlane, bestInliers);

    //Here the model is accepted, we update all the state variables
    finalPlane.setInliers(bestInliers);
    updateLists(bestInliers);
    detectedPlanes.push_back(finalPlane);
    return pair<bool, Plane>(true, finalPlane);
}

void RANSAC::extractAllPlanes() {
    while (extractPlane().first && nbFoundPlanes < maxNbOfPlanes) {
        cout << "Extracted " << nbFoundPlanes << " planes." << endl;
        cout << "Nb of structural lines detected : " << lineSet.size() - isNotClassifiedTwice.size()
             << " out of " << lineSet.size() << endl;
    }
}

void RANSAC::updateLists(const vector<int> &inlierList) {
    drawnWithoutPlaneI.push_back(isNotClassifiedTwice);
    assert(drawnWithoutPlaneI.size() == nbFoundPlanes + 1);
    for (int it : inlierList) {
        lineSet[it].classify(nbFoundPlanes);
        int nbTimesClassified = lineSet[it].getNbTimesClassified();
        assert(nbTimesClassified < 3);
        if (nbTimesClassified >= 1) {
            //Delete the inlier index from drawnWithoutPlaneI[nbFoundPlanes]
            auto item_it = find(drawnWithoutPlaneI[nbFoundPlanes].begin(), drawnWithoutPlaneI[nbFoundPlanes].end(),
                                it);
            assert(item_it != drawnWithoutPlaneI[nbFoundPlanes].end());
            drawnWithoutPlaneI[nbFoundPlanes].erase(item_it);
        }
        if (nbTimesClassified == 2) {
            //Delete the inlier index from isNotClassifiedTwice
            auto item_it = find(isNotClassifiedTwice.begin(), isNotClassifiedTwice.end(), it);
            assert(item_it != isNotClassifiedTwice.end());
            isNotClassifiedTwice.erase(item_it);

            //Delete the inlier index from drawnWithoutPlaneI[j] for j = 0 ... nbFoundPlanes - 1
            for (unsigned int j = 0; j < nbFoundPlanes; j++) {
                auto item_it_loop = find(drawnWithoutPlaneI[j].begin(), drawnWithoutPlaneI[j].end(), it);
                if (item_it_loop != drawnWithoutPlaneI[j].end())
                    drawnWithoutPlaneI[j].erase(item_it_loop);
            }
        }
    }
    nbFoundPlanes++;
}

PlaneIt RANSAC::mergePlanes(const PlanePair &curPair, list<Plane>& planeList) const
{
    // Merging inliers (without double entries)
    set<int> mergedInliers(get<0>(curPair)->getInliers().begin(), get<0>(curPair)->getInliers().end());
    mergedInliers.insert(get<1>(curPair)->getInliers().begin(), get<1>(curPair)->getInliers().end());
    vector<int> newPlaneInliers(mergedInliers.begin(), mergedInliers.end());
    // Evaluating merged plane attributes
    pair<Vec3d, Vec3d> planeAttributes = makePlaneFromInliers(newPlaneInliers);
    // Remove the 2 old planes from the list
    planeList.erase(get<0>(curPair));
    planeList.erase(get<1>(curPair));
    // Adding the merged plane
    Plane newPlane(planeAttributes.first, planeAttributes.second);
    newPlane.setInliers(newPlaneInliers);
    planeList.push_back(newPlane);
    // Return an iterator to the new plane in the list
    return prev(planeList.rbegin().base());
}

void RANSAC::resetLinePlanesAttributionFromPlaneInlierLists(vector<LineRansac>& lineContainer,
                                                            const vector<Plane>& detectedPlanes)
{
    // Reset lines index
    for (auto &line: lineContainer)
        line.emptyPlaneIndex();

    // Update the planes association
    for (int i = 0; i < detectedPlanes.size(); i++)
        for (auto lineIdx: detectedPlanes[i].getInliers())
            lineContainer[lineIdx].classify(i);
}

void RANSAC::refineExtractedPlanes() {
    assert(!detectedPlanes.empty());

    // Lambda function for inserting a pair of plane in a pair set iif
    // the candidate pair satisfies the 1st condition (small angle) and the 2nd condition (enough shared support)
    auto pairInsert =
            [this](multiset<PlanePair, PlanePairComp> &pairSet, const PlaneIt &pOne, const PlaneIt &pTwo) {
                double score = acos(fabs(pOne->getPlaneNormal().dot(pTwo->getPlaneNormal())));
                if (score >= TEN_DEGREES_IN_RADIANS) return; //1st condition
                // Second condition on total number of mutual inliers between the 2 planes
                int accum = 0;
                for (int lineIdx: pOne->getInliers()) {
                    if (computeDistanceLineToPlane(this->lineSet[lineIdx], *pTwo) < this->epsilon &&
                        computeDistanceLineToPlane(this->lineSet[lineIdx], *pOne) < this->epsilon)
                        accum++;
                }
                if (accum < min(pOne->getInliers().size(), pTwo->getInliers().size()) / 5) return; //2nd condition
                pairSet.insert(make_tuple(pOne, pTwo, score));
            };

    // Build plane list
    list<Plane> planeList;
    for (const auto &detectedPlane : detectedPlanes)
        planeList.push_back(detectedPlane);

    // Build a multiset containing all the pairs of planes satisfying 1st condition and sorted by increasing angle
    multiset<PlanePair, PlanePairComp> allPairs;
    for (auto planeOne = planeList.begin(); planeOne != planeList.end(); planeOne++) {
        auto planeTwo = planeOne;
        planeTwo++;
        while (planeTwo != planeList.end()) {
            pairInsert(allPairs, planeOne, planeTwo);
            planeTwo++;
        }
    }

    // Merging
    int nbMergedPlanes = 0;
    while (!allPairs.empty()) {
        auto curPair = allPairs.begin();
        assert(get<2>(*curPair) < TEN_DEGREES_IN_RADIANS);
        // Merging procedure : update the list of candidate pairs and merge the planes which are in curPair
        nbMergedPlanes++;

        // New allPair will contain all the pairs not related to any of the 2 merging planes
        multiset<PlanePair, PlanePairComp> newAllPairs;
        for(auto pair: allPairs)
        {
            if(pair == *curPair) continue;
            if(get<0>(pair) == get<0>(*curPair)) continue;
            if(get<0>(pair) == get<1>(*curPair)) continue;
            if(get<1>(pair) == get<0>(*curPair)) continue;
            if(get<1>(pair) == get<1>(*curPair)) continue;
            newAllPairs.insert(pair);
        }

        // Actually merging the 2 planes in curPair

        // Merging inliers (without double entries)
        set<int> mergedInliers(get<0>(*curPair)->getInliers().begin(), get<0>(*curPair)->getInliers().end());
        mergedInliers.insert(get<1>(*curPair)->getInliers().begin(), get<1>(*curPair)->getInliers().end());
        vector<int> newPlaneInliers(mergedInliers.begin(), mergedInliers.end());
        // Evaluating merged plane attributes
        std::pair<Vec3d, Vec3d> planeAttributes = makePlaneFromInliers(newPlaneInliers);
        Plane newPlane(planeAttributes.first, planeAttributes.second);
        // Checking that the thickness of the new plane is < epsilonRate*epsilon
        bool validPlane = true;
        for (int lineIdx: newPlaneInliers)
            if (computeDistanceLineToPlane(lineSet[lineIdx], newPlane) >= 3.*epsilon)
            {
                validPlane = false;
                break;
            }
        // If not, we abort the merging
        if(!validPlane)
        {
            allPairs.erase(curPair);
            continue;
        }
        // Now, merging is confirmed
        nbMergedPlanes++;
        // Remove the 2 old planes from the list
        planeList.erase(get<0>(*curPair));
        planeList.erase(get<1>(*curPair));
        // Adding the merged plane
        newPlane.setInliers(newPlaneInliers);
        planeList.push_back(newPlane);
        // Get an iterator to the new plane in the list
        auto newPlaneIt = prev(planeList.rbegin().base());

        // Adding pairs of planes that include the new plane in the new allPair multiset
        for(auto plane = planeList.begin(); plane != planeList.end(); plane++)
        {
            if(plane == newPlaneIt) continue;
            pairInsert(newAllPairs, plane, newPlaneIt);
        }
        allPairs = newAllPairs;
    }

    if(verbose)
    {
        cout << nbMergedPlanes << " merging operations have been done." << endl;
        cout << "Old number of planes : " << detectedPlanes.size() << endl;
        cout << "New number of planes : " << planeList.size() << endl;
    }

    // Update the detected planes
    detectedPlanes = vector<Plane>(0);
    for(const auto &plane: planeList)
        detectedPlanes.push_back(plane);

    // Update the line -> plane relationships
    resetLinePlanesAttributionFromPlaneInlierLists(lineSet, detectedPlanes);

    if(!verbose) return;

    // Write summary of the refined plane //
    multiset<double> failingEpsilon;
    for(const auto& plane: planeList)
    {
        double epsilonMax = 0.;
        for(const auto& lineIdx: plane.getInliers())
        {
            double cur_score = computeDistanceLineToPlane(lineSet[lineIdx], plane);
            if (cur_score > epsilon)
                epsilonMax = cur_score;
        }
        if(epsilonMax != 0.) failingEpsilon.insert(epsilonMax);
    }
    cout << "Number of planes whose thickness is greater than epsilon : " << failingEpsilon.size() << endl;
    cout << "Thickness/epsilon rates : " << endl;
    for(auto epsilonFail: failingEpsilon)
        cout << epsilonFail / epsilon << endl;
    cout << endl;
}

void RANSAC::refineExtractedPlanesAngleOnly(const double epsilonRate)
{
    assert(!detectedPlanes.empty());

    // Lambda function for inserting a pair of plane in a pair set iif
    // the candidate pair satisfies the 1st condition (small angle)
    auto pairInsert =
            [this](multiset<PlanePair, PlanePairComp> &pairSet, const PlaneIt &pOne, const PlaneIt &pTwo) {
                double score = acos(fabs(pOne->getPlaneNormal().dot(pTwo->getPlaneNormal())));
                if (score >= TEN_DEGREES_IN_RADIANS) return; //1st condition
                pairSet.insert(make_tuple(pOne, pTwo, score));
            };

    // Build plane list
    list<Plane> planeList;
    for (const auto &detectedPlane : detectedPlanes)
        planeList.push_back(detectedPlane);

    // Build a multiset containing all the pairs of planes satisfying 1st condition and sorted by increasing angle
    multiset<PlanePair, PlanePairComp> allPairs;
    for (auto planeOne = planeList.begin(); planeOne != planeList.end(); planeOne++) {
        auto planeTwo = planeOne;
        planeTwo++;
        while (planeTwo != planeList.end()) {
            pairInsert(allPairs, planeOne, planeTwo);
            planeTwo++;
        }
    }

    // Merging
    int nbMergedPlanes = 0;
    while (!allPairs.empty()) {
        auto curPair = allPairs.begin();
        assert(get<2>(*curPair) < TEN_DEGREES_IN_RADIANS);

        // New allPair will contain all the pairs not related to any of the 2 merging planes
        multiset<PlanePair, PlanePairComp> newAllPairs;
        for(auto pair: allPairs)
        {
            if(pair == *curPair) continue;
            if(get<0>(pair) == get<0>(*curPair)) continue;
            if(get<0>(pair) == get<1>(*curPair)) continue;
            if(get<1>(pair) == get<0>(*curPair)) continue;
            if(get<1>(pair) == get<1>(*curPair)) continue;
            newAllPairs.insert(pair);
        }

        // Actually merging the 2 planes in curPair

        // Merging inliers (without double entries)
        set<int> mergedInliers(get<0>(*curPair)->getInliers().begin(), get<0>(*curPair)->getInliers().end());
        mergedInliers.insert(get<1>(*curPair)->getInliers().begin(), get<1>(*curPair)->getInliers().end());
        vector<int> newPlaneInliers(mergedInliers.begin(), mergedInliers.end());
        // Evaluating merged plane attributes
        std::pair<Vec3d, Vec3d> planeAttributes = makePlaneFromInliers(newPlaneInliers);
        Plane newPlane(planeAttributes.first, planeAttributes.second);
        // Checking that the thickness of the new plane is < epsilonRate*epsilon
        bool validPlane = true;
        for (int lineIdx: newPlaneInliers)
            if (computeDistanceLineToPlane(lineSet[lineIdx], newPlane) >= epsilonRate*epsilon)
            {
                validPlane = false;
                break;
            }
        // If not, we abort the merging
        if(!validPlane)
        {
            allPairs.erase(curPair);
            continue;
        }
        // Now, merging is confirmed
        nbMergedPlanes++;
        // Remove the 2 old planes from the list
        planeList.erase(get<0>(*curPair));
        planeList.erase(get<1>(*curPair));
        // Adding the merged plane
        newPlane.setInliers(newPlaneInliers);
        planeList.push_back(newPlane);
        // Get an iterator to the new plane in the list
        auto newPlaneIt = prev(planeList.rbegin().base());
        // Adding pairs of planes that include the new plane in the new allPair multiset
        for(auto plane = planeList.begin(); plane != planeList.end(); plane++)
        {
            if(plane == newPlaneIt) continue;
            pairInsert(newAllPairs, plane, newPlaneIt);
        }
        allPairs = newAllPairs;
    }

    if(verbose)
    {
        cout << nbMergedPlanes << " merging operations have been done." << endl;
        cout << "Old number of planes : " << detectedPlanes.size() << endl;
        cout << "New number of planes : " << planeList.size() << endl;
    }

    // Update the detected planes
    detectedPlanes = vector<Plane>(0);
    for(const auto &plane: planeList)
        detectedPlanes.push_back(plane);

    // Update the line -> plane relationships
    resetLinePlanesAttributionFromPlaneInlierLists(lineSet, detectedPlanes);

    if(!verbose) return;

    // Write summary of the refined planes //
    multiset<double> failingEpsilon;
    for(const auto& plane: planeList)
    {
        double epsilonMax = 0.;
        for(const auto& lineIdx: plane.getInliers())
        {
            double cur_score = computeDistanceLineToPlane(lineSet[lineIdx], plane);
            if (cur_score > epsilon)
                epsilonMax = cur_score;
        }
        if(epsilonMax != 0.) failingEpsilon.insert(epsilonMax);
    }
    cout << "Number of planes whose thickness is greater than epsilon : " << failingEpsilon.size() << endl;
    cout << "Thickness/epsilon rates : " << endl;
    for(auto epsilonFail: failingEpsilon)
        cout << epsilonFail / epsilon << endl;
    cout << endl;

}

void RANSAC::writeOutputJson(std::string path) const {
    string outputPath = path + "_input_for_reconstruction.json";
    Json outData;

    //Lines
    Json outDataLines;
    for (auto &curLine : lineSet) {
        // Computing line extremities
        Json curLineJson;
        curLineJson["pt1"] = eigen2vec(curLine.getFirstPoint());
        curLineJson["pt2"] = eigen2vec(curLine.getSecondPoint());

        //Computing line point of views
        Json curPointOfViews;
        const vector<Vec3d> &pointOfViews = curLine.getPointOfViews();
        for (auto &pointOfView : pointOfViews)
            curPointOfViews.push_back(eigen2vec(pointOfView));
        curLineJson["pt_views"] = curPointOfViews;

        //Computing line residuals
        Json curResiduals;
        const vector<vector<double>> &residuals = curLine.getResiduals();
        for (auto &residual: residuals)
            curResiduals.push_back(residual);
        curLineJson["residual_per_pt_view"] = curResiduals;

        //Computing line planes
        Json curLinePlanes = curLine.getPlanesIndex();
        curLineJson["plane_index"] = curLinePlanes;

        //Appending the line
        outDataLines.push_back(curLineJson);

    }
    outData["lines"] = outDataLines;

    //Planes
    Json outDataPlanes;
    for (auto &curPlane: detectedPlanes) {
        Json curPlaneJson;
        curPlaneJson["normal"] = eigen2vec(curPlane.getPlaneNormal());
        curPlaneJson["inlier"] = eigen2vec(curPlane.getPlaneInlier());
        outDataPlanes.push_back(curPlaneJson);
    }
    outData["planes"] = outDataPlanes;


    ofstream outStream(outputPath.c_str());
    if (!outStream)
        cerr << "Could not open output file location : " << path << endl;

    outStream << outData.dump();
    outStream.close();
}

const std::vector<int> RANSAC::getIsNotClassifiedTwice() const {
    return isNotClassifiedTwice;
}

const std::vector<std::vector<int>> RANSAC::getDrawnWithoutPlaneI() const {
    return drawnWithoutPlaneI;
}

const std::vector<Plane>& RANSAC::getDetectedPlanes() const {
    return detectedPlanes;
}

int RANSAC::getNbOfStructuralLines() const {
    return int(lineSet.size() - isNotClassifiedTwice.size());
}
