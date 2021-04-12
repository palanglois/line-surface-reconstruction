#ifndef RANSAC_ON_LINES_CPP_RANSAC_H
#define RANSAC_ON_LINES_CPP_RANSAC_H

#include <set>
#include "geometric_functions.h"

#define TEN_DEGREES_IN_RADIANS 0.17453

class RANSAC
{
public:
    RANSAC(std::vector<LineRansac> _lineSet, double _parallelThreshold, double _epsilon,
           int _maxDrawIterations=int(1e5), int _nbRANSACIterations=100,
           int _maxRefitIterations=20, int _maxNbOfPlanes=1000, bool _verbose=false);

    /* Draw two lines and estimate a plane model from it */
    std::pair<bool, Plane> drawModel() const;

    /* Find the inliers of a plane model */
    void evaluateModel(const Plane& planeModel, std::vector<int> &inliers) const;

    /* Check that two sets of lines have same configuration (in front of/behind)
     * with respect to their respective planes */
    bool haveSameConfiguration(const std::vector<int> &oldInliers, const Plane& oldModel,
                               const std::vector<int> &newInliers, const Plane& newModel) const;

    /* Make a plane model from a set of inliers */
    std::pair<Vec3d, Vec3d> makePlaneFromInliers(const std::vector<int>& inliers) const;

    /* Refitting of a found model */
    void refitting(Plane& plane, std::vector<int> &inliers) const;

    /* Extract a plane in the model */
    std::pair<bool, Plane> extractPlane();

    /* Extract as much plane as possible in the model */
    void extractAllPlanes();

    /* Update the class variables at the end of a plane extraction */
    void updateLists(const std::vector<int>& inlierList);

    /* Merging 2 extracted planes and returns the resulting plane */
    PlaneIt mergePlanes(const PlanePair &curPair, std::list<Plane>& planeList) const;

    /* There is a bidirectional relationship between lines and planes.
     * During the plane refinement procedure, the line -> plane relationship is broken, so we reset it
     * thanks to the plane -> line relationship which is valid.*/
    static void resetLinePlanesAttributionFromPlaneInlierLists(std::vector<LineRansac>& lineContainer,
                                                               const std::vector<Plane>& detectedPlanes);

    /* Refining the planes by merging */
    void refineExtractedPlanes();

    /* Refining the planes by merging (version with criteria on angle only) */
    void refineExtractedPlanesAngleOnly(double epsilonRate = 3.);

    /* Write the results as json file for the reconstruction part */
    void writeOutputJson(std::string path) const;

    /* Getters */
    const std::vector<int> getIsNotClassifiedTwice() const;
    const std::vector<std::vector<int>> getDrawnWithoutPlaneI() const;
    const std::vector<Plane>& getDetectedPlanes() const;
    int getNbOfStructuralLines() const;
private:
    const double parallelThreshold;
    const double epsilon;
    const int maxDrawIterations;
    const int nbRANSACIterations;
    const int maxRefitIterations;
    const int maxNbOfPlanes;
    const bool verbose;
    std::vector<LineRansac> lineSet;

    std::vector<Plane> detectedPlanes;
    std::vector<int> isNotClassifiedTwice;
    std::vector<std::vector<int>> drawnWithoutPlaneI;
    unsigned int nbFoundPlanes;
};

#endif //RANSAC_ON_LINES_CPP_RANSAC_H
