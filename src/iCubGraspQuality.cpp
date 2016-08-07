/**************************************************************************************************
 *  File:    iCubGraspQuality.cpp                                                                 *
 *  Author:  Jose Miguel Nogueira, josemscnogueira@gmail.com                                      *
 *                                                                                                *
 *  History: Adapted from Simox code by Niko Vahrenkamp                                           *
 **************************************************************************************************/


/**************************************************************************************************
 *  Include Files                                                                                 *
 **************************************************************************************************/
#include <iCubGraspQuality.hpp>

namespace GraspStudio
{

/**
 * Constructor for iCubGraspQuality
 */
iCubGraspQuality::iCubGraspQuality(float object_length,
                                   float unit_force,
                                   float friction_coeff,
                                   int   friction_samples)
{
    _coneGenerator.reset(new ContactConeGenerator(friction_samples, friction_coeff, unit_force));

    _object_length    = object_length;
    _unit_force       = unit_force;
    _friction_coeff   = friction_coeff;
    _friction_samples = friction_samples;
    _verbose          = false;

    GWSCalculated = false;
}


/**
 * Sets the points of contact for grasp metric evaluation
 *
 * @param contactPoints Points of contact
 */
void iCubGraspQuality::setContactPoints(const std::vector<VirtualRobot::MathTools::ContactPoint>& contactPoints)
{
    this -> _contactPoints .clear();
    this -> _contactPointsM.clear();

    std::vector<VirtualRobot::MathTools::ContactPoint>::const_iterator objPointsIter;

    for (objPointsIter = contactPoints.begin(); objPointsIter != contactPoints.end(); objPointsIter++)
    {
        VirtualRobot::MathTools::ContactPoint point       = (*objPointsIter);
                                            //   point.p    -= centerOfModel; TODO
                                              point.n.normalize();
                                              point.force = 1.0f;

        this -> _contactPoints.push_back(point);
    }

    VirtualRobot::MathTools::convertMM2M(this -> _contactPoints, this -> _contactPointsM);

    if (_verbose)
    {
        VR_INFO << ": Nr of contact points:" << this -> _contactPoints.size() << std::endl;
    }

    GWSCalculated = false;
}


/**
 * Calculates and returns the grasp quality matric
 *
 * @return  Grasp quality metric
 */
float iCubGraspQuality::getGraspQuality(void)
{
    calculateGraspQuality();

    return _grasp_quality;
}


/**
 * Evaluates if grasp is force closure
 *
 * @return  True if grasp is force-closure
 */
bool iCubGraspQuality::isGraspForceClosure(void)
{
    if (!GWSCalculated)
    {
        calculateGWS();
    }

    return isOriginInGWSHull();
}


/**
 * Calculates GWS
 *     [Protected attribute]
 */
void iCubGraspQuality::calculateGWS(void)
{
    bool printAll = false;

    if (_contactPointsM.empty())
    {
        printf("Contact points not set.\n");
        return;
    }

    //Rotate generic friction cone to align with object normals
    std::vector<VirtualRobot::MathTools::ContactPoint>::iterator objPointsIter;
    std::vector<VirtualRobot::MathTools::ContactPoint>           conePoints;

    if (_verbose && printAll)
    {
        std::cout << "GWS contact points:" << std::endl;
    }

    for (objPointsIter = _contactPointsM.begin(); objPointsIter != _contactPointsM.end(); objPointsIter++)
    {
        if (_verbose && printAll)
        {
            VirtualRobot::MathTools::print((*objPointsIter));
        }

        _coneGenerator -> computeConePoints((*objPointsIter), conePoints);
    }

    //Generate convex hull from rotated contact friction cones
    convexHullGWS       = calculateConvexHull(conePoints);
    convexHullCenterGWS = convexHullGWS->center;

    if (_verbose && printAll)
    {
        GRASPSTUDIO_INFO << " CENTER of GWS: " << std::endl;
        VirtualRobot::MathTools::print(convexHullCenterGWS);
    }

    GWSCalculated = true;
}


/**
 * Calculates convex hull from points of contact
 *     [Protected]
 *
 * @param  points  Points of contact
 *
 * @return         Convexhull object
 */
VirtualRobot::MathTools::ConvexHull6DPtr iCubGraspQuality::calculateConvexHull(std::vector<VirtualRobot::MathTools::ContactPoint>& points)
{
    bool printAll = true;

    if (_verbose && printAll)
    {
        GRASPSTUDIO_INFO << "Convex hull points for wrench calculation:" << std::endl;
        printContacts(points);
    }

    // create wrench
    std::vector<VirtualRobot::MathTools::ContactPoint> wrenchPoints = createWrenchPoints(points, Eigen::Vector3f::Zero(), _object_length); // contact points are already moved so that com is at origin

    if (_verbose && printAll)
    {
        GRASPSTUDIO_INFO << "Wrench points:" << endl;
        printContacts(wrenchPoints);
    }

    return ConvexHullGenerator::CreateConvexHull(wrenchPoints);
}


/**
 * Creates Wrench points for convexhull creation from points of contact
 *
 * @param points         Points of contact
 * @param centerOfModel  Center of the object
 * @param objectLengthMM Object length in mm
 *
 * @return               Wrench space points of contact
 */
std::vector<VirtualRobot::MathTools::ContactPoint> iCubGraspQuality::createWrenchPoints(std::vector<VirtualRobot::MathTools::ContactPoint>& points,
                                                                                        const Eigen::Vector3f&                              centerOfModel,
                                                                                        float                                               objectLengthMM)
{
    std::vector<VirtualRobot::MathTools::ContactPoint>           result;
    std::vector<VirtualRobot::MathTools::ContactPoint>::iterator iter = points.begin();
    VirtualRobot::MathTools::ContactPoint                        p;
    Eigen::Vector3f                                              normal;
    VirtualRobot::MathTools::ContactPoint                        tmpP;

    bool  convertMM2M = true;
    float factor      = 1.0f;

    if (objectLengthMM != 0)
    {
        if (convertMM2M) factor = 2.0f / (objectLengthMM / 1000.0f);    // == max distance from center ( 1 / (length/2) )
        else             factor = 2.0f / objectLengthMM;                // == max distance from center ( 1 / (length/2) )
    }

    // Calculate Wrench Points
    while (iter != points.end())
    {
        p.p    = iter -> n;

        tmpP.p =   iter -> p - centerOfModel;
        tmpP.n = -(iter -> n);

        p.n = factor * tmpP.p.cross(tmpP.n);

        result.push_back(p);
        iter++;
    }

    return result;
}


/**
 * Prints points of contact to stdout
 *
 * @param points Points of contact
 */
void iCubGraspQuality::printContacts(std::vector<VirtualRobot::MathTools::ContactPoint>& points)
{
    std::vector<VirtualRobot::MathTools::ContactPoint>::iterator iter = points.begin();

    while (iter != points.end())
    {
        std::cout << "# ";
        VirtualRobot::MathTools::print((*iter));
        iter++;
    }
}


/**
 * Calculates convexhull center from convexhull
 *
 * @param  hull Convexhull
 *
 * @return      Center of convexhull in wrenchspace
 */
VirtualRobot::MathTools::ContactPoint iCubGraspQuality::calculateHullCenter(VirtualRobot::MathTools::ConvexHull6DPtr hull)
{
    if (!hull)
    {
        GRASPSTUDIO_ERROR << "NULL data?!" << endl;
        return VirtualRobot::MathTools::ContactPoint();
    }

    VirtualRobot::MathTools::ContactPoint                        resultCenter;
    std::vector<VirtualRobot::MathTools::ContactPoint>::iterator iter;

    resultCenter.p.setZero();
    resultCenter.n.setZero();

    if (hull->vertices.size() == 0)
    {
        std::cout << __FUNCTION__ << ": error, no vertices..." << endl;
        return resultCenter;
    }

    for (iter = hull->vertices.begin(); iter != hull->vertices.end(); iter++)
    {
        resultCenter.p += iter -> p;
        resultCenter.n += iter -> n;
    }

    resultCenter.p /= (float)hull -> vertices.size();
    resultCenter.n /= (float)hull -> vertices.size();

    return resultCenter;
}


/**
 * Calculates the minimum distance to GWShull
 *
 * @param  point  Point of in wrench space
 *
 * @return        Distance
 */
float iCubGraspQuality::minDistanceToGWSHull(VirtualRobot::MathTools::ContactPoint& point)
{
    float minDist = FLT_MAX;
    float dist[6];
    float currentDist2;
    std::vector<VirtualRobot::MathTools::TriangleFace6D>::iterator faceIter;

    for (faceIter = convexHullGWS -> faces.begin(); faceIter != convexHullGWS -> faces.end(); faceIter++)
    {
        VirtualRobot::MathTools::ContactPoint faceCenter;

        faceCenter.p.setZero();
        faceCenter.n.setZero();

        for (int j = 0; j < 6; j++)
        {
            faceCenter.p += (convexHullGWS->vertices)[faceIter->id[j]].p;
            faceCenter.n += (convexHullGWS->vertices)[faceIter->id[j]].n;
        }

        faceCenter.p /= 6.0f;
        faceCenter.n /= 6.0f;
        currentDist2  = 0;

        for (int j = 0; j < 3; j++)
        {
            dist[j    ]   = (faceCenter.p(j) - point.p(j));
            dist[j + 3]   = (faceCenter.n(j) - point.n(j));

            currentDist2 += dist[j    ] * dist[j    ];
            currentDist2 += dist[j + 3] * dist[j + 3];
        }

        if (currentDist2 < minDist)
        {
            minDist = currentDist2;
        }
    }

    return sqrtf(minDist);
}


/**
 * Evaluates if origin is inside GWShull
 *
 * @return  True if origin is inside the GWShull
 */
bool iCubGraspQuality::isOriginInGWSHull(void)
{
    if (!GWSCalculated || !convexHullGWS)
    {
        return false;
    }

    std::vector<VirtualRobot::MathTools::TriangleFace6D>::iterator faceIter;

    for (faceIter = convexHullGWS -> faces.begin(); faceIter != convexHullGWS -> faces.end(); faceIter++)
    {
        // ignore rounding errors
        if (faceIter -> distPlaneZero > 1e-4)
        {
            return false;
        }
    }

    return true;
}


/**
 * Calculates the minimum distance from hullcenter to one of its facets
 *
 * @param  ch Convexhull
 *
 * @return    Distance in wrench space
 */
float iCubGraspQuality::minOffset(VirtualRobot::MathTools::ConvexHull6DPtr ch)
{
    if (!ch)
    {
        return 0.0f;
    }

    float fRes         = FLT_MAX;
    int   nWrongFacets = 0;

    for (size_t i = 0; i < (size_t)ch -> faces.size(); i++)
    {
        if      (  ch->faces[i].distNormCenter  > 0   ) nWrongFacets++;
        else if (-(ch->faces[i].distNormCenter) < fRes) fRes = -(ch->faces[i].distNormCenter);
    }

    if (nWrongFacets > 0)
    {
        std::cout << __FUNCTION__ << " Warning: offset of " << nWrongFacets << " facets >0 (# of facets:" << ch->faces.size() << ")" << endl;
    }

    return fRes;
}


/**
 * Calculate volume of grasp
 *
 * @return  Volume
 */
float iCubGraspQuality::getVolumeGraspMeasure(void)
{
    if (!GWSCalculated)
    {
        calculateGWS();
    }

    if (!convexHullGWS || convexHullGWS->vertices.size() == 0)
    {
        std::cout << __FUNCTION__ << "No vertices in Grasp Wrench Space?! Maybe I was not initialized correctly..." << endl;
        return 0.0;
    }

    return convexHullGWS -> volume;
}


/**
 * Calculates GraspQuality measure
 *
 * @return  TRue if grasp quality was properly calculated
 */
bool iCubGraspQuality::calculateGraspQuality(void)
{
    if (!GWSCalculated) calculateGWS();

    _grasp_quality = 0.0f;

    if (!convexHullGWS || (convexHullGWS -> vertices.size() == 0))
    {
        std::cout << __FUNCTION__ << "No vertices in Grasp Wrench Space?! Maybe I was not initialized correctly..." << std::endl;
        return false;
    }

    _grasp_quality = minOffset(convexHullGWS);

    if (_verbose)
    {
        GRASPSTUDIO_INFO << std::endl;
        GRASPSTUDIO_INFO << ": GWS volume    : " << convexHullGWS -> volume << std::endl;
        GRASPSTUDIO_INFO << ": GraspQuality  : " << _grasp_quality          << std::endl;
    }

    return true;
}

/**
 * Auxiliary cross Product (simox???)
 * @param  v1  vector
 * @return     cross Product output @TODO is this needed?
 */
Eigen::Vector3f iCubGraspQuality::crossProductPosNormalInv(const VirtualRobot::MathTools::ContactPoint& v1)
{
    Eigen::Vector3f res;
    res(0) = v1.p(1) * (-v1.n(2)) - v1.p(2) * (-v1.n(1));
    res(1) = v1.p(2) * (-v1.n(0)) - v1.p(0) * (-v1.n(2));
    res(2) = v1.p(0) * (-v1.n(1)) - v1.p(1) * (-v1.n(0));
    return res;
}


} // namespace
