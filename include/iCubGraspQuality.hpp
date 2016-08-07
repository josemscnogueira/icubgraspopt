/**************************************************************************************************
 *  File:    iCubGraspQuality.h                                                                   *
 *  Author:  Jose Miguel Nogueira, josemscnogueira@gmail.com                                      *
 *                                                                                                *
 *  History: Adapted from Simox code by ...                                                       *
 **************************************************************************************************/

#ifndef __ICUBGRASPQUALITY_H_
#define __ICUBGRASPQUALITY_H_

/**************************************************************************************************
 *  Include Files                                                                                 *
 **************************************************************************************************/
#include <Eigen/Core>
#include <VirtualRobot/MathTools.h>
#include <GraspPlanning/GraspStudio.h>
#include <GraspPlanning/ConvexHullGenerator.h>
#include <GraspPlanning/ContactConeGenerator.h>

namespace GraspStudio
{

class iCubGraspQuality
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    // Constructors
    iCubGraspQuality(float object_length, float unit_force = 1.0f, float friction_coeff = 0.35f, int friction_samples = 8);

    // Destructor
    ~iCubGraspQuality() {};

    // Virtual Methods
    virtual float getGraspQuality      (void);
    virtual float getVolumeGraspMeasure(void);
    virtual bool  isGraspForceClosure  (void);

    // ConvexHull Methods
    virtual VirtualRobot::MathTools::ConvexHull6DPtr getConvexHullGWS(void) { return convexHullGWS; }

    void calculateGWS(void);
    bool GWSExists   (void) { return GWSCalculated; }

    VirtualRobot::MathTools::ContactPoint getCenterGWS(void) { return convexHullCenterGWS; }

    // Contact Points Methods
    virtual void setContactPoints     (const std::vector<VirtualRobot::MathTools::ContactPoint>& contactPoints);
    virtual bool calculateGraspQuality(void);

    static std::vector<VirtualRobot::MathTools::ContactPoint> createWrenchPoints(std::vector<VirtualRobot::MathTools::ContactPoint>& points,
                                                                                 const Eigen::Vector3f&                              centerOfModel,
                                                                                 float                                               objectLengthMM);

    //! Goes through all facets of convex hull and searches the minimum distance to it's center
    static float minOffset(VirtualRobot::MathTools::ConvexHull6DPtr ch);

protected:
    //Methods
    VirtualRobot::MathTools::ConvexHull6DPtr calculateConvexHull     (std::vector<VirtualRobot::MathTools::ContactPoint>& points);
    VirtualRobot::MathTools::ContactPoint    calculateHullCenter     (VirtualRobot::MathTools::ConvexHull6DPtr            hull  );
    float                                    minDistanceToGWSHull    (VirtualRobot::MathTools::ContactPoint&              point );
    bool                                     isOriginInGWSHull       (void);
    void                                     printContacts           (std::vector<VirtualRobot::MathTools::ContactPoint>& points);
    static Eigen::Vector3f                   crossProductPosNormalInv(const VirtualRobot::MathTools::ContactPoint&        v1    );

    //For safety
    bool GWSCalculated;

    //For Object and Grasp Wrench Space Calculation
    VirtualRobot::MathTools::ConvexHull6DPtr convexHullGWS;
    VirtualRobot::MathTools::ContactPoint    convexHullCenterGWS;

    float                                              _object_length;
    float                                              _unit_force;
    float                                              _friction_coeff;
    int                                                _friction_samples;
    ContactConeGeneratorPtr                            _coneGenerator;
    std::vector<VirtualRobot::MathTools::ContactPoint> _contactPoints;  // in MM
    std::vector<VirtualRobot::MathTools::ContactPoint> _contactPointsM; // converted to M
    bool                                               _verbose;
    float                                              _grasp_quality;
};

} // end of namespace

#endif // __ICUBGRASPQUALITY_H_
