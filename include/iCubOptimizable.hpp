/**************************************************************************************************
 *  File:    iCubOptimizable.hpp                                                                  *
 *  Author:  Jose Miguel Nogueira, josemscnogueira@gmail.com                                      *
 *                                                                                                *
 *  History:                                                                                      *
 **************************************************************************************************/


#ifndef _iCubOptimizable_hpp_
#define _iCubOptimizable_hpp_

/**************************************************************************************************
 *  Include Files                                                                                 *
 **************************************************************************************************/
#include <stdlib.h>
#include <vector>
#include <string>
#include <math.h>
#include <iostream>
#include <boost/pointer_cast.hpp>
#include <VirtualRobot/VirtualRobotCommon.h>
#include <Eigen/Dense>
#include <bayesopt/bayesopt.hpp>
#include <queue>
#include <random>

#include <tgpoptimizable.hpp>
#include <learningqueue.hpp>

#include <iCub.hpp>
#include <iCubHand.hpp>
#include <iCubOptParameters.hpp>


/**************************************************************************************************
 *  Typedefs                                                                                      *
 **************************************************************************************************/
typedef GraspQualityMeasureWrenchSpacePtr         GraspQualityPtr;
typedef vector<std::normal_distribution<double> > GaussianVector;


/**************************************************************************************************
 *  Class: GraspResult                                                                            *
 **************************************************************************************************/
class GraspResult
{
public:
    double measure;
    double volume;
    bool   force_closure;

    GraspResult(void)
    {
        measure       = 0;
        volume        = 0;
        force_closure = false;
    }

    GraspResult(GraspQualityMeasureWrenchSpacePtr space)
    {
        measure       = space -> getGraspQuality      ();
        volume        = space -> getVolumeGraspMeasure();
        force_closure = space -> isGraspForceClosure  ();
    }
};

namespace bayesopt {

/**************************************************************************************************
 *  Class: GraspOptimization                                                                      *
 **************************************************************************************************/
class iCubOptimizable : public TGPOptimizable
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    // Constructors
    iCubOptimizable(iCub::iCubRobotPtr robot, VirtualRobot::SceneObjectPtr object, iCubOptParameters& params);

    // Destructors
    ~iCubOptimizable(void) {};

    // Methods
    vector<GraspResult> applyQueryToHand      (const vectord&      query, std::vector<double>& position, std::vector<double>& orientation);
    double              evaluateGraspQuality  (vector<GraspResult> qualities, float& isforceclosure);
    double              evaluate              (vectord             query);
    void                showBestGrasps        (uint index,         LearningQueueWrapper& best_grasps);
    void                initSamples           (vecOfvec& xx, vectord& yy) { };
    void                getOptParams          (TgpParameters& tgp_params, Parameters& opt_params);
    void                getBoundingBox        (vectord& lower, vectord& upper);

protected:
    // Attributes
        // Robot and Objects
    iCub::iCubRobotPtr                _robot;
    VirtualRobot::SceneObjectSetPtr   _object_set;
    VirtualRobot::SceneObjectPtr      _object;
    VirtualRobot::CollisionCheckerPtr _col_checker;
        // Grasp
    uint                              _number_of_contacts;

        // Noise
    GaussianVector                    _noise_distribution;

        // Optimization
    iCubOptParameters&                _icubparams;
    vector<bool>                      _active_variables_mask;
    uint                              _original_dim;

    // Methods
    GraspQualityPtr   applyQueryToHandMinimal  (const vectord& query, std::vector<double>& position, std::vector<double>& orientation);

private:
    // Methods
    void              setExplorationBoundingBox(void);
    void              initializeRandomDist     (double         std_dev);
    iCub::Vector09dof calculateGraspComponents (const vectord& query);
    vector<float>     calculateApproachVector  (const vectord& query);
    void              selectOptVariables       (void);
    void              chooseActiveVariables    (      vectord& query);

};

} // End of namespace bayesopt

#endif // _iCubOptimizable_h_
