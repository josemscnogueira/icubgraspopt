/**************************************************************************************************
 *  File:    iCubOptimizable.hpp                                                                  *
 *  Author:  Jose Miguel Nogueira, josemscnogueira@gmail.com                                      *
 *                                                                                                *
 *  History:                                                                                      *
 **************************************************************************************************/


#ifndef _iCubOptimizable_h_
#define _iCubOptimizable_h_

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
#include <bayesopt.hpp>
#include <queue>
#include <random>

#include <tgpoptimizable.hpp>
#include <learningqueue.hpp>

#include <iCub.h>
#include <iCubHand.h>
#include <RKHS.h>
#include <iCubOptParameters.h>



/**************************************************************************************************
 *  Enumerations                                                                                  *
 **************************************************************************************************/
enum
{
    TRANS_X   = 0,
    TRANS_Y   = 1,
    TRANS_Z   = 2,
    ROT_ROLL  = 3,
    ROT_PITCH = 4,
    ROT_YAW   = 5,
    FIRST_SIN = 6,
};

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
    vector<GraspResult> applyQueryToHand      (const vectord&      query);
    double              evaluateGraspQuality  (vector<GraspResult> qualities, float& isforceclosure);
    double              evaluate              (vectord             query);
    void                showBestGrasps        (uint index,         LearningQueueWrapper& best_grasps);
    void                initSamples           (vecOfvec& xx, vectord& yy) { };
    void                getOptParams          (tgp_parameters& tgp_params, bopt_params& opt_params);
    void                getBoundingBox        (vectord& lower, vectord& upper);

    // Static Methods
    static iCubOptParameters initDefaultParams(void);
    static void              initDefaultParams(iCubOptParameters& params);
    static void              fillOthers       (iCubOptParameters& icub_params, tgp_parameters& tgp_params);

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
    GraspQualityPtr   applyQueryToHandMinimal  (const vectord& query);

private:
    // Methods
    void              setExplorationBoundingBox(void);
    void              initializeRandomDist     (double         std_dev);
    iCub::Vector09dof calculateGraspComponents (const vectord& query);
    vector<float>     calculateApproachVector  (const vectord& query);
    void              selectOptVariables       (void);
    void              chooseActiveVariables    (      vectord& query);

};

#endif // _iCubOptimizable_h_
