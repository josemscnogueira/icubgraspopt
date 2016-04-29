/**************************************************************************************************
 *  File:    ApproachMovementSpace.h                                                              *
 *  Author:  Jose Miguel Nogueira, josemscnogueira@gmail.com                                      *
 *                                                                                                *
 *  History:                                                                                      *
 **************************************************************************************************/
#ifndef _ApproachMovementSpace_h_
#define _ApproachMovementSpace_h_

/**************************************************************************************************
 *  Include Files                                                                                 *
 **************************************************************************************************/
#include <vector>
#include <string>
#include <math.h>
#include <iostream>
#include <boost/pointer_cast.hpp>
#include <VirtualRobot/VirtualRobotCommon.h>
#include <GraspPlanning/ApproachMovementGenerator.h>
#include <Eigen/Dense>
#include <Eigen/LU>
#include <Eigen/Geometry>
#include <algorithm>

#include <OrientedBoundingBox.h>


/**************************************************************************************************
 *  Used namespaces                                                                               *
 **************************************************************************************************/
using namespace VirtualRobot;

using std::string;
using std::vector;
using Eigen::Vector3f;
using Eigen::Matrix4f;
using Eigen::Matrix3f;
using Eigen::MatrixXf;
using Eigen::AngleAxisf;
using Eigen::Quaternion;


/**************************************************************************************************
 *  Namespace: GraspStudio                                                                        *
 **************************************************************************************************/
namespace GraspStudio
{

/**************************************************************************************************
 *  Enumerations                                                                                  *
 **************************************************************************************************/
enum
{
    POSI_THETA    = 0,
    POSI_PHI      = 1,
    POSI_RHO      = 2,
    HAND_ANGLEZ   = 3,
    HAND_DX       = 4,
    HAND_DY       = 5,
};

enum
{
    POS_DX    = 0,
    POS_DY    = 1,
    POS_DZ    = 2,
    ANG_Z1    = 3,
    ANG_Y1    = 4,
    ANG_Z2    = 5,
};

enum
{
    RIFR = 0,
    RIUP    ,
    RIBA    ,
    RIDO    ,
    FRLE    ,
    FRUP    ,
    FRRI    ,
    FRDO    ,
    LEBA    ,
    LEUP    ,
    LEFR    ,
    LEDO    ,
    BARI    ,
    BAUP    ,
    BALE    ,
    BADO    ,
    TOFR    ,
    TOLE    ,
    TOBA    ,
    TORI    ,
    BOFR    ,
    BORI    ,
    BOBA    ,
    BOLE    ,
    ESPH    ,
};

/**************************************************************************************************
 *  Typedefs                                                                                      *
 **************************************************************************************************/
typedef Eigen::Matrix<float, 4, 1>   Vector4f;
typedef Eigen::Matrix<float, 6, 1>   Vector6f;
typedef Eigen::Matrix<float, 7, 1>   Vector7f;

/**************************************************************************************************
 *  Class: ApproachMovementSpace                                                                  *
 **************************************************************************************************/
class ApproachMovementSpace : public ApproachMovementGenerator
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	// Constructors
	ApproachMovementSpace(SceneObjectPtr object, EndEffectorPtr eef);

	// Methods
	void                    setParameters           (Vector6f approach_parameters);
	Matrix4f                createNewApproachPose   (void);
    Matrix4f                moveHandRelativeToObject(Vector6f approach_parameters        , bool update = true);
	Matrix4f                moveHandRelativeToObject(float    theta, float phi, float rho, bool update = true);
    Matrix4f                moveHandDistanceClass   (uint     mode ,                       bool update = true);
    Matrix4f                moveHandDistanceClass   (uint     mode , Vector6f parameters , bool update = true);

    Matrix4f                getEEFGlobalPose        (void);

protected:
    // Attributes
	SceneObjectPtr          object;
    BoundingBox             boundingbox;
    OrientedBoundingBoxPtr  oobb;
	EndEffectorPtr          eef;

	Matrix4f                offset;

	Vector6f                approach_parameters;  // hand orientation, approach orientation, distance

	// Methods
	Matrix3f                calculateRotationMatrixEulerZYZ(float      angle_z1, float angle_y1 , float angle_z2);
    Matrix3f                calculateRotationSimpleXYZ     (float      angle_x , float angle_y  , float angle_z );
	Vector3f                calculateCarthesianCoordinates (float      theta   , float phi      , float rho     );

    // Static Methods
        // Generic Matrix Operations
    static Vector3f         normalizeVector3f              (Vector3f   vector1);
    static Matrix3f         gramSchmidtNormalization3f     (Matrix3f   matrix );
    static Matrix3f         swapColumns                    (Matrix3f   matrix, uint  column1, uint column2);
    static void             invertCol                      (Matrix3f  &matrix, uint  column);
};
typedef boost::shared_ptr<ApproachMovementSpace> ApproachMovementSpacePtr;

} // End: namespace GraspStudio

#endif // _ApproachMovementSpace_h_