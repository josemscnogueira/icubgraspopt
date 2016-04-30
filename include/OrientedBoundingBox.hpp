/**************************************************************************************************
 *  File:    OrientedBoundingBox.h                                                                *
 *  Author:  Jose Miguel Nogueira, josemscnogueira@gmail.com                                      *
 *                                                                                                *
 *  History:                                                                                      *
 **************************************************************************************************/
#ifndef _OrientedBoundingBox_hpp_
#define _OrientedBoundingBox_hpp_

/**************************************************************************************************
 *  Include Files                                                                                 *
 **************************************************************************************************/
#include <vector>
#include <string>
#include <math.h>
#include <iostream>
#include <boost/pointer_cast.hpp>
#include <VirtualRobot/VirtualRobotCommon.h>
#include <Inventor/nodes/SoSeparator.h>
#include <Eigen/Dense>
#include <Eigen/LU>
#include <Eigen/Geometry>
#include <algorithm>
#include <VirtualRobot/Visualization/CoinVisualization/CoinVisualization.h>
#include <VirtualRobot/Visualization/CoinVisualization/CoinVisualizationFactory.h>


/**************************************************************************************************
 *  Used namespaces                                                                               *
 **************************************************************************************************/
using namespace VirtualRobot;

using std::string;
using std::vector;
using Eigen::Vector3f;
using Eigen::Matrix4f;
using Eigen::Matrix3f;


/**************************************************************************************************
 *  Namespace: GraspStudio                                                                        *
 **************************************************************************************************/
namespace VirtualRobot
{

/**************************************************************************************************
 *  Enumerations                                                                                  *
 **************************************************************************************************/


/**************************************************************************************************
 *  Typedefs                                                                                      *
 **************************************************************************************************/
typedef struct
{
    struct
    {
        Vector3f         max;
        Vector3f         min;
        Vector3f         center;

        vector<Vector3f> vertices;

        Vector3f         half_distance;
    }
    local;

    struct
    {
        Vector3f         max;
        Vector3f         min;
        Vector3f         center;

        vector<Vector3f> vertices;
    }
    global;

}
OOBB_Param;

/**************************************************************************************************
 *  Class: ApproachMovementSpace                                                                  *
 **************************************************************************************************/
class OrientedBoundingBox
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    // Constructors
    OrientedBoundingBox (void);
    OrientedBoundingBox (SceneObjectPtr object);

    // Methods
    void                print                  (bool     global = true);
    vector<Vector3f>    getVertices            (bool     global = true);
    vector<ObstaclePtr> createVerticesObstacles(bool     global = true);
    Matrix3f            getOrientation         (void);
    Vector3f            getCenter              (void);
    Vector3f            getMax                 (bool     global = true);
    Vector3f            getMin                 (bool     global = true);
    Vector3f            calculateScaling       (Matrix3f coordinate_system);

protected:
    // Attributes
    SceneObjectPtr                  object;

    Vector3f                        mean;
    Matrix3f                        covariance;

    Matrix3f                        orientation;

    OOBB_Param                      parameters;

    VisualizationNodePtr            visualization_model;

    // Methods
    void calculateMean          (vector<Vector3f> points, bool reset = true);
    void calculateCovariance    (vector<Vector3f> points, bool reset = true);
    void calculateBoxOrientation(void);
    void calculateParameters    (vector<Vector3f> points);
    void calculateVertices      (void);

};
typedef boost::shared_ptr<OrientedBoundingBox> OrientedBoundingBoxPtr;

} // End: namespace VirtualBox

#endif // _OrientedBoundingBox_h_
