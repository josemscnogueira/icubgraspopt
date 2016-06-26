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
#include <VirtualRobot/VirtualRobotCommon.h>
#include <Eigen/Geometry>


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
        Eigen::Vector3f              max;
        Eigen::Vector3f              min;
        Eigen::Vector3f              center;

        std::vector<Eigen::Vector3f> vertices;

        Eigen::Vector3f              half_distance;
    }
    local;

    struct
    {
        Eigen::Vector3f              max;
        Eigen::Vector3f              min;
        Eigen::Vector3f              center;

        std::vector<Eigen::Vector3f> vertices;
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
    void                         print                  (bool            global = true);
    std::vector<Eigen::Vector3f> getVertices            (bool            global = true);
    std::vector<ObstaclePtr>     createVerticesObstacles(bool            global = true);
    Eigen::Matrix3f              getOrientation         (void);
    Eigen::Vector3f              getCenter              (void);
    Eigen::Vector3f              getMax                 (bool            global = true);
    Eigen::Vector3f              getMin                 (bool            global = true);
    Eigen::Vector3f              calculateScaling       (Eigen::Matrix3f coordinate_system);

protected:
    // Attributes
    SceneObjectPtr               object;

    Eigen::Vector3f              mean;
    Eigen::Matrix3f              covariance;

    Eigen::Matrix3f              orientation;

    OOBB_Param                   parameters;

    VisualizationNodePtr         visualization_model;

    // Methods
    void calculateMean          (std::vector<Eigen::Vector3f> points, bool reset = true);
    void calculateCovariance    (std::vector<Eigen::Vector3f> points, bool reset = true);
    void calculateBoxOrientation(void);
    void calculateParameters    (std::vector<Eigen::Vector3f> points);
    void calculateVertices      (void);

    void init                   (void);
    void init                   (SceneObjectPtr   object);

};
typedef boost::shared_ptr<OrientedBoundingBox> OrientedBoundingBoxPtr;

} // End: namespace VirtualBox

#endif // _OrientedBoundingBox_h_
