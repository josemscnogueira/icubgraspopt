/**************************************************************************************************
 *  File:    iCub.h                                                                               *
 *  Author:  Jose Miguel Nogueira, josemscnogueira@gmail.com                                      *
 *                                                                                                *
 *  History: 20141126   File Creation.                                                            *
 *           20141202   Hand handling created. It is possible to control the hand using the       *
 *                      the actuated joints like in the real robot.                               *
 *           20141203   Removed hand variables from iCub class and created a new class iCubHand   *
 *           20141213   Hand can be controlled through the physical joints, motor joints or       *
 *                      thorugh a set of pre-trained grasps.                                      *
 *                      Hand now closes for a specific grasp preshape. It checks for collisions.  *
 **************************************************************************************************/
#ifndef _iCub_hpp_
#define _iCub_hpp_

/**************************************************************************************************
 *  Include Files                                                                                 *
 **************************************************************************************************/
#include <vector>
#include <string>
#include <iostream>
#include <boost/pointer_cast.hpp>
#include <VirtualRobot/VirtualRobotCommon.h>
#include <Eigen/Dense>
#include <algorithm>

#include <iCubHand.hpp>
#include <ApproachMovementSpace.hpp>

/**************************************************************************************************
 *  Used namespaces                                                                               *
 **************************************************************************************************/
using namespace VirtualRobot;
using namespace GraspStudio;

using std::string;
using std::vector;
using Eigen::Matrix;
using Eigen::Matrix4f;


/**************************************************************************************************
 *  Namespace: iCub                                                                               *
 **************************************************************************************************/
namespace iCub
{

/**************************************************************************************************
 *  Class: iCubRobot                                                                              *
 **************************************************************************************************/
class iCubRobot
{

public:
    //Attributes
    RobotPtr current_robot_part;

    // Constructors
    iCubRobot(void);
    iCubRobot(string   filelocation);
    iCubRobot(ScenePtr scene, string robotname = "iCub");
    iCubRobot(ScenePtr scene, string robotname, string start_node);
    iCubRobot(RobotPtr robot);

    //Methods
    void                              reset          (void);
    bool                              hasCurrentRobot(void);
    void                              selectSubRobot (string      node_name );
    RobotPtr                          extractSubPart (string      start_node);
    void                              controlHand    (Vector09dof actuated_joints,       int  mode);
    GraspQualityMeasureWrenchSpacePtr closeHand      (Vector09dof auxiliary_components, uint  grasp, SceneObjectSetPtr obstacles, uint &number_of_contacts);
    void                              openHand       (Vector09dof auxiliary_components, uint  grasp);
    uint                              getHandState   (void);
    EndEffectorPtr                    getEEF         (void);
    void                              approachHand   (SceneObjectPtr object, vector<int>   sliders         , uint mode, bool reset = false);
    void                              approachHand   (SceneObjectPtr object, vector<float> learn_parameters, uint mode, bool reset = false);

    uint                              calculateNumberOfDimensionsForLearning(uint mode);

    vector<CollisionModelPtr>         getCollisionModel(void);
    Matrix4f                          getEEFGlobalPose (void);
    void                              printEEFState    (void);

private:
    // Attributes
    RobotPtr                 robot;
    iCubHandPtr              hand;
    ApproachMovementSpacePtr space;

    // Methods
    bool isLoaded        (bool verbose = true);
    void updateHandNodes (void);
};
typedef boost::shared_ptr<iCubRobot> iCubRobotPtr;

} // End: namespace iCub


#endif // _iCub_h_
