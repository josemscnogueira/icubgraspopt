/**************************************************************************************************
 *  File:    iCub.cpp                                                                             *
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


/**************************************************************************************************
 *  Include Files                                                                                 *
 **************************************************************************************************/
#include <iCub.hpp>

#include <Eigen/Geometry>

/**************************************************************************************************
 *  Used namespaces                                                                               *
 **************************************************************************************************/
using namespace VirtualRobot;
using namespace GraspStudio;

using std::cout;
using std::endl;
using std::string;
using std::vector;


/**************************************************************************************************
 *  Namespace: iCub                                                                               *
 **************************************************************************************************/
namespace iCub
{


/**************************************************************************************************
 *  Procecure                                                                                     *
 *                                                                                                *
 *  Description: isLoaded                                                                         *
 *  Class      : iCubRobot                                                                        *
 **************************************************************************************************/
bool iCubRobot::isLoaded(bool verbose)
{
    if (robot)
    {
        if (verbose) cout << endl << "iCub loaded [OK]" << endl;

        return true;
    }
    else
    {
        if (verbose) cout << endl << "iCub not correctly loaded [ERROR]" << endl;

        return false;
    }
}

/**************************************************************************************************
 *  Procecure                                                                                     *
 *                                                                                                *
 *  Description: isLoaded                                                                         *
 *  Class      : iCubRobot                                                                        *
 **************************************************************************************************/
bool iCubRobot::hasCurrentRobot(void)
{
    if (current_robot_part) return true;
    else                    return false;
}


/**************************************************************************************************
 *  Procecure                                                                                     *
 *                                                                                                *
 *  Description: Constructor                                                                      *
 *  Class      : iCubRobot                                                                        *
 **************************************************************************************************/
iCubRobot::iCubRobot()
{
    robot = RobotIO::loadRobot("/home/zenogueira/Programs/simox/VirtualRobot/data/robots/iCub/iCub.xml");

    isLoaded(true);

    reset();
}


/**************************************************************************************************
 *  Procecure                                                                                     *
 *                                                                                                *
 *  Description: Constructor                                                                      *
 *  Class      : iCubRobot                                                                        *
 **************************************************************************************************/
iCubRobot::iCubRobot(string filelocation)
{
    robot = RobotIO::loadRobot(filelocation);

    isLoaded(true);

    reset();
}


/**************************************************************************************************
 *  Procecure                                                                                     *
 *                                                                                                *
 *  Description: Constructor                                                                      *
 *  Class      : iCubRobot                                                                        *
 **************************************************************************************************/
iCubRobot::iCubRobot(ScenePtr scene, string robotname)
{
    robot = scene -> getRobot(robotname);

    if(!robot)
    {
        cout << endl << "[Error] No robot found. Forcing shutdown." << cout;

        exit(-1);
    }

    RobotConfigPtr config = scene -> getRobotConfig(robotname, "start config");

    if (config) robot -> setConfig(config);

    isLoaded(true);

    reset();
}


/**************************************************************************************************
 *  Procecure                                                                                     *
 *                                                                                                *
 *  Description: Constructor                                                                      *
 *  Class      : iCubRobot                                                                        *
 **************************************************************************************************/
iCubRobot::iCubRobot(RobotPtr robot)
{
    if (!robot)
    {
        cout << endl << "[Error] Robot could not be loaded." << endl;

        exit(-1);
    }

    this -> robot = robot;

    isLoaded(true);

    reset();
}


/**************************************************************************************************
 *  Procecure                                                                                     *
 *                                                                                                *
 *  Description: Constructor                                                                      *
 *  Class      : iCubRobot                                                                        *
 **************************************************************************************************/
iCubRobot::iCubRobot(ScenePtr scene, string robotname, string start_node)
{
    RobotPtr root_robot = scene -> getRobot(robotname);

    // Check for robot
    if(!root_robot)
    {
        cout << endl << "[Error] No robot found. Forcing shutdown." << cout;

        exit(-1);
    }

    // Setup robot initial configuration
    RobotConfigPtr config = scene -> getRobotConfig(robotname, "start config");

    if (config) root_robot -> setConfig(config);

    // Exctract Subpart
    RobotNodePtr robot_part = root_robot -> getRobotNode(start_node);

    if (robot_part) robot  =  root_robot -> extractSubPart(robot_part, start_node, root_robot -> getType());

    // Check for robot part
    if(!robot)
    {
        cout << endl << "[Error] Robot Part not found. Forcing shutdown." << cout;

        exit(-1);
    }


    isLoaded(true);

    reset();
}



/**************************************************************************************************
 *  Procecure                                                                                     *
 *                                                                                                *
 *  Description: reset                                                                            *
 *  Class      : iCubRobot                                                                        *
 **************************************************************************************************/
void iCubRobot::reset()
{
    // Initialize iCub Variables
        // Current Robot Parts
    current_robot_part = robot;

        // iCub Hand
    hand.reset(new iCubHand(current_robot_part));
}


/**************************************************************************************************
 *  Procecure                                                                                     *
 *                                                                                                *
 *  Description: selectRobotNode                                                                  *
 *  Class      : iCubRobot                                                                        *
 **************************************************************************************************/
void iCubRobot::selectSubRobot(string start_node)
{
    RobotNodePtr robot_part = robot -> getRobotNode(start_node);

    // Update Robot
    if (robot_part) current_robot_part  = robot -> extractSubPart(robot_part, start_node, robot -> getType());
    else return;

    // Update Hand
    hand -> loadHand(current_robot_part);

}

RobotPtr iCubRobot::extractSubPart(string start_node)
{
    RobotPtr null_robot;

    RobotNodePtr robot_part = robot -> getRobotNode(start_node);

    if (!robot_part) return null_robot;
    else             return robot -> extractSubPart(robot_part, start_node, robot -> getType());
}


/**************************************************************************************************
 *  Procecure                                                                                     *
 *                                                                                                *
 *  Description: controlHand                                                                      *
 *  Class      : iCubRobot                                                                        *
 **************************************************************************************************/
void iCubRobot::controlHand(Vector09dof actuated_joints, int mode)
{
    hand -> controlHand(actuated_joints, mode);
}


/**************************************************************************************************
 *  Procecure                                                                                     *
 *                                                                                                *
 *  Description: closeHand                                                                        *
 *  Class      : iCubRobot                                                                        *
 **************************************************************************************************/
GraspQualityMeasureWrenchSpacePtr iCubRobot::closeHand(Vector09dof auxiliary_components, uint grasp, SceneObjectSetPtr obstacles, uint &number_of_contacts)
{
    return hand -> closeGrasp(grasp, auxiliary_components, obstacles, number_of_contacts);
}

/**************************************************************************************************
 *  Procecure                                                                                     *
 *                                                                                                *
 *  Description: openHand                                                                         *
 *  Class      : iCubRobot                                                                        *
 **************************************************************************************************/
void iCubRobot::openHand(Vector09dof auxiliary_components, uint grasp)
{
    hand -> openGrasp(grasp, auxiliary_components);
}


/**************************************************************************************************
 *  Procecure                                                                                     *
 *                                                                                                *
 *  Description: getHandState                                                                     *
 *  Class      : iCubRobot                                                                        *
 **************************************************************************************************/
uint iCubRobot::getHandState(void)
{
    return hand -> getState();
}


/**************************************************************************************************
 *  Procecure                                                                                     *
 *                                                                                                *
 *  Description: getEEF                                                                           *
 *  Class      : iCubRobot                                                                        *
 **************************************************************************************************/
EndEffectorPtr iCubRobot::getEEF(void)
{
    return hand -> getEEF();
}


/**************************************************************************************************
 *  Procecure                                                                                     *
 *                                                                                                *
 *  Description: approachHand                                                                     *
 *  Class      : iCubRobot                                                                        *
 **************************************************************************************************/
void iCubRobot::approachHand(SceneObjectPtr object, vector<int> sliders, uint mode, bool reset)
{
    if (!space || reset) space.reset(new ApproachMovementSpace(object, this -> getEEF()));

    Vector6f                 parameters;

    switch (mode)
    {
        case ESPH :
        {
            parameters(POSI_THETA ) = ( ((float)sliders[POSI_THETA ] -   0.0f) * 1 * M_PI / 1000.0 );
            parameters(POSI_PHI   ) = ( ((float)sliders[POSI_PHI   ] -   0.0f) * 2 * M_PI / 1000.0 );
            parameters(POSI_RHO   ) = ( ((float)sliders[POSI_RHO   ] -   0.0f) * 1 *    1 /    1.0 );
            parameters(HAND_ANGLEZ) = ( ((float)sliders[HAND_ANGLEZ] -   0.0f) * 2 * M_PI / 1000.0 );
            parameters(HAND_DX    ) = ( ((float)sliders[HAND_DX    ] - 500.0f) * 1 *    1 /   10.0 );
            parameters(HAND_DY    ) = ( ((float)sliders[HAND_DY    ] - 500.0f) * 1 *    1 /   10.0 );

            space -> moveHandRelativeToObject(parameters);

            break;
        }

        default :
        {
            parameters(POS_DX     ) = (float)sliders[POS_DX];
            parameters(POS_DY     ) = (float)sliders[POS_DY];
            parameters(POS_DZ     ) = (float)sliders[POS_DZ];
            parameters(ANG_Z1     ) = (float)sliders[ANG_Z1];
            parameters(ANG_Y1     ) = (float)sliders[ANG_Y1];
            parameters(ANG_Z2     ) = (float)sliders[ANG_Z2];

            space -> moveHandDistanceClass(mode, parameters);

            break;
        }
    }
}

/**************************************************************************************************
 *  Procecure                                                                                     *
 *                                                                                                *
 *  Description: approachHand                                                                     *
 *  Class      : iCubRobot                                                                        *
 **************************************************************************************************/
Eigen::Matrix4f iCubRobot::approachHand(SceneObjectPtr object, vector<float> learn_parameters, uint mode, bool reset)
{
    if (learn_parameters.size() != 6) return Eigen::Matrix4f();

    if (!space || reset) space.reset(new ApproachMovementSpace(object, this -> getEEF()));

    Vector6f parameters;

    parameters(POS_DX) = (float)learn_parameters[POS_DX];
    parameters(POS_DY) = (float)learn_parameters[POS_DY];
    parameters(POS_DZ) = (float)learn_parameters[POS_DZ];
    parameters(ANG_Z1) = (float)learn_parameters[ANG_Z1];
    parameters(ANG_Y1) = (float)learn_parameters[ANG_Y1];
    parameters(ANG_Z2) = (float)learn_parameters[ANG_Z2];

    return space -> moveHandDistanceClass(mode, parameters);
}


/**************************************************************************************************
 *  Procecure                                                                                     *
 *                                                                                                *
 *  Description: approachHand                                                                     *
 *  Class      : iCubRobot                                                                        *
 **************************************************************************************************/
void iCubRobot::approachHand(SceneObjectPtr object, vector<float> learn_parameters, uint mode,
                             std::vector<double>& position,
                             std::vector<double>& orientation,
                             bool reset)
{

    Eigen::Matrix4f hand_pose = approachHand(object, learn_parameters, mode, reset);
    Eigen::Matrix4f obj_pose  = object -> getGlobalPose();

    position   .clear(); position   .reserve(3);
    orientation.clear(); orientation.reserve(4);

    // Define hand posiition
    for (size_t idx = 0; idx < 3; idx++)
    {
        position[idx] = hand_pose(4,idx) - obj_pose(4,idx);
    }

    Eigen::Matrix3f r_hand = hand_pose.topLeftCorner(3,3);
    Eigen::Matrix3f r_obj  = obj_pose .topLeftCorner(3,3);

    // Define orientation
    Eigen::Quaternionf orientation_hand(r_hand);
    Eigen::Quaternionf orientation_obj (r_obj );
    Eigen::Quaternionf orientation_q = orientation_hand * orientation_obj.inverse();

    orientation[0] = orientation_q.w();
    orientation[1] = orientation_q.x();
    orientation[2] = orientation_q.y();
    orientation[3] = orientation_q.z();
}


/**************************************************************************************************
 *  Procecure                                                                                     *
 *                                                                                                *
 *  Description: calculateNumberOfDimensionsForLearning                                           *
 *  Class      : iCubRobot                                                                        *
 **************************************************************************************************/
uint iCubRobot::calculateNumberOfDimensionsForLearning(uint mode)
{
    return hand -> calculateNumberOfDimensionsForLearning(mode);
}


/**************************************************************************************************
 *  Procecure                                                                                     *
 *                                                                                                *
 *  Description: getCollisionModel                                                                *
 *  Class      : iCubRobot                                                                        *
 **************************************************************************************************/
vector<CollisionModelPtr> iCubRobot::getCollisionModel(void)
{
    return robot -> getCollisionModels();
}

/**************************************************************************************************
 *  Procecure                                                                                     *
 *                                                                                                *
 *  Description: getEEFGlobalPose                                                                 *
 *  Class      : iCubRobot                                                                        *
 **************************************************************************************************/
Matrix4f iCubRobot::getEEFGlobalPose(void)
{
    return hand -> getEEFGlobalPose();
}


/**************************************************************************************************
 *  Procecure                                                                                     *
 *                                                                                                *
 *  Description: printEEFState                                                                    *
 *  Class      : iCubRobot                                                                        *
 **************************************************************************************************/
void iCubRobot::printEEFState(void)
{
    return hand -> printEEFState();
}

} // End: namespace iCub
