/**************************************************************************************************
 *  File:    iCubHand                                                                             *
 *  Author:  Jose Miguel Nogueira, josemscnogueira@gmail.com                                      *
 *                                                                                                *
 *  History: 20141203   File Creation.                                                            *
 *           20141213   Hand can be controlled through the physical joints, motor joints or       *
 *                      thorugh a set of pre-trained grasps.                                      *
 *                      Hand now closes for a specific grasp preshape. It checks for collisions.  *
 **************************************************************************************************/
#ifndef _iCubHand_hpp_
#define _iCubHand_hpp_


/**************************************************************************************************
 *  Include Files                                                                                 *
 **************************************************************************************************/
#include <vector>
#include <string>
#include <iostream>
#include <boost/shared_ptr.hpp>
#include <VirtualRobot/VirtualRobotCommon.h>
#include <Eigen/Dense>
#include <algorithm>
#include <GraspPlanning/GraspStudio.h>
#include <GraspPlanning/GraspQuality/GraspQualityMeasureWrenchSpace.h>


/**************************************************************************************************
 *  Defines                                                                                       *
 **************************************************************************************************/
#define NUMBER_OF_GRASPS 9

/**************************************************************************************************
 *  Used namespaces                                                                               *
 **************************************************************************************************/
using namespace VirtualRobot;

using std::string;
using std::stringstream;
using std::vector;

using Eigen::Matrix;
using Eigen::Matrix4f;

using GraspStudio::GraspQualityMeasureWrenchSpace;
using GraspStudio::GraspQualityMeasureWrenchSpacePtr;


/**************************************************************************************************
 *  Namespace: iCub                                                                               *
 **************************************************************************************************/
namespace iCub
{


/**************************************************************************************************
 *  Enumerations                                                                                  *
 **************************************************************************************************/
enum
{
    JOINTS_MODE    = 0,
    TRYPOD         = 1,
    PALMAR_PINCH   = 2,
    LATERAL           ,
    WRITING_TRIPOD    ,
    PARALLEL_EXTENSION,
    ADDUCTION_GRIP    ,
    TIP_PINCH         ,
    LATERAL_TRIPOD    ,
    POWER_GRIP        ,
};

enum
{
    PRINT_NOTHING          = 0    ,
    PRINT_HAND_NODES       = 1    ,
    PRINT_GRASP_RESULT     = 2    ,
    PRINT_DELETED_CONTACTS = 3    ,
    PRINT_CONTACTS_INFO    = 4    ,
};

enum
{
    CLOSED = 0,
    OPENED = 1,
};


/**************************************************************************************************
 *  Typedefs                                                                                      *
 **************************************************************************************************/
typedef Matrix<float, 9, 1>   Vector09dof;
typedef Matrix<float,20, 1>   Vector20dof;
typedef Matrix<float, 9, 9>   Matrixf9x9;


typedef struct
{
    struct
    {
        Matrix<float,20, 9>   actuacted_to_physical;
    }
    maps;

    Vector20dof values;
    Vector20dof offsets;
}
Type_Handjoints;



typedef struct
{
    Vector09dof mean;
    Matrixf9x9  map;

    struct
    {
        int    primary;
        int    auxiliary;
    }
    components;
}
Type_Grasps;

/**************************************************************************************************
 *  Class: iCubHand                                                                               *
 **************************************************************************************************/
class iCubHand
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    // Constructors
    iCubHand(void);
    iCubHand(RobotPtr robot);

    // Methods
    void                               loadHand                              (RobotPtr    robot);
    void                               controlHand                           (Vector09dof actuated_joints, int          mode);
    GraspQualityMeasureWrenchSpacePtr  closeGrasp                            (uint        grasp,           Vector09dof  auxiliary_components, SceneObjectSetPtr obstacles, uint &number_of_contacts, float step = 0.01, float open_offeset = 0);
    void                               openGrasp                             (uint        grasp,           Vector09dof  auxiliary_components,                                                                           float open_offeset = 0);
    uint                               getState                              (void);
    EndEffectorPtr                     getEEF                                (void);
    uint                               getNumberOfAuxiliaryGraspParameters   (uint mode);
    uint                               calculateNumberOfDimensionsForLearning(uint mode);

    EndEffector::ContactInfoVector     getContactPoints                      (void);
    Matrix4f                           getEEFGlobalPose                      (void);
    RobotConfigPtr                     getEEFJointConfig                     (void);
    void                               printEEFState                         (void);
    void                               printContactInfo                      (EndEffector::ContactInfo       contact);
    void                               printContactInfoVector                (EndEffector::ContactInfoVector contacts);


protected:
    // Attributes
    uint32_t                           verbose;

    // Methods
    void                               setVerbose (uint32_t verbose);


private:
    // Attributes
    EndEffectorPtr                     endeffector;
    vector<RobotNodePtr>               nodes;

    Type_Handjoints                    joints;
    vector<Type_Grasps>                grasps;
    bool                               state;
    vector<uint32_t>                   actor_to_joint_mask;

    bool                               allow_object_trespass;
    GraspQualityMeasureWrenchSpacePtr  grasp_quality;
    bool                               grasp_quality_initialized;
    EndEffector::ContactInfoVector     contacts;

    // Methods
    void                               initializeConstVariables(void);
    void                               getJointValues          (void);
    void                               setJointValues          (void);
    void                               updateHandNodes         (void);
    bool                               checkIfSelfColides      (uint32_t          &active_mask);
    bool                               checkIfColides          (uint32_t          &active_mask, SceneObjectSetPtr obstacles);
    EndEffector::ContactInfoVector     getContacts             (SceneObjectSetPtr obstacles,    float             step = 0.01);

};
typedef boost::shared_ptr<iCubHand> iCubHandPtr;


} // End: namespace iCub

#endif // _iCubHand_h_
