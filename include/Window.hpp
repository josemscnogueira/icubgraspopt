/**************************************************************************************************
 *  File:    Window.h                                                                             *
 *  Author:  Jose Miguel Nogueira, josemscnogueira@gmail.com                                      *
 *                                                                                                *
 *  History: 20141126   File Creation.                                                            *
 *           201412**   ------ missing history ------                                             *
 *                      It's possible to manipulate every joint of the iCub                       *
 *                      There is a mode where one can only view the iCub hand                     *
 *                      The hand can be manipulated through the real actuated motor joints        *
 *                      The hand can be manipulated throught a set of pre-trained grasps          *
 *           20141213   The hand now closes using a pre-trained grasp. It checks for collisions.  *
 **************************************************************************************************/

#ifndef _Window_hpp_
#define _Window_hpp_


/**************************************************************************************************
 *  Include Files                                                                                 *
 **************************************************************************************************/
#include <fstream>
#include <iomanip>
#include <stdlib.h>
#include <string.h>
#include <vector>
#include <QtCore/QtGlobal>
#include <QtGui/QtGui>
#include <QtCore/QtCore>
#include <QFileDialog>
#include <boost/shared_ptr.hpp>
#include <boost/thread.hpp>
#include <time.h>
#include <iostream>

#include <VirtualRobot/Visualization/CoinVisualization/CoinVisualization.h>
#include <Inventor/sensors/SoTimerSensor.h>
#include <Inventor/nodes/SoEventCallback.h>
#include <Inventor/Qt/viewers/SoQtExaminerViewer.h>
#include <Inventor/Qt/SoQt.h>
#include <Inventor/nodes/SoSeparator.h>
#include "Inventor/actions/SoLineHighlightRenderAction.h"

#include <learningqueue.hpp>

#include <iCub.h>
#include <iCubHand.h>
#include <OrientedBoundingBox.h>

#include <ui_iCubSimulator.h>

/**************************************************************************************************
 *  Used namespaces                                                                               *
 **************************************************************************************************/
using namespace iCub;
using namespace Ui;
using namespace VirtualRobot;

using std::vector;
using std::setw;
using std::ofstream;


/**************************************************************************************************
 *  Typedefs                                                                                      *
 **************************************************************************************************/
typedef boost::shared_ptr<VirtualRobot::CoinVisualization> CoinVistualizationPtr;


/**************************************************************************************************
 *  Class: ShowWindow                                                                             *
 **************************************************************************************************/
class ShowWindow : public QMainWindow
{
    Q_OBJECT

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    // Constructors
    ShowWindow(Qt::WFlags flags = 0);
    ShowWindow(uint test_index, uint total_tests, Qt::WFlags flags = 0);

    // Destructors
    ~ShowWindow(void);

    // Methods
    int  main(void);
    void quit(void);

public slots:
    // Methods
    void searchObjectTest        (void);
    void optimizeGrasp           (void);
    void graspMetricTest         (void);

    void selectRNS               (int index);
    void selectJoint             (int index);
    void selectSliderMode        (int mode);
    void selectMoveHand          (int mode);
    void jointValueChanged       (int slide_pos);


    void handJointSlider1        (int pos);
    void handJointSlider2        (int pos);
    void handJointSlider3        (int pos);
    void handJointSlider4        (int pos);
    void handJointSlider5        (int pos);
    void handJointSlider6        (int pos);
    void handJointSlider7        (int pos);
    void handJointSlider8        (int pos);
    void handJointSlider9        (int pos);

    void moveHandSlider1         (int pos);
    void moveHandSlider2         (int pos);
    void moveHandSlider3         (int pos);
    void moveHandSlider4         (int pos);
    void moveHandSlider5         (int pos);
    void moveHandSlider6         (int pos);

    void selectGraspOpt          (int pos);

    void resetGraspSliders       (void);
    void resetMoveHandSliders    (void);
    void resetSceneryAll         (void);
    void loadHand                (void);
    void closeGrasp              (void);

protected:
    // Attributes
        // Window
    Ui::MainWindowShowRobot       userinterface;
    SoQtExaminerViewer           *viewer;
    SoSeparator                  *robotSep;
    SoSeparator                  *sceneSep;
    SoSeparator                  *objectsSep;
    CoinVistualizationPtr         visualization;

        // Environment
    iCubRobotPtr                  icub;
    ScenePtr                      scene;
    SceneObjectSetPtr             environment;
    SceneObjectSetPtr             obstacles;

        // Objects
    vector<ManipulationObjectPtr> objects;
    ManipulationObjectPtr         target_object;
    OrientedBoundingBoxPtr        oobb;

    vector<ObstaclePtr>           oobb_vertices;

        // Robot nodes
    vector<RobotNodePtr>          allrobotnodes;
    vector<RobotNodePtr>          current_robotnodes;
    vector<RobotNodeSetPtr>       robotnodesets;
    vector<EndEffectorPtr>        eefs;
    EndEffectorPtr                current_eef;
    RobotNodeSetPtr               current_robotnodeset;
    RobotNodePtr                  current_robotnode;
    bool                          hand_loaded;

        // Optimization
    uint                          prev_grasp;
    uint                          test_index  = 0;
    uint                          total_tests = 1;
    LearningQueueWrapper          best_results;

        // Sliders
    vector<int>                   sliders;
    uint                          slider_mode;

    uint                          movehand_mode;

        // Movement Sliders
    vector<int>                   sliders_movement;

        // Manipulative Joints
    vector<float>                 actuated_joints_max {  4.14,  2.33,  1.57,  3.14,  1.57,  3,14,  1.57,  3.14,  1.57};
    vector<float>                 actuated_joints_min { -1.37,  0.00,  0.00,  0.00,  0.00,  0.00,  0.00,  0.00,  0.00};
    Vector09dof                   actuated_joints;

        // Threads
    boost::thread                 processing_thread;
    boost::mutex                  mutex;
    bool                          thread_running;
    bool                          thread_unprocessed;


    // Methods
    void setupUI                (void);
    void loadScene              (void);
    void loadRobot              (void);
    void updateRobotInfo        (void);

    void rebuildVisualization   (void);

    void updateJointBox         (void);
    void updateMoveHandBox      (void);
    void updateRNSBox           (void);
    void updateSliderBox        (void);
    void updateGraspBox         (void);

    void convertSliders         (int index);

    void redraw                 (void);

    // Thread Functions
    void searchObjectTestThread (void);
    void optimizeGraspThread    (void);
    void closeGraspThread       (void);
    void graspMetricTestThread  (void);
};


#endif // _Window_h_
