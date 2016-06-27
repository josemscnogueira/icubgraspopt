/**************************************************************************************************
 *  File:    Window.cpp                                                                           *
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

/**************************************************************************************************
 *  Include Files                                                                                 *
 **************************************************************************************************/
#include <Window.hpp>

#include <tgpoptimizable.hpp>
#include <iCubOptimizable.hpp>
#include <TGPOptimization.hpp>
#include <LogManager.hpp>
#include <criteria/criteria_uei.hpp>

using bayesopt::TGPOptimization;
using bayesopt::Parameters;
using bayesopt::TgpParameters;
using bayesopt::iCubOptimizable;
using bayesopt::iCubOptParameters;


/**************************************************************************************************
 *  Used namespaces                                                                               *
 **************************************************************************************************/
using namespace VirtualRobot;
using std::string;
using std::cout;
using std::endl;


/**************************************************************************************************
 *  Procecure                                                                                     *
 *                                                                                                *
 *  Description: Constructor                                                                      *
 *  Class      : ShowWindow                                                                       *
 **************************************************************************************************/
ShowWindow::ShowWindow(Qt::WFlags flags)
: QMainWindow(NULL)
{

    VR_INFO << " start " << endl;

    // Thread variables
    thread_running     = false;
    thread_unprocessed = false;

    // Create SoSeparators
    robotSep   = new SoSeparator;
    sceneSep   = new SoSeparator;
    objectsSep = new SoSeparator;

    // Links SoSeps
    sceneSep -> ref();
    sceneSep -> addChild(robotSep);
    sceneSep -> addChild(objectsSep);

    // Setup User Interface
    setupUI();

    // Load Scene
    loadScene();

    // Load Robots
    loadRobot();

    viewer -> viewAll();

    hand_loaded = false;

    // Temp
    loadHand        (          );
    selectSliderMode(POWER_GRIP);
    selectMoveHand  (LEDO      );

    this -> test_index  = 0;
    this -> total_tests = 1;
    this -> use_tgp     = false;
    this -> log_mode    = LOG_LEARNING | MONTE_CARLO_OPT;

    best_results = LearningQueueWrapper(0);
}

/**************************************************************************************************
 *  Procecure                                                                                     *
 *                                                                                                *
 *  Description: Constructor                                                                      *
 *  Class      : ShowWindow                                                                       *
 **************************************************************************************************/
ShowWindow::ShowWindow(uint test_index, uint total_tests, Qt::WFlags flags)
: ShowWindow(flags)
{
    this -> test_index  = test_index;
    this -> total_tests = total_tests;
    this -> use_tgp     = use_tgp;
    this -> log_mode    = log_mode;

    selectSliderMode(icub_param.grasp);
    selectMoveHand  (icub_param.position);

    optimizeGrasp();
}

/**************************************************************************************************
 *  Procecure                                                                                     *
 *                                                                                                *
 *  Description: Constructor                                                                      *
 *  Class      : ShowWindow                                                                       *
 **************************************************************************************************/
ShowWindow::ShowWindow(uint test_index, uint total_tests, bool use_tgp, uint log_mode, Json::Value config, Qt::WFlags flags)
: ShowWindow(flags)
{
    this -> test_index  = test_index;
    this -> total_tests = total_tests;
    this -> use_tgp     = use_tgp;
    this -> log_mode    = log_mode;
    this -> configs     = config;

    if (!config["iCubOptParameters"].isNull())
    {
        this -> icub_param  = iCubOptParameters(config["iCubOptParameters"]);
    }

    selectSliderMode(icub_param.grasp);
    selectMoveHand  (icub_param.position);

    optimizeGrasp();
}


/**************************************************************************************************
 *  Procecure                                                                                     *
 *                                                                                                *
 *  Description: Destructor                                                                       *
 *  Class      : ShowWindow                                                                       *
 **************************************************************************************************/
ShowWindow::~ShowWindow()
{
    icub     -> reset();
    sceneSep -> unref();
}


/**************************************************************************************************
 *  Procecure                                                                                     *
 *                                                                                                *
 *  Description: resetSceneryAll                                                                  *
 *  Class      : ShowWindow                                                                       *
 **************************************************************************************************/
void ShowWindow::resetSceneryAll(void)
{
    if (!icub -> hasCurrentRobot()) return;


    if (!hand_loaded)
    {
        vector<float> jointvalues(allrobotnodes.size(), 0.0f);

        icub -> current_robot_part -> setJointValues(allrobotnodes, jointvalues);

        selectJoint(userinterface.comboBoxJoint -> currentIndex());
    }
    else
    {
        resetMoveHandSliders();
    }

}


/**************************************************************************************************
 *  Procecure                                                                                     *
 *                                                                                                *
 *  Description: setupUI                                                                      *
 *  Class      : ShowWindow                                                                       *
 **************************************************************************************************/
void ShowWindow::setupUI(void)
{
    userinterface.setupUi(this);

    viewer = new SoQtExaminerViewer(userinterface.frameViewer,"",TRUE,SoQtExaminerViewer::BUILD_POPUP);

    // Setup
    viewer -> setBackgroundColor   (SbColor(1.0f, 1.0f, 1.0f));
    viewer -> setAccumulationBuffer(true);
    viewer -> setAntialiasing      (true, 4);
    viewer -> setGLRenderAction    (new SoLineHighlightRenderAction);
    viewer -> setTransparencyType  (SoGLRenderAction::BLEND);
    viewer -> setFeedbackVisibility(true);
    viewer -> setSceneGraph        (sceneSep);
    viewer -> viewAll();

    // Initialize sliders
    sliders         .resize(9, 0);
    sliders_movement.resize(6,0);

    // Initialize Actuated Joints
    actuated_joints.fill(0.0f);

    // Initialize Slider Mode
    selectSliderMode(JOINTS_MODE);
    selectMoveHand  (ESPH       );

    // Connect UI to Functions
        // Push Buttons
    connect(userinterface.pushButtonReset     , SIGNAL(clicked())        , this, SLOT(resetSceneryAll    (void)));
    connect(userinterface.pushButtonLoadHand  , SIGNAL(clicked())        , this, SLOT(loadHand           (void)));
    connect(userinterface.pushButtonGraspReset, SIGNAL(clicked())        , this, SLOT(resetGraspSliders  (void)));
    connect(userinterface.pushButtonCloseGrasp, SIGNAL(clicked())        , this, SLOT(closeGrasp         (void)));
    connect(userinterface.pushButtonMoveObject, SIGNAL(clicked())        , this, SLOT(searchObjectTest   (void)));
    connect(userinterface.pushButtonGraspOpt  , SIGNAL(clicked())        , this, SLOT(optimizeGrasp      (void)));
    connect(userinterface.pushButtonMetric    , SIGNAL(clicked())        , this, SLOT(graspMetricTest    (void)));

    // Select Node and Joints
    connect(userinterface.comboBoxRobotNodeSet, SIGNAL(activated(int))   , this, SLOT(selectRNS          (int )));
    connect(userinterface.comboBoxJoint       , SIGNAL(activated(int))   , this, SLOT(selectJoint        (int )));
        // Joint Slider
    connect(userinterface.horizontalSliderPos , SIGNAL(valueChanged(int)), this, SLOT(jointValueChanged  (int )));
        // Grasp Slider
    connect(userinterface.comboBoxGrasp       , SIGNAL(activated(int))   , this, SLOT(selectSliderMode   (int )));
        // Best grasps from optimization
    connect(userinterface.comboBoxGraspOpt    , SIGNAL(activated(int))   , this, SLOT(selectGraspOpt     (int )));

    connect(userinterface.graspSlider1        , SIGNAL(valueChanged(int)), this, SLOT(handJointSlider1   (int )));
    connect(userinterface.graspSlider2        , SIGNAL(valueChanged(int)), this, SLOT(handJointSlider2   (int )));
    connect(userinterface.graspSlider3        , SIGNAL(valueChanged(int)), this, SLOT(handJointSlider3   (int )));
    connect(userinterface.graspSlider4        , SIGNAL(valueChanged(int)), this, SLOT(handJointSlider4   (int )));
    connect(userinterface.graspSlider5        , SIGNAL(valueChanged(int)), this, SLOT(handJointSlider5   (int )));
    connect(userinterface.graspSlider6        , SIGNAL(valueChanged(int)), this, SLOT(handJointSlider6   (int )));
    connect(userinterface.graspSlider7        , SIGNAL(valueChanged(int)), this, SLOT(handJointSlider7   (int )));
    connect(userinterface.graspSlider8        , SIGNAL(valueChanged(int)), this, SLOT(handJointSlider8   (int )));
    connect(userinterface.graspSlider9        , SIGNAL(valueChanged(int)), this, SLOT(handJointSlider9   (int )));

    // Hand Movement Sliders
    connect(userinterface.comboBoxMoveHand    , SIGNAL(activated(int))   , this, SLOT(selectMoveHand     (int )));
    connect(userinterface.moveHandSlider1     , SIGNAL(valueChanged(int)), this, SLOT(moveHandSlider1    (int )));
    connect(userinterface.moveHandSlider2     , SIGNAL(valueChanged(int)), this, SLOT(moveHandSlider2    (int )));
    connect(userinterface.moveHandSlider3     , SIGNAL(valueChanged(int)), this, SLOT(moveHandSlider3    (int )));
    connect(userinterface.moveHandSlider4     , SIGNAL(valueChanged(int)), this, SLOT(moveHandSlider4    (int )));
    connect(userinterface.moveHandSlider5     , SIGNAL(valueChanged(int)), this, SLOT(moveHandSlider5    (int )));
    connect(userinterface.moveHandSlider6     , SIGNAL(valueChanged(int)), this, SLOT(moveHandSlider6    (int )));
}


/**************************************************************************************************
 *  Procecure                                                                                     *
 *                                                                                                *
 *  Description: loadScene                                                                        *
 *  Class      : ShowWindow                                                                       *
 **************************************************************************************************/
void ShowWindow::loadScene(void)
{
    // Load Scene
    if(!scene) scene = SceneIO::loadScene("data/iCub_scene.xml");

    environment        = scene  -> getSceneObjectSet    ("Environment");
    objects.push_back(   scene  -> getManipulationObject(icub_param.object.c_str()));
    objects.push_back(   scene  -> getManipulationObject("Table"));
    objects.push_back(   scene  -> getManipulationObject("Mug"));


    target_object = objects[0];
    oobb.reset(new OrientedBoundingBox(target_object));
    oobb_vertices = oobb -> createVerticesObstacles();

    cout << endl << "Number of facets of sphere: " << objects[0] -> getNumFaces();

    cout << endl << "Pose: "             << endl   << objects[0] -> getGlobalPose();

    cout << endl << "Inertia Matrix: "   << endl   << objects[2] -> getInertiaMatrix();

    obstacles.reset(new SceneObjectSet());

    obstacles -> addSceneObject(objects[0]);
}


/**************************************************************************************************
 *  Procecure                                                                                     *
 *                                                                                                *
 *  Description: loadRobot                                                                        *
 *  Class      : ShowWindow                                                                       *
 **************************************************************************************************/
void ShowWindow::loadRobot(void)
{
    robotSep -> removeAllChildren();

    // Load iCub
    if (!icub)
    {
        if (scene) icub.reset(new  iCubRobot(scene, "iCub"));
        else       icub.reset(new  iCubRobot()             );
    }

    // Check if iCub is Loaded
    if (!icub -> hasCurrentRobot()) exit(-1);

    // Reset current pointers
    current_eef         .reset();
    current_robotnode   .reset();
    current_robotnodeset.reset();

    updateRobotInfo();
}


/**************************************************************************************************
 *  Procecure                                                                                     *
 *                                                                                                *
 *  Description: updateRobotInfo                                                                  *
 *  Class      : ShowWindow                                                                       *
 **************************************************************************************************/
void ShowWindow::updateRobotInfo(void)
{
    // If there is no robot, return
    if (!(icub -> current_robot_part)) return;

    // Get Robot Nodes
    icub -> current_robot_part -> getRobotNodes   (allrobotnodes);
    icub -> current_robot_part -> getRobotNodeSets(robotnodesets);
    icub -> current_robot_part -> getEndEffectors (eefs);

    // updateEEFBox();
    updateSliderBox();
    updateMoveHandBox();
    updateRNSBox();
    selectRNS(0);

    if (allrobotnodes.size() == 0)  selectJoint(-1);
    else                            selectJoint( 0);

    // Build Visualization
    rebuildVisualization();

    selectJoint(userinterface.comboBoxJoint -> currentIndex());

    viewer -> viewAll();
}


/**************************************************************************************************
 *  Procecure                                                                                     *
 *                                                                                                *
 *  Description: rebuildVisualization                                                             *
 *  Class      : ShowWindow                                                                       *
 **************************************************************************************************/
void ShowWindow::rebuildVisualization(void)
{
    // If there is no scene, return
    if (!scene) return;

    VisualizationNodePtr                                visual_object;
    vector<VisualizationNodePtr>                        visual_vector;
    boost::shared_ptr<VirtualRobot::CoinVisualization>  visualization_objects;

    // Load Environment
    objectsSep -> removeAllChildren();

    for(uint index = 0; index < objects.size(); index += 1)
    {
        visual_object = objects[index] -> getVisualization();

        visual_vector.push_back(visual_object);
    }

    for(uint index = 0; index < oobb_vertices.size(); index += 1)
    {
        visual_object = oobb_vertices[index] -> getVisualization();

        visual_vector.push_back(visual_object);
    }

    visualization_objects.reset(new VirtualRobot::CoinVisualization(visual_vector));

    // Build Object Visualization
    objectsSep -> addChild(visualization_objects -> getCoinVisualization());

    // If there is no robot, return
    if (!icub)                         return;
    if (!(icub -> current_robot_part)) return;

    robotSep -> removeAllChildren();

    // Get iCub visual model
    if (false) // we can check later if we want the collision model
        visualization = icub -> current_robot_part -> getVisualization<CoinVisualization>(SceneObject::Collision);
    else
        visualization = icub -> current_robot_part -> getVisualization<CoinVisualization>(SceneObject::Full);

    SoNode *visualization_node = NULL;

    if (visualization)
        visualization_node = visualization -> getCoinVisualization();

    if (visualization_node)
        robotSep -> addChild(visualization_node);

    redraw();
}


/**************************************************************************************************
 *  Procecure                                                                                     *
 *                                                                                                *
 *  Description: searchObjectTest                                                                 *
 *  Class      : ShowWindow                                                                       *
 **************************************************************************************************/
void ShowWindow::searchObjectTest(void)
{
    // Check if thread is already running
    if (thread_running || thread_unprocessed) return;

    mutex.lock();
    {
        thread_running    = true;
    }
    mutex.unlock();

    processing_thread = boost::thread(&ShowWindow::searchObjectTestThread, this);
}


/**************************************************************************************************
 *  Procecure                                                                                     *
 *                                                                                                *
 *  Description: searchObjectTestThread                                                           *
 *  Class      : ShowWindow                                                                       *
 **************************************************************************************************/
void ShowWindow::searchObjectTestThread(void)
{
    boost::lock_guard<boost::mutex> lock(mutex);

    uint             number_of_contacts;

    const uint       grid_elem         =  20;
    const float      grid_step         =   6.0;
    const float      grid_size         = grid_elem * grid_step / 2.0f;

    uint             index1, index2;
    ofstream         output_file;
    float            transformation[6];
    Eigen::Matrix4f  pose;

    // Prepare object for sweeping
    transformation[0] =   grid_size - grid_step;
    transformation[1] =  -grid_size - grid_step;
    transformation[2] =  0.0;
    transformation[3] =  0.0;
    transformation[4] =  0.0;
    transformation[5] =  0.0;

    MathTools::posrpy2eigen4f(transformation, pose);

    pose   = target_object -> getGlobalPose() * pose;
    target_object          -> setGlobalPose(pose);

    output_file.open("logs/testobjectgrasp.log");

    output_file << "     x     |     y     |     z     |   forceC  |  quality  |   volume  |" << "\n";
    output_file << "-----------|-----------|-----------|-----------|-----------|-----------|" << "\n";

    for (index1 = 0; index1 < grid_elem; index1 +=1)
    {
        transformation[0] = -2 * grid_size;
        transformation[1] =      grid_step;

        MathTools::posrpy2eigen4f(transformation, pose);

        pose = target_object -> getGlobalPose() * pose;
        target_object        -> setGlobalPose(pose);

        transformation[0] =  grid_step;
        transformation[1] =  0;

        for (index2 = 0; index2 < grid_elem; index2 +=1)
        {
            MathTools::posrpy2eigen4f(transformation, pose);

            pose = target_object -> getGlobalPose() * pose;
            target_object        -> setGlobalPose(pose);

            cout << "Closing Hand...   ";

            GraspQualityMeasureWrenchSpacePtr grasp = icub -> closeHand(actuated_joints, slider_mode - 1, obstacles, number_of_contacts);

            cout << "Hand Closed for: index1 =" << index1 << ", index2 =" << index2 << " .....   ";

            icub -> openHand (actuated_joints, slider_mode - 1);

            cout << "Hand Opened" << endl;

            output_file << setw(11) << pose(0,3)                        << "|";
            output_file << setw(11) << pose(1,3)                        << "|";
            output_file << setw(11) << pose(2,3)                        << "|";

            if(number_of_contacts > 1)
            {
                output_file << setw(11) << grasp -> isGraspForceClosure()   << "|";
                output_file << setw(11) << grasp -> getGraspQuality()       << "|";
                output_file << setw(11) << grasp -> getVolumeGraspMeasure() << "|\nget";
            }
            else
            {
                output_file << setw(11) << 0                                << "|";
                output_file << setw(11) << 0                                << "|";
                output_file << setw(11) << 0                                << "|\n";
            }
        }
    }

    output_file.close();

    thread_unprocessed = true;
    thread_unprocessed = false;
    thread_running     = false;
}



void ShowWindow::optimizeGrasp(void)
{
    // cout << endl <<"Clicked Optimization Button" << endl;

    // Check if thread is already running
    if (thread_running || thread_unprocessed) return;

    mutex.lock();
    {
        thread_running = true;
    }
    mutex.unlock();

    processing_thread = boost::thread(&ShowWindow::optimizeGraspThread, this);
}


void ShowWindow::optimizeGraspThread(void)
{
    iCubOptimizable*  func = new iCubOptimizable(icub, target_object, icub_param);

    vectord           bestpoint, upper, lower;

    // Fill optimization parameters
    func -> getOptParams(tgp_param , opt_param);

    // Load Parameters from json config file
    if (!configs["Parameters"].isNull())
    {
        opt_param.loadJson(configs["Parameters"]);
        std::cout << std::endl << "Loaded Parameters";
    }

    if (!configs["TgpParameters"].isNull())
    {
        tgp_param.loadJson(configs["TgpParameters"]);
        std::cout << std::endl << "Loaded TgpParameters";
    }

    // See if Optimization should use TGP or GP
    if (!configs["useTGP"].isNull())
    {
        use_tgp = configs["useTGP"].asBool();
    }

    // Set logging mode
    if (!configs["logMode"].isNull())
    {
        log_mode = LogManager::stringToLogmode(configs["logMode"].asString());
    }

    // Get Exploration Bounding Box
    func -> getBoundingBox(lower, upper);

    if ( (slider_mode   != JOINTS_MODE) &&
         (movehand_mode != ESPH       )   )
    {
        // Choose seed
        opt_param.random_seed = ( tgp_param.dimensions * opt_param.n_init_samples * test_index ) + 1;

        // Start Optimization
        TGPOptimization* tgp_opt  = new TGPOptimization(tgp_param, opt_param, (*func), log_mode, use_tgp, test_index);
                         tgp_opt ->     setBoundingBox (lower, upper);
                         tgp_opt ->     optimize       (bestpoint);

        if (test_index == (total_tests - 1))
        {
            tgp_opt -> printLogFooter();
            tgp_opt -> copyJsonConfig();
        }

        best_results = tgp_opt -> getBestResults();

        updateGraspBox();

        delete tgp_opt;
        delete func;

        exit(0);
    }
    else
    {
        cout << endl << endl << "Not valid Grasp mode or Hand Position mode." << endl;
    }

    delete func;

    thread_running = false;
}


void ShowWindow::graspMetricTest(void)
{
    cout << endl << "Clicked Metric Test Button" << endl;

    // Check if thread is already running
    if (thread_running || thread_unprocessed) return;

    mutex.lock();
    {
        thread_running = true;
    }
    mutex.unlock();

    processing_thread = boost::thread(&ShowWindow::graspMetricTestThread, this);
}


void ShowWindow::graspMetricTestThread(void)
{
    Parameters        opt_param;
    TgpParameters     tgp_param;

    iCubOptParameters icub_param;
                      icub_param.grasp            = slider_mode - 1;
                      icub_param.position         = movehand_mode;
                      icub_param.object           = target_object -> getName();
                      icub_param.default_query    = vectord(7);
                      icub_param.default_query[0] = 140;
                      icub_param.default_query[1] = 460;
                      icub_param.default_query[2] = 2.432;
                      icub_param.default_query[3] = 1.083;
                      icub_param.default_query[4] = 359.388;
                      icub_param.default_query[5] = 103.021;
                      icub_param.default_query[6] = -3.139;

                      icub_param.active_variables.clear();
                    //   icub_param.active_variables.push_back(TRANS_X);
                    //   icub_param.active_variables.push_back(TRANS_Y);

    iCubOptimizable*  func = new iCubOptimizable(icub, target_object, icub_param);

    // Set parameters aux
    icub_param.n_grasp_trials = 1;

    // Fill optimization parameters
    func -> getOptParams(tgp_param , opt_param);

    // Get Uncertainty Matrix for Unscented Expected Improvement
    matrixd px = func -> getUncertaintyMatrix(MC_STD); // Must be Normalized for montecarlo sampling
    bayesopt::UnscentedExpectedImprovement::convertMatrixToParams(opt_param, px);


    func -> evaluate(icub_param.default_query);

    // LogManager* logmanager = new LogManager(*func, tgp_param);
    //             logmanager -> printFuncProfile(1);
    //
    //
    //
    // delete logmanager;
    delete func;

    thread_running = false;
}


/**************************************************************************************************
 *  Procecure                                                                                     *
 *                                                                                                *
 *  Description: selectRNS                                                                        *
 *  Class      : ShowWindow                                                                       *
 **************************************************************************************************/
void ShowWindow::selectRNS(int index)
{
    // Reset Current Node Set
    current_robotnodeset.reset();

    // Select Node Set
    if (index <= 0)
    {
        current_robotnodes = allrobotnodes;
    }
    else
    {
        index -= 1;

        if (index >= (int)robotnodesets.size()) return;

        current_robotnodeset = robotnodesets[index];
        current_robotnodes   = current_robotnodeset -> getAllRobotNodes();
    }

    // Update Visual Box
    updateJointBox();

    // Select First Joint of NodeSet
    selectJoint(0);
}


/**************************************************************************************************
 *  Procecure                                                                                     *
 *                                                                                                *
 *  Description: selectSliderMode                                                                 *
 *  Class      : ShowWindow                                                                       *
 **************************************************************************************************/
void ShowWindow::selectSliderMode(int mode)
{
    // Update Slider mode
    slider_mode = mode;

    // Reset Sliders to the middle position
    resetGraspSliders();
}


/**************************************************************************************************
 *  Procecure                                                                                     *
 *                                                                                                *
 *  Description: selectSliderMode                                                                 *
 *  Class      : ShowWindow                                                                       *
 **************************************************************************************************/
void ShowWindow::selectMoveHand(int mode)
{
    // Update Slider mode
    movehand_mode = mode;

    // Reset Sliders to the middle position
    resetMoveHandSliders();
}


/**************************************************************************************************
 *  Procecure                                                                                     *
 *                                                                                                *
 *  Description: resetGraspSliders                                                                *
 *  Class      : ShowWindow                                                                       *
 **************************************************************************************************/
void ShowWindow::resetGraspSliders(void)
{
    switch (slider_mode)
    {
        case TRYPOD             :
        case PALMAR_PINCH       :
        case LATERAL            :
        case WRITING_TRIPOD     :
        case PARALLEL_EXTENSION :
        case ADDUCTION_GRIP     :
        case TIP_PINCH          :
        case LATERAL_TRIPOD     :
        case POWER_GRIP         :
        {
            userinterface.graspSlider1 -> setValue  (500);
            userinterface.graspSlider2 -> setValue  (500);
            userinterface.graspSlider3 -> setValue  (500);
            userinterface.graspSlider4 -> setValue  (500);
            userinterface.graspSlider5 -> setValue  (500);
            userinterface.graspSlider6 -> setValue  (500);
            userinterface.graspSlider7 -> setValue  (500);
            userinterface.graspSlider8 -> setValue  (500);
            userinterface.graspSlider9 -> setValue  (500);

            sliders.assign(sliders.size(), 500);

            convertSliders(0);
            convertSliders(1);
            convertSliders(2);
            convertSliders(3);
            convertSliders(4);
            convertSliders(5);
            convertSliders(6);
            convertSliders(7);
            convertSliders(8);

            icub -> controlHand (actuated_joints, slider_mode);

            break;
        }
    }

    rebuildVisualization();
}


/**************************************************************************************************
 *  Procecure                                                                                     *
 *                                                                                                *
 *  Description: resetMoveHandSliders                                                             *
 *  Class      : ShowWindow                                                                       *
 **************************************************************************************************/
void ShowWindow::resetMoveHandSliders(void)
{
    switch (movehand_mode)
    {
        case ESPH :
        {

            sliders_movement[0] =   0;
            sliders_movement[1] =   0;
            sliders_movement[2] =   0;
            sliders_movement[3] =   0;
            sliders_movement[4] = 500;
            sliders_movement[5] = 500;

            userinterface.moveHandSlider1 -> setValue  (  0);
            userinterface.moveHandSlider2 -> setValue  (  0);
            userinterface.moveHandSlider3 -> setValue  (  0);
            userinterface.moveHandSlider4 -> setValue  (  0);
            userinterface.moveHandSlider5 -> setValue  (500);
            userinterface.moveHandSlider6 -> setValue  (500);

            if (hand_loaded && icub) moveHandSlider6(500);

            break;
        }

        default :
        {
            sliders_movement[0] = 500;
            sliders_movement[1] = 500;
            sliders_movement[2] =   0;
            sliders_movement[3] = 500;
            sliders_movement[4] = 500;
            sliders_movement[5] = 500;

            userinterface.moveHandSlider1 -> setValue  (500);
            userinterface.moveHandSlider2 -> setValue  (500);
            userinterface.moveHandSlider3 -> setValue  (  0);
            userinterface.moveHandSlider4 -> setValue  (500);
            userinterface.moveHandSlider5 -> setValue  (500);
            userinterface.moveHandSlider6 -> setValue  (500);

            if (hand_loaded) moveHandSlider6(500);

            break;
        }
    }
}


/**************************************************************************************************
 *  Procecure                                                                                     *
 *                                                                                                *
 *  Description: closeGrasp                                                                       *
 *  Class      : ShowWindow                                                                       *
 **************************************************************************************************/
void ShowWindow::closeGrasp(void)
{
    switch (slider_mode)
    {
        case TRYPOD             :
        case PALMAR_PINCH       :
        case LATERAL            :
        case WRITING_TRIPOD     :
        case PARALLEL_EXTENSION :
        case ADDUCTION_GRIP     :
        case TIP_PINCH          :
        case LATERAL_TRIPOD     :
        case POWER_GRIP         :
        {
            if( icub -> getHandState() == OPENED)
            {
                // Check if thread is already running
                if (thread_running || thread_unprocessed) return;

                mutex.lock();
                {
                    thread_running    = true;
                }
                mutex.unlock();

                processing_thread = boost::thread(&ShowWindow::closeGraspThread, this);
            }
            else
            {
                icub -> openHand(actuated_joints, slider_mode - 1);
            }
        }
    }

    rebuildVisualization();
}


/**************************************************************************************************
 *  Procecure                                                                                     *
 *                                                                                                *
 *  Description: closeGraspThread                                                                 *
 *  Class      : ShowWindow                                                                       *
 **************************************************************************************************/
void ShowWindow::closeGraspThread(void)
{
    boost::lock_guard<boost::mutex> lock(mutex);

    uint number_of_contacts;

    GraspQualityMeasureWrenchSpacePtr grasp = icub -> closeHand(actuated_joints, slider_mode - 1, obstacles, number_of_contacts);

    std::cout << std::endl << "force closure = " << grasp -> isGraspForceClosure();
    std::cout << std::endl << "grasp quality = " << grasp -> getGraspQuality() << std::endl;

    std::cout << std::endl << "slidermovement[0] = " << sliders_movement[0] << std::endl;
    std::cout << std::endl << "slidermovement[1] = " << sliders_movement[1] << std::endl;
    std::cout << std::endl << "slidermovement[2] = " << sliders_movement[2] << std::endl;
    std::cout << std::endl << "slidermovement[3] = " << sliders_movement[3] << std::endl;
    std::cout << std::endl << "slidermovement[4] = " << sliders_movement[4] << std::endl;
    std::cout << std::endl << "slidermovement[5] = " << sliders_movement[6] << std::endl;

    thread_running     = false;
    thread_unprocessed = true;
    thread_unprocessed = false;
}


/**************************************************************************************************
 *  Procecure                                                                                     *
 *                                                                                                *
 *  Description: selectJoint                                                                      *
 *  Class      : ShowWindow                                                                       *
 **************************************************************************************************/
void ShowWindow::selectJoint(int index)
{
    // Reset Currently selected robot node
    if (current_robotnode)
        current_robotnode -> showBoundingBox(false);

    current_robotnode.reset();

    // Select Robot Node
    if ( (index < 0) || (index >= (int)current_robotnodes.size()) ) return;

    current_robotnode = current_robotnodes[index];
    current_robotnode -> showBoundingBox(true, true);
    //current_robotnode -> print();

    float max_joint_value = current_robotnode -> getJointLimitHi();
    float min_joint_value = current_robotnode -> getJointLimitLo();
    float cur_joint_value = current_robotnode -> getJointValue();

    int   slider_pos      = (int) ( (cur_joint_value - min_joint_value) / (max_joint_value - min_joint_value) * 1000.0f);

    // Update UI
    userinterface.labelMaxPos         -> setText(QString::number(max_joint_value));
    userinterface.labelMinPos         -> setText(QString::number(min_joint_value));
    userinterface.lcdNumberJointValue -> display(        (double)cur_joint_value);

    // Configure Joint Slider
    if ( fabs(max_joint_value - min_joint_value) && (    current_robotnode -> isTranslationalJoint()
                                                      || current_robotnode -> isRotationalJoint()   ) )
    {
        userinterface.horizontalSliderPos -> setEnabled(true);
        userinterface.horizontalSliderPos -> setValue  (slider_pos);
    }
    else
    {
        userinterface.horizontalSliderPos -> setEnabled(false);
        userinterface.horizontalSliderPos -> setValue  (500);
    }

    if (visualization)
    {
        icub -> current_robot_part -> highlight(visualization, false);
        current_robotnode          -> highlight(visualization, true);
    }

    rebuildVisualization();
}


/**************************************************************************************************
 *  Procecure                                                                                     *
 *                                                                                                *
 *  Description: jointValueChanged                                                                *
 *  Class      : ShowWindow                                                                       *
 **************************************************************************************************/
void ShowWindow::jointValueChanged(int slide_pos)
{
    // Get current joint index
    int index = userinterface.comboBoxJoint -> currentIndex();

    // Check if out of bounds
    if ( (index < 0) || (index >= (int)current_robotnodes.size()) ) return;

    // Calculate new pos
    float f_pos  = current_robotnodes[index] -> getJointLimitLo();
          f_pos += ((float)slide_pos / 1000.0f) * (current_robotnodes[index] -> getJointLimitHi() - current_robotnodes[index] -> getJointLimitLo());

    // Update Robot
    icub -> current_robot_part -> setJointValue(current_robotnodes[index], f_pos);

    // Update UI
    userinterface.lcdNumberJointValue -> display((double)f_pos);

    // rebuildVisualization();
}


/**************************************************************************************************
 *  Procecure                                                                                     *
 *                                                                                                *
 *  Description: convertSliders                                                                   *
 *  Class      : ShowWindow                                                                       *
 **************************************************************************************************/
void ShowWindow::convertSliders(int index)
{
    switch (slider_mode)
    {
        case JOINTS_MODE :
        {
            actuated_joints[index]  = actuated_joints_min[index];
            actuated_joints[index] += ((float)sliders[index] / 1000.0f) * (actuated_joints_max[index] - actuated_joints_min[index]);
            break;
        }

        case TRYPOD             :
        case PALMAR_PINCH       :
        case LATERAL            :
        case WRITING_TRIPOD     :
        case PARALLEL_EXTENSION :
        case ADDUCTION_GRIP     :
        case TIP_PINCH          :
        case LATERAL_TRIPOD     :
        case POWER_GRIP         :
        {
            actuated_joints[index]  = - M_PI;
            actuated_joints[index] += ((float)sliders[index] / 1000.0f) * (M_PI * 2);
            break;
        }
    }

}


/**************************************************************************************************
 *  Inline Functions                                                                              *
 **************************************************************************************************/
void ShowWindow::handJointSlider1(int pos) {sliders[0] = pos; convertSliders(0); icub -> controlHand (actuated_joints, slider_mode); rebuildVisualization();}
void ShowWindow::handJointSlider2(int pos) {sliders[1] = pos; convertSliders(1); icub -> controlHand (actuated_joints, slider_mode); rebuildVisualization();}
void ShowWindow::handJointSlider3(int pos) {sliders[2] = pos; convertSliders(2); icub -> controlHand (actuated_joints, slider_mode); rebuildVisualization();}
void ShowWindow::handJointSlider4(int pos) {sliders[3] = pos; convertSliders(3); icub -> controlHand (actuated_joints, slider_mode); rebuildVisualization();}
void ShowWindow::handJointSlider5(int pos) {sliders[4] = pos; convertSliders(4); icub -> controlHand (actuated_joints, slider_mode); rebuildVisualization();}
void ShowWindow::handJointSlider6(int pos) {sliders[5] = pos; convertSliders(5); icub -> controlHand (actuated_joints, slider_mode); rebuildVisualization();}
void ShowWindow::handJointSlider7(int pos) {sliders[6] = pos; convertSliders(6); icub -> controlHand (actuated_joints, slider_mode); rebuildVisualization();}
void ShowWindow::handJointSlider8(int pos) {sliders[7] = pos; convertSliders(7); icub -> controlHand (actuated_joints, slider_mode); rebuildVisualization();}
void ShowWindow::handJointSlider9(int pos) {sliders[8] = pos; convertSliders(8); icub -> controlHand (actuated_joints, slider_mode); rebuildVisualization();}


void ShowWindow::moveHandSlider1(int pos) {sliders_movement[0] = pos; icub -> approachHand(target_object, sliders_movement, movehand_mode); rebuildVisualization();}
void ShowWindow::moveHandSlider2(int pos) {sliders_movement[1] = pos; icub -> approachHand(target_object, sliders_movement, movehand_mode); rebuildVisualization();}
void ShowWindow::moveHandSlider3(int pos) {sliders_movement[2] = pos; icub -> approachHand(target_object, sliders_movement, movehand_mode); rebuildVisualization();}
void ShowWindow::moveHandSlider4(int pos) {sliders_movement[3] = pos; icub -> approachHand(target_object, sliders_movement, movehand_mode); rebuildVisualization();}
void ShowWindow::moveHandSlider5(int pos) {sliders_movement[4] = pos; icub -> approachHand(target_object, sliders_movement, movehand_mode); rebuildVisualization();}
void ShowWindow::moveHandSlider6(int pos) {sliders_movement[5] = pos; icub -> approachHand(target_object, sliders_movement, movehand_mode); rebuildVisualization();}


/**************************************************************************************************
 *  Procecure                                                                                     *
 *                                                                                                *
 *  Description: selectGraspOpt                                                                   *
 *  Class      : ShowWindow                                                                       *
 **************************************************************************************************/
void ShowWindow::selectGraspOpt(int pos)
{
    if (pos < (int)best_results.size())
    {
        iCubOptParameters icub_param;
                          icub_param.grasp    = slider_mode - 1;
                          icub_param.position = movehand_mode;

        iCubOptimizable*  func = new iCubOptimizable(icub, target_object, icub_param);

        cout << endl << pos << endl;

        func -> showBestGrasps(pos, best_results);

        delete func;
    }
}


/**************************************************************************************************
 *  Procecure                                                                                     *
 *                                                                                                *
 *  Description: updateGraspBox                                                                   *
 *  Class      : ShowWindow                                                                       *
 **************************************************************************************************/
void ShowWindow::updateGraspBox(void)
{
    // Clear all available robot node sets
    userinterface.comboBoxGraspOpt -> clear();

    if (best_results.size() == 0) return;

    // Add the all nodes option
    for (uint index = 0; index < best_results.size(); index += 1)
    {
        std::ostringstream ss;
        std::string        s;

        ss << best_results[index].quality;
        s  =  ss.str();

        userinterface.comboBoxGraspOpt -> addItem( QString( s.c_str() ) );
    }
}


/**************************************************************************************************
 *  Procecure                                                                                     *
 *                                                                                                *
 *  Description: updateJointBox                                                                   *
 *  Class      : ShowWindow                                                                       *
 **************************************************************************************************/
void ShowWindow::updateJointBox(void)
{
    // Clear all available joints
    userinterface.comboBoxJoint -> clear();

    // Update joints for the new robot node
    for (unsigned int index = 0; index < current_robotnodes.size(); index++)
    {
        userinterface.comboBoxJoint -> addItem( QString( current_robotnodes[index] -> getName().c_str() ) );
    }
}


/**************************************************************************************************
 *  Procecure                                                                                     *
 *                                                                                                *
 *  Description: updateRNSBox                                                                     *
 *  Class      : ShowWindow                                                                       *
 **************************************************************************************************/
void ShowWindow::updateRNSBox(void)
{
    // Clear all available robot node sets
    userinterface.comboBoxRobotNodeSet -> clear();

    // Add the all nodes option
    userinterface.comboBoxRobotNodeSet -> addItem( QString("<All>") );

    // Add all other nodesets
    for (unsigned int index = 0; index < robotnodesets.size(); index++)
    {
        userinterface.comboBoxRobotNodeSet -> addItem( QString( robotnodesets[index] -> getName().c_str() ) );
    }
}


/**************************************************************************************************
 *  Procecure                                                                                     *
 *                                                                                                *
 *  Description: updateMoveHandBox                                                                *
 *  Class      : ShowWindow                                                                       *
 **************************************************************************************************/
void ShowWindow::updateMoveHandBox()
{
    // Clear all content
    userinterface.comboBoxMoveHand -> clear();

    // Add nodes
    userinterface.comboBoxMoveHand -> addItem( QString("RI-FR") );
    userinterface.comboBoxMoveHand -> addItem( QString("RI-UP") );
    userinterface.comboBoxMoveHand -> addItem( QString("RI-BA") );
    userinterface.comboBoxMoveHand -> addItem( QString("RI-DO") );
    userinterface.comboBoxMoveHand -> addItem( QString("FR-LE") );
    userinterface.comboBoxMoveHand -> addItem( QString("FR-UP") );
    userinterface.comboBoxMoveHand -> addItem( QString("FR-RI") );
    userinterface.comboBoxMoveHand -> addItem( QString("FR-DO") );
    userinterface.comboBoxMoveHand -> addItem( QString("LE-BA") );
    userinterface.comboBoxMoveHand -> addItem( QString("LE-UP") );
    userinterface.comboBoxMoveHand -> addItem( QString("LE-FR") );
    userinterface.comboBoxMoveHand -> addItem( QString("LE-DO") );
    userinterface.comboBoxMoveHand -> addItem( QString("BA-RI") );
    userinterface.comboBoxMoveHand -> addItem( QString("BA-UP") );
    userinterface.comboBoxMoveHand -> addItem( QString("BA-LE") );
    userinterface.comboBoxMoveHand -> addItem( QString("BA-DO") );
    userinterface.comboBoxMoveHand -> addItem( QString("TO-FR") );
    userinterface.comboBoxMoveHand -> addItem( QString("TO-LE") );
    userinterface.comboBoxMoveHand -> addItem( QString("TO-BA") );
    userinterface.comboBoxMoveHand -> addItem( QString("TO-RI") );
    userinterface.comboBoxMoveHand -> addItem( QString("BO-FR") );
    userinterface.comboBoxMoveHand -> addItem( QString("BO-RI") );
    userinterface.comboBoxMoveHand -> addItem( QString("BO-BA") );
    userinterface.comboBoxMoveHand -> addItem( QString("BO-LE") );
    userinterface.comboBoxMoveHand -> addItem( QString("ES-PH") );
}


/**************************************************************************************************
 *  Procecure                                                                                     *
 *                                                                                                *
 *  Description: updateSliderBox                                                                  *
 *  Class      : ShowWindow                                                                       *
 **************************************************************************************************/
void ShowWindow::updateSliderBox()
{
    // Clear the ComboBox
    userinterface.comboBoxGrasp -> clear();

    // Add items
    userinterface.comboBoxGrasp -> addItem( QString("Actuated Joints"    ) );
    userinterface.comboBoxGrasp -> addItem( QString("Tripod"             ) );
    userinterface.comboBoxGrasp -> addItem( QString("Palmar Pinch"       ) );
    userinterface.comboBoxGrasp -> addItem( QString("Lateral"            ) );
    userinterface.comboBoxGrasp -> addItem( QString("Writing Tripod"     ) );
    userinterface.comboBoxGrasp -> addItem( QString("Parallel Extension" ) );
    userinterface.comboBoxGrasp -> addItem( QString("Adduction Grip"     ) );
    userinterface.comboBoxGrasp -> addItem( QString("Tip Pinch"          ) );
    userinterface.comboBoxGrasp -> addItem( QString("Lateral Tripod"     ) );
    userinterface.comboBoxGrasp -> addItem( QString("PowerGrip"          ) );
}


/**************************************************************************************************
 *  Procecure                                                                                     *
 *                                                                                                *
 *  Description: main                                                                             *
 *  Class      : ShowWindow                                                                       *
 **************************************************************************************************/
int ShowWindow::main()
{
    SoQt::show(this);
    SoQt::mainLoop();

    return 0;
}


/**************************************************************************************************
 *  Procecure                                                                                     *
 *                                                                                                *
 *  Description: quit                                                                             *
 *  Class      : ShowWindow                                                                       *
 **************************************************************************************************/
void ShowWindow::quit()
{
    cout << "ShowWindow: Closing" << endl;

    this -> close();

    SoQt::exitMainLoop();
}


/**************************************************************************************************
 *  Procecure                                                                                     *
 *                                                                                                *
 *  Description: quit                                                                             *
 *  Class      : ShowWindow                                                                       *
 **************************************************************************************************/
void ShowWindow::loadHand()
{
    icub.reset(new iCubRobot(scene, "iCub", "Left Hand Start"));

    while (objects.size() > 1)
    {
        objects.pop_back();
    }

    loadRobot();

    hand_loaded = true;

    selectMoveHand(RIFR);
}


/**************************************************************************************************
 *  Procecure                                                                                     *
 *                                                                                                *
 *  Description: redraw                                                                           *
 *  Class      : ShowWindow                                                                       *
 **************************************************************************************************/
void ShowWindow::redraw(void)
{
    viewer                    -> scheduleRedraw();
    userinterface.frameViewer -> update();
    viewer                    -> scheduleRedraw();
    this                      -> update();
    viewer                    -> scheduleRedraw();
    this                      -> repaint();
}
