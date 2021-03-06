/**************************************************************************************************
 *  File:    iCubOptimizable.cpp                                                                  *
 *  Author:  Jose Miguel Nogueira, josemscnogueira@gmail.com                                      *
 *                                                                                                *
 *  History:                                                                                      *
 **************************************************************************************************/


/**************************************************************************************************
 *  Include Files                                                                                 *
 **************************************************************************************************/
#include <iCubOptimizable.hpp>
#include <iCubHand.hpp>
#include <boost/lexical_cast.hpp>
#include <chrono>


/**************************************************************************************************
 *  Used namespaces                                                                               *
 **************************************************************************************************/
using std::vector;
using std::cout;
using std::endl;
using std::default_random_engine;
using std::normal_distribution;
using iCub::iCubRobotPtr;
using iCub::Vector09dof;
using VirtualRobot::SceneObjectPtr;
using VirtualRobot::CollisionChecker;

/**************************************************************************************************
 *  Definitions                                                                                   *
 **************************************************************************************************/
#define METRIC_START_VALUE   0.000
#define METRIC_ENDED_VALUE   1.000
#define METRIC_STEP          0.001

namespace bayesopt {

/**************************************************************************************************
 *  Procecure                                                                                     *
 *                                                                                                *
 *  Description: Constructor                                                                      *
 *  Class      : iCubOptimizable                                                                  *
 **************************************************************************************************/
iCubOptimizable::iCubOptimizable(iCubRobotPtr robot, SceneObjectPtr object, iCubOptParameters& params) :
_icubparams(params)
{
    this -> dim     = robot -> calculateNumberOfDimensionsForLearning(params.grasp);
    this -> name    = "iCubOptimizable";
    this -> ymin    = 0;
    this -> ymax    = 1;

    _robot          = robot;
    _col_checker    = CollisionChecker::getGlobalCollisionChecker();

    _object_set.reset(new SceneObjectSet());
    _object_set -> addSceneObject(object);
    _object = _object_set -> getSceneObject(0);

    _original_dim        = dim;
    _active_variables_mask.resize(_original_dim);

    if ( (_icubparams.default_query.size() == 0) || (_icubparams.active_variables.size() == 0) )
    {
        _icubparams.default_query = vectord(_original_dim, 0.0);

        for (uint index = 0; index < _original_dim; index += 1)
        {
            _icubparams.active_variables.push_back(index);
        }
    }

    if (_original_dim != _icubparams.default_query.size())
    {
        cout << endl << "[ERROR] Default Query size doesn't match iCubOptimizable Dim. From: iCubOptimizable::iCubOptimizable.";

        exit(-1);
    }

    for (uint index = 0; index < _original_dim; index += 1)
    {
        _active_variables_mask[index] = true;
    }

    setExplorationBoundingBox();
    initializeRandomDist(_icubparams.trial_stddev);
    selectOptVariables();
}


/**************************************************************************************************
 *  Procecure                                                                                     *
 *                                                                                                *
 *  Description: applyQueryToHand                                                                 *
 *  Class      : iCubOptimizable                                                                  *
 **************************************************************************************************/
vector<GraspResult> iCubOptimizable::applyQueryToHand(const vectord& query, std::vector<double>& position, std::vector<double>& orientation)
{
    // Variables
    vector<GraspResult>     qualities;
    vectord                 noise(_original_dim);
    unsigned                rand_seed = std::chrono::system_clock::now().time_since_epoch().count();
    default_random_engine   generator(rand_seed);
    vector<uint>            index_dim;

    for(uint trial = 0; trial < _icubparams.n_grasp_trials; trial += 1)
    {
        // Reset number of contacts
        _number_of_contacts = 0;

        noise = query;

        if (_icubparams.n_grasp_trials > 1)
        {
            // Add noise to query
            for(uint index = 0; index < _original_dim; index += 1)
            {
                if ( (noise(index) >= lower_bound(index)) &&
                     (noise(index) <= upper_bound(index))    )
                {
                     noise(index) = query(index) + (_noise_distribution[index])(generator);
                }
            }
        }

        std::vector<double> dummy1, dummy2;

        // Reset Grasp and Hand configs
        applyQueryToHandMinimal(vectord(_original_dim, 0), dummy1, dummy2);

        // Apply inteded query
        GraspQualityPtr quality = applyQueryToHandMinimal(noise, position, orientation);

        if (!quality) return qualities;

        // Return quality
        if (_number_of_contacts > 1) { qualities.push_back(GraspResult(quality));}
        else                         { return qualities;}

        if (quality -> getGraspQuality() < _icubparams.grasp_threshold) return qualities;
    }

    // Return qualities
    return qualities;
}


/**************************************************************************************************
 *  Procecure                                                                                     *
 *                                                                                                *
 *  Description: applyQueryToHandMinimal                                                          *
 *  Class      : iCubOptimizable                                                                  *
 **************************************************************************************************/
GraspQualityPtr iCubOptimizable::applyQueryToHandMinimal(const vectord& query, std::vector<double>& position, std::vector<double>& orientation)
{
    GraspQualityPtr result;

    // Calculate Grasp Components
    Vector09dof grasp_components = calculateGraspComponents(query);

    // Open Hand if it is closed
    _robot -> openHand(grasp_components, _icubparams.grasp);

    // Approach hand according to query -> second argument is a vector for (dx, dy, dz, dax, day, daz)
    _robot -> approachHand(_object, calculateApproachVector(query), _icubparams.position);

    // Check if hand and object collide
    CollisionModelPtr         col_model1 = _object -> getCollisionModel();
    vector<CollisionModelPtr> col_model2 = _robot  -> getCollisionModel();

    // If collides return null
    if ( _col_checker -> checkCollision(col_model2, col_model1) )
    {
        _robot -> closeHand(grasp_components, _icubparams.grasp, _object_set, _number_of_contacts);
    }
    else
    {
        // Get grasp quality according to query -> first argument is a vector for (grasp sinergy components)
        result = _robot -> closeHand(grasp_components, _icubparams.grasp, _object_set, _number_of_contacts);
    }

    return result;
}


/**************************************************************************************************
 *  Procecure                                                                                     *
 *                                                                                                *
 *  Description: evaluateGraspQuality                                                             *
 *  Class      : iCubOptimizable                                                                  *
 **************************************************************************************************/
double iCubOptimizable::evaluateGraspQuality(vector<GraspResult> qualities, float& isforceclosure)
{
    double result         = 0.0;
           isforceclosure = 0.0;

    // If there is no qualities return 0.0
    if (qualities.size() < 1) return result;

    // Sum all qualities
    for (uint trial = 0; trial < qualities.size(); trial += 1)
    {
        result += qualities[trial].measure;

        if (qualities[trial].force_closure) isforceclosure += 1.0;
    }

    // Average them
    result         /= qualities.size();
    isforceclosure /= qualities.size();

    // Return Grasp Quality
    return result;
}


/**************************************************************************************************
 *  Procecure                                                                                     *
 *                                                                                                *
 *  Description: evaluate                                                             *
 *  Class      : iCubOptimizable                                                                  *
 **************************************************************************************************/
double iCubOptimizable::evaluate(vectord query)
{
    float isforceclosure;

    chooseActiveVariables(query);

    std::vector<double> dummy1, dummy2;

    // query = _icubparams.default_query;

    return -evaluateGraspQuality(applyQueryToHand(query, dummy1, dummy2), isforceclosure);
}


/**************************************************************************************************
 *  Procecure                                                                                     *
 *                                                                                                *
 *  Description: calculateGraspComponents                                                         *
 *  Class      : iCubOptimizable                                                                  *
 **************************************************************************************************/
Vector09dof iCubOptimizable::calculateGraspComponents(const vectord& query)
{
    Vector09dof result;
                result.fill(0);

    // If there's no grasp components, only approach components, return
    if (_original_dim < 7)
    {
        cout << endl << "[ERROR] There are no auxiliary components. From: iCubOptimizable::calculateGraspComponents.";

        exit(-1);
    }

    // Calculate Grasp Components
    for(uint index = 0; index < (_original_dim - 6); index += 1)
    {
        result(index + 1, 0) = query (6 + index);
    }

    return result;
}


/**************************************************************************************************
 *  Procecure                                                                                     *
 *                                                                                                *
 *  Description: calculateApproachVector                                                          *
 *  Class      : iCubOptimizable                                                                  *
 **************************************************************************************************/
vector<float> iCubOptimizable::calculateApproachVector(const vectord& query)
{
    // If there isn't all six approach components, return
    if (_original_dim < 6)
    {
        cout << endl << "[ERROR] Approach components must have 6 dimensions. From: iCubOptimizable::calculateApproachVector.";

        exit(-1);
    }

    vector<float> result;

    result.push_back(query(0));
    result.push_back(query(1));
    result.push_back(query(2));
    result.push_back(query(3));
    result.push_back(query(4));
    result.push_back(query(5));

    return result;
}


/**************************************************************************************************
 *  Procecure                                                                                     *
 *                                                                                                *
 *  Description: setExplorationBoundingBox                                                        *
 *  Class      : iCubOptimizable                                                                  *
 **************************************************************************************************/
void iCubOptimizable::setExplorationBoundingBox(void)
{
    lower_bound = vectord(_original_dim, 0);
    upper_bound = vectord(_original_dim, 1);

    // Set dx, dy, dz, droll, dpitch, dyaw bounds
    for(uint index = 0; index < 6; index += 1)
    {
        lower_bound(index) =    0;
        upper_bound(index) = 1000;
    }

    // Grasp Auxiliary Components
    for(uint index = 6; index < _original_dim; index += 1)
    {
        lower_bound(index) = -M_PI;
        upper_bound(index) =  M_PI;
    }
}

/**************************************************************************************************
 *  Procecure                                                                                     *
 *                                                                                                *
 *  Description: getBoundingBox                                                                   *
 *  Class      : iCubOptimizable                                                                  *
 **************************************************************************************************/
void iCubOptimizable::getBoundingBox(vectord& lower, vectord& upper)
{
    uint active_counter = 0;
         lower          = vectord(dim, 0.0);
         upper          = vectord(dim, 0.0);

    for (uint index = 0; index < _original_dim; index += 1)
    {
        if (_active_variables_mask[index] == true)
        {
            lower[active_counter] = lower_bound[index];
            upper[active_counter] = upper_bound[index];

            active_counter       += 1;
        }
    }

    if (active_counter != dim)
    {
        cout << endl << "[ERROR] Bounds size are not equal to mask active components. From: iCubOptimizable::getBoundingBox.";

        exit(-1);
    }
};


/**************************************************************************************************
 *  Procecure                                                                                     *
 *                                                                                                *
 *  Description: initializeRandomDist                                                             *
 *  Class      : iCubOptimizable                                                                  *
 **************************************************************************************************/
void iCubOptimizable::initializeRandomDist(double std_dev)
{
    _noise_distribution.clear();

    // Update noise distribution according to the exploration boundries
    for(uint index = 0; index < _original_dim; index += 1)
    {
        normal_distribution<double> distribution( 0, (upper_bound(index) - lower_bound(index)) * std_dev);

        _noise_distribution.push_back(distribution);
    }
}


/**************************************************************************************************
 *  Procecure                                                                                     *
 *                                                                                                *
 *  Description: showBestGrasps                                                                   *
 *  Class      : iCubOptimizable                                                                  *
 **************************************************************************************************/
void iCubOptimizable::showBestGrasps(uint index, LearningQueueWrapper& best_grasps)
{
    float isforceclosure;

    vector<vectord> best_queries = best_grasps.getBestQueries();

    if (index < best_queries.size())
    {
        cout << endl << "Best Query:   " << best_queries[index] << endl;

        std::vector<double> dummy1, dummy2;

        evaluateGraspQuality(applyQueryToHand(best_queries[index], dummy1, dummy2), isforceclosure);
    }
}


/**************************************************************************************************
 *  Procecure                                                                                     *
 *                                                                                                *
 *  Description: getOptParams                                                                     *
 *  Class      : iCubOptimizable                                                                  *
 **************************************************************************************************/
void iCubOptimizable::getOptParams(TgpParameters& tgp_params, Parameters& opt_params)
{
    TGPOptimizable::getOptParams(tgp_params, opt_params);

    opt_params.n_init_samples    =   30;
    opt_params.n_iterations      =  150 - opt_params.n_init_samples;
    opt_params.crit_params[0]    =    1;    // exp
    opt_params.crit_params[1]    =    0.00; // bias
    opt_params.noise             =    1e-4;
    opt_params.sigma_s           =    0.35;

    tgp_params.mcmc_particles    =   10;
    tgp_params.min_data_per_leaf =   20;
    tgp_params.wheight_power     =    1;
}


/**************************************************************************************************
 *  Procecure                                                                                     *
 *                                                                                                *
 *  Description: selectOptVariables                                                               *
 *  Class      : iCubOptimizable                                                                  *
 **************************************************************************************************/
void iCubOptimizable::selectOptVariables(void)
{
    // Clear Active Mask
    _active_variables_mask.clear();
    _active_variables_mask.resize(_original_dim);

    // Update number of active dimensions
    dim = _icubparams.active_variables.size();

    // Set all to false
    for (uint index = 0; index < _original_dim; index += 1)
    {
        _active_variables_mask[index] = false;
    }

    // Set Active Variable Mask
    for (uint index = 0; index < dim; index += 1)
    {
        if ( (_icubparams.active_variables[index] >= 0) && (_icubparams.active_variables[index] < _original_dim) )
        {
            _active_variables_mask[_icubparams.active_variables[index]] = true;
        }
    }
}


/**************************************************************************************************
 *  Procecure                                                                                     *
 *                                                                                                *
 *  Description: chooseActiveVariables                                                            *
 *  Class      : iCubOptimizable                                                                  *
 **************************************************************************************************/
void iCubOptimizable::chooseActiveVariables(vectord& query)
{
    if (dim != query.size())
    {
        cout << endl << "[ERROR] Query size is not equal to mask active components. From: iCubOptimizable::chooseActiveVariables.";

        exit(-1);
    }

    uint    variables_updated = 0;
    vectord result            = vectord(_original_dim, 0.0);

    for (uint index = 0; index < _original_dim; index += 1)
    {
        if (_active_variables_mask[index] == true)
        {
            result[index]      = query[variables_updated];
            variables_updated += 1;
        }
        else
        {
            result[index]      = _icubparams.default_query[index];
        }
    }

    // Return new query
    query = result;
}


} // End of namespace bayesopt
