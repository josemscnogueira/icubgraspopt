/**************************************************************************************************
 *  File:    OrientedBoundingBox.cpp                                                              *
 *  Author:  Jose Miguel Nogueira, josemscnogueira@gmail.com                                      *
 *                                                                                                *
 *  History:                                                                                      *
 **************************************************************************************************/


/**************************************************************************************************
 *  Include Files                                                                                 *
 **************************************************************************************************/
#include <OrientedBoundingBox.h>


/**************************************************************************************************
 *  Used namespaces                                                                               *
 **************************************************************************************************/
using namespace VirtualRobot;

using std::cout;
using std::endl;
using std::string;
using std::vector;


/**************************************************************************************************
 *  Namespace: VirtualRobot                                                                       *
 **************************************************************************************************/
namespace VirtualRobot
{


/**************************************************************************************************
 *  Procecure                                                                                     *
 *                                                                                                *
 *  Description: Constructor                                                                      *
 *  Class      : OrienteBoundingBox                                                               *
 **************************************************************************************************/
OrientedBoundingBox::OrientedBoundingBox(void)
{
    object             .reset();
    mean               .setZero();
    covariance         .setZero();

    visualization_model.reset(new VisualizationNode());

    parameters.local .vertices.clear();
    parameters.global.vertices.clear();
}


/**************************************************************************************************
 *  Procecure                                                                                     *
 *                                                                                                *
 *  Description: Constructor                                                                      *
 *  Class      : OrienteBoundingBox                                                               *
 **************************************************************************************************/
OrientedBoundingBox::OrientedBoundingBox(SceneObjectPtr object) : OrientedBoundingBox()
{
    this -> object = object;

    // Get Object Trimesh and pose
    TriMeshModelPtr  trimodel = object -> getCollisionModel() -> getTriMeshModel();
    Matrix4f         pose     = object -> getCollisionModel() -> getGlobalPose();

    // Debug Print
    cout << endl << pose << endl;

    // Copy vertices from trimesh model
    vector<Vector3f> points(trimodel -> vertices);

    // Transform to global coordinates
    for(uint index = 0; index < points.size(); index += 1)
    {
        points[index] = MathTools::transformPosition(points[index], pose);
    }

    // Calculate points' statistics
    calculateMean      (points, false);
    calculateCovariance(points, false);

    // Calculate Box Orientation
    calculateBoxOrientation();

    // Calculate Box Extremes
    calculateParameters(points);

    // Calculate Vertices
    calculateVertices();

    // Debug Print
    // print();
}


/**************************************************************************************************
 *  Procecure                                                                                     *
 *                                                                                                *
 *  Description: calculateMean                                                                    *
 *  Class      : OrienteBoundingBox                                                               *
 **************************************************************************************************/
void OrientedBoundingBox::calculateMean(vector<Vector3f> points, bool reset)
{
    if (reset) mean.setZero();

    for (uint index = 0; index < points.size(); index += 1)
    {
        mean += points[index];
    }

    mean /= points.size();
}


/**************************************************************************************************
 *  Procecure                                                                                     *
 *                                                                                                *
 *  Description: calculateCovariance                                                              *
 *  Class      : OrienteBoundingBox                                                               *
 **************************************************************************************************/
void OrientedBoundingBox::calculateCovariance(vector<Vector3f> points, bool reset)
{
    if (reset) covariance.setZero();

    // Calculate expected correlations
    for (        uint column = 0; column < 3            ; column += 1)
    {
        for (    uint row    = 0; row    < 3            ; row    += 1)
        {
            for (uint index  = 0; index  < points.size(); index  += 1)
            {
                covariance(row, column) += points[index](row) * points[index](column);
            }
        }
    }

    // Normalize
    covariance /= points.size();

    // Subtract Means
    for (        uint column = 0; column < 3            ; column += 1)
    {
        for (    uint row    = 0; row    < 3            ; row    += 1)
        {
            covariance(row, column) -= mean[row] * mean[column];
        }
    }
}


/**************************************************************************************************
 *  Procecure                                                                                     *
 *                                                                                                *
 *  Description: calculateBoxOrientation                                                          *
 *  Class      : OrienteBoundingBox                                                               *
 **************************************************************************************************/
void OrientedBoundingBox::calculateBoxOrientation(void)
{
     Eigen::SelfAdjointEigenSolver<Matrix3f> eigensolver(covariance);

     if (eigensolver.info() != Eigen::Success)
     {
        cout << endl << "[Error] Could not calculate eigen vectors in OrientedBoundingBox::calculateBoxOrientation. " << endl;

        exit(-1);
     }

     orientation = eigensolver.eigenvectors();

    if ( fabs(orientation.determinant() - 1) > 0.01 )
    {
        Matrix3f m;

         m << -1, 0, 0,
               0, 1, 0,
               0, 0, 1;

        orientation = orientation * m;
    }

    // Assertion Verification
    if ( fabs(orientation.determinant() - 1) > 0.01 )
    {
        cout << endl << "Determinant: " << orientation.determinant();
        cout << endl << "[ERROR] Rotation Matrix is not a rotation matrix : OrientedBoundingBox::calculateBoxOrientation." << endl;

        exit(-1);
    }
}


/**************************************************************************************************
 *  Procecure                                                                                     *
 *                                                                                                *
 *  Description: calculateOOBBParameters                                                          *
 *  Class      : OrienteBoundingBox                                                               *
 **************************************************************************************************/
void OrientedBoundingBox::calculateParameters(vector<Vector3f> points)
{
    parameters.local.max << -FLT_MAX, -FLT_MAX, -FLT_MAX;
    parameters.local.min <<  FLT_MAX,  FLT_MAX,  FLT_MAX;

    cout << endl << parameters.local.max << endl;
    cout << endl << parameters.local.min << endl;

    for (uint column = 0; column < 3; column += 1)
    {
        Vector3f col = orientation.col(column);

        for (uint index = 0; index < points.size(); index += 1)
        {
            float projection;

            projection = col.dot(points[index]);

            if ( parameters.local.max(column) < projection) parameters.local.max(column) = projection;
            if ( parameters.local.min(column) > projection) parameters.local.min(column) = projection;
        }
    }

    // Transform to global coordinate system
    parameters.global.max           = orientation * parameters.local.max;
    parameters.global.min           = orientation * parameters.local.min;

    // Calculate Center
    parameters.local .center        = (parameters.local .max + parameters.local .min) / 2;
    parameters.global.center        = (parameters.global.max + parameters.global.min) / 2;

    // Calculate Half Distance
    parameters.local.half_distance  = (parameters.local .max - parameters.local .min) / 2;
}


/**************************************************************************************************
 *  Procecure                                                                                     *
 *                                                                                                *
 *  Description: calculateOOBBVertices                                                            *
 *  Class      : OrienteBoundingBox                                                               *
 **************************************************************************************************/
void OrientedBoundingBox::calculateVertices(void)
{
    Vector3f point = parameters.local.min;
    Vector3f add   = parameters.local.half_distance * 2;

    // Clear current vertices
    parameters.local .vertices.clear();
    parameters.global.vertices.clear();

    // Vertex 1
    parameters.local.vertices.push_back(point);

    // Vertex 2
    point += add(0) * Vector3f(1,0,0);
    parameters.local.vertices.push_back(point);

    // Vertex 3
    point += add(1) * Vector3f(0,1,0);
    parameters.local.vertices.push_back(point);

    // Vertex 4
    point -= add(0) * Vector3f(1,0,0);
    parameters.local.vertices.push_back(point);

    // Vertex 5
    point -= add(1) * Vector3f(0,1,0);
    point += add(2) * Vector3f(0,0,1);
    parameters.local.vertices.push_back(point);

    // Vertex 6
    point += add(0) * Vector3f(1,0,0);
    parameters.local.vertices.push_back(point);

    // Vertex 7
    point += add(1) * Vector3f(0,1,0);
    parameters.local.vertices.push_back(point);

    // Vertex 8
    point -= add(0) * Vector3f(1,0,0);
    parameters.local.vertices.push_back(point);

    // Transform points to global
    for(uint index = 0; index < parameters.local.vertices.size(); index += 1)
    {
        parameters.global.vertices.push_back( orientation * parameters.local.vertices[index]);
    }
}


/**************************************************************************************************
 *  Procecure                                                                                     *
 *                                                                                                *
 *  Description: print                                                                            *
 *  Class      : OrienteBoundingBox                                                               *
 **************************************************************************************************/
void OrientedBoundingBox::print(bool global)
{
    cout << endl << "********* Print Oriented Bounding Box *********"             << endl;
    cout << endl << "  ---  Statistics:"                                                 ;
    cout << endl << "Mean: ("      << mean(0) << " , " << mean(1) << " , " << mean(2) << ")" << endl;
    cout << endl << "Covariance:"  << endl    << covariance                           << ")" << endl;
    cout << endl << "Orientation:" << endl    << orientation                          << ")" << endl;

    if (global)
    {
        cout << endl << "Max   : ("             << parameters.global.max   (0)        << " , " << parameters.global.max   (1)        << " , " << parameters.global.max   (2)        << ")" << endl;
        cout << endl << "Min   : ("             << parameters.global.min   (0)        << " , " << parameters.global.min   (1)        << " , " << parameters.global.min   (2)        << ")" << endl;
        cout << endl << "Center: ("             << parameters.global.center(0)        << " , " << parameters.global.center(1)        << " , " << parameters.global.center(2)        << ")" << endl;

        for (uint index = 0; index < parameters.global.vertices.size(); index += 1)
        {
            cout << endl << "Vertices[" << index << "]: (" << parameters.global.vertices[index](0) << " , " << parameters.global.vertices[index](1) << " , " << parameters.global.vertices[index](2) << ")" << endl;
        }
    }
    else
    {
        cout << endl << "Max          : ("      << parameters.local .max   (0)        << " , " << parameters.local .max   (1)        << " , " << parameters.local .max   (2)        << ")" << endl;
        cout << endl << "Min          : ("      << parameters.local .min   (0)        << " , " << parameters.local .min   (1)        << " , " << parameters.local .min   (2)        << ")" << endl;
        cout << endl << "Center       : ("      << parameters.local .center(0)        << " , " << parameters.local .center(1)        << " , " << parameters.local .center(2)        << ")" << endl;
        cout << endl << "Half Distance: ("      << parameters.local .half_distance(0) << " , " << parameters.local .half_distance(1) << " , " << parameters.local .half_distance(2) << ")" << endl;

        for (uint index = 0; index < parameters.local .vertices.size(); index += 1)
        {
            cout << endl << "Vertices[" << index << "]: (" << parameters.local .vertices[index](0) << " , " << parameters.local .vertices[index](1) << " , " << parameters.local .vertices[index](2) << ")" << endl;
        }
    }
}


/**************************************************************************************************
 *  Procecure                                                                                     *
 *                                                                                                *
 *  Description: print                                                                            *
 *  Class      : OrienteBoundingBox                                                               *
 **************************************************************************************************/
vector<Vector3f> OrientedBoundingBox::getVertices(bool global)
{
    if (global) return parameters.global.vertices;
    else        return parameters.local .vertices;
}


/**************************************************************************************************
 *  Procecure                                                                                     *
 *                                                                                                *
 *  Description: print                                                                            *
 *  Class      : OrienteBoundingBox                                                               *
 **************************************************************************************************/
vector<ObstaclePtr> OrientedBoundingBox::createVerticesObstacles(bool global)
{
    vector<Vector3f>     vertices;
    vector<ObstaclePtr>  result;
    Matrix4f             pose;
                         pose.setIdentity();

    if (global) vertices = parameters.global.vertices;
    else        vertices = parameters.local .vertices;

    for(uint index = 0; index < vertices.size(); index += 1)
    {
        result.push_back(Obstacle::createSphere (2.0f));

        pose.topRightCorner(3,1) = vertices[index];

        result[index] -> setGlobalPose(pose);
    }

    return result;
}


/**************************************************************************************************
 *  Procecure                                                                                     *
 *                                                                                                *
 *  Description: getCenter                                                                        *
 *  Class      : OrienteBoundingBox                                                               *
 **************************************************************************************************/
Vector3f OrientedBoundingBox::getCenter(void)
{
    return parameters.global.center;
}


/**************************************************************************************************
 *  Procecure                                                                                     *
 *                                                                                                *
 *  Description: getMax                                                                           *
 *  Class      : OrienteBoundingBox                                                               *
 **************************************************************************************************/
Vector3f OrientedBoundingBox::getMax(bool global)
{
    if (global) return parameters.global.max;
    else        return parameters.local .max;
}


/**************************************************************************************************
 *  Procecure                                                                                     *
 *                                                                                                *
 *  Description: getMin                                                                           *
 *  Class      : OrienteBoundingBox                                                               *
 **************************************************************************************************/
Vector3f OrientedBoundingBox::getMin(bool global)
{
    if (global) return parameters.global.min;
    else        return parameters.local .min;
}


/**************************************************************************************************
 *  Procecure                                                                                     *
 *                                                                                                *
 *  Description: calculateScaling                                                                 *
 *  Class      : OrienteBoundingBox                                                               *
 **************************************************************************************************/
Vector3f OrientedBoundingBox::calculateScaling(Matrix3f coordinate_system)
{
    if (coordinate_system.determinant() == 0)
    {
        cout << endl << "[ERROR] Transformation matring in OrientedBoundingBox::calculateScaling can't be inversed.";
        exit(-1);
    }

    // Calculate Parameter Scaling
    Vector3f ob_max     = parameters.global.max;
    Vector3f ob_min     = parameters.global.min;
    Vector3f ob_avg     = parameters.global.center;
    Vector3f scaling;

    ob_max             -= ob_avg;
    ob_min             -= ob_avg;

    ob_max              = coordinate_system.inverse() * ob_max;
    ob_min              = coordinate_system.inverse() * ob_min;

    if ( ob_max(0) < 0 ) scaling(0) =  -ob_max(0) + ob_min(0);
    else                 scaling(0) =  -ob_min(0) + ob_max(0);

    if ( ob_max(1) < 0 ) scaling(1) =  -ob_max(1) + ob_min(1);
    else                 scaling(1) =  -ob_min(1) + ob_max(1);

    if ( ob_max(2) < 0 ) scaling(2) =  -ob_max(2);
    else                 scaling(2) =  -ob_min(2);

    return scaling;
}


/**************************************************************************************************
 *  Procecure                                                                                     *
 *                                                                                                *
 *  Description: getOrientation                                                                   *
 *  Class      : OrienteBoundingBox                                                               *
 **************************************************************************************************/
Matrix3f OrientedBoundingBox::getOrientation()
{
    return orientation;
}

} // End of Namespace: VirtualRobot