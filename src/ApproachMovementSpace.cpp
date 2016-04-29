/**************************************************************************************************
 *  File:    ApproachMovementSpace.cpp                                                            *
 *  Author:  Jose Miguel Nogueira, josemscnogueira@gmail.com                                      *
 *                                                                                                *
 *  History:                                                                                      *
 **************************************************************************************************/


/**************************************************************************************************
 *  Include Files                                                                                 *
 **************************************************************************************************/
#include <ApproachMovementSpace.h>


/**************************************************************************************************
 *  Used namespaces                                                                               *
 **************************************************************************************************/
using namespace VirtualRobot;

using std::cout;
using std::endl;
using std::string;
using std::vector;
using std::sort;


/**************************************************************************************************
 *  Namespace: GraspStudio                                                                        *
 **************************************************************************************************/
namespace GraspStudio
{


/**************************************************************************************************
 *  Procecure                                                                                     *
 *                                                                                                *
 *  Description: Constructor                                                                      *
 *  Class      : ApproachMovementSpace                                                            *
 **************************************************************************************************/
ApproachMovementSpace::ApproachMovementSpace(SceneObjectPtr object, EndEffectorPtr eef) : ApproachMovementGenerator(object, eef)
{
	// Set offset
	offset.setZero();
	offset.topRightCorner(3,1) = Vector3f(50,-10,-15);

	offset(2,0) =  1;
	offset(1,1) = -1;
	offset(0,2) = -1;

	// Link objects
	this -> object = object;
	this -> eef    = eef;

	// Create BoudingBox
	boundingbox    = object -> getCollisionModel() -> getBoundingBox();
	oobb.reset(new OrientedBoundingBox(object));

	cout << endl << "Center of this boundingbox:" << endl << (boundingbox.getMax() + boundingbox.getMin()) / 2 << endl;
}


/**************************************************************************************************
 *  Procecure                                                                                     *
 *                                                                                                *
 *  Description: setParameters                                                                    *
 *  Class      : ApproachMovementSpace                                                            *
 **************************************************************************************************/
void ApproachMovementSpace::setParameters(Vector6f approach_parameters)
{
	this -> approach_parameters = approach_parameters;
}


/**************************************************************************************************
 *  Procecure                                                                                     *
 *                                                                                                *
 *  Description: calculateEndEffectorOrientation                                                  *
 *  Class      : ApproachMovementSpace                                                            *
 **************************************************************************************************/
Matrix3f ApproachMovementSpace::calculateRotationMatrixEulerZYZ(float angle_z1, float angle_y1, float angle_z2)
{
	AngleAxisf         angle1(angle_z1,  Eigen::Vector3f::UnitZ());
	AngleAxisf         angle2(angle_y1,  Eigen::Vector3f::UnitY());
	AngleAxisf         angle3(angle_z2,  Eigen::Vector3f::UnitZ());

	Quaternion<float>  rotation_quaternion = angle1 * angle2 * angle3;

	return rotation_quaternion.matrix();
}

Matrix3f ApproachMovementSpace::calculateRotationSimpleXYZ(float angle_x, float angle_y, float angle_z)
{
	AngleAxisf         angle1(angle_x,  Eigen::Vector3f::UnitX());
	AngleAxisf         angle2(angle_y,  Eigen::Vector3f::UnitY());
	AngleAxisf         angle3(angle_z,  Eigen::Vector3f::UnitZ());

	Quaternion<float>  rotation_quaternion = angle1 * angle2 * angle3;

	return rotation_quaternion.matrix();
}


/**************************************************************************************************
 *  Procecure                                                                                     *
 *                                                                                                *
 *  Description: calculateCarthesianCoordinates                                                   *
 *  Class      : ApproachMovementSpace                                                            *
 **************************************************************************************************/
Vector3f ApproachMovementSpace::calculateCarthesianCoordinates(float theta, float phi, float rho)
{
	Vector3f result;

	result(0) = rho * sin(theta) * cos(phi);
	result(1) = rho * sin(theta) * sin(phi);
	result(2) = rho * cos(theta) *        1;

	return result;
}


/**************************************************************************************************
 *  Procecure                                                                                     *
 *                                                                                                *
 *  Description: moveHandRelativeToObject                                                         *
 *  Class      : ApproachMovementSpace                                                            *
 **************************************************************************************************/
Matrix4f ApproachMovementSpace::moveHandRelativeToObject(float theta, float phi, float rho, bool update)
{
	RobotPtr     hand = eef  -> getRobot();

	// Don't let the hand collide with the object
	if (rho < 10) rho = 10;

	// Calculate new XYZ coordinates for the hand
	Vector3f new_pos     = object -> getGlobalPose().topLeftCorner (3,3) * calculateCarthesianCoordinates(theta, phi, rho);
	         new_pos    += object -> getGlobalPose().topRightCorner(3,1);

	Matrix4f new_pose    = hand -> getGlobalPose();
	Matrix3f orientation;

	// Calculate First Vector of the orientation Matrix
		// It is colinear with the distance vector between object and hand
	orientation.block(0,0,3,1) = object -> getGlobalPose().topRightCorner(3,1) - new_pos;

	// Calculate the Remainder two orientation vectors
	Matrix3f inertia  = object -> getGlobalPose().topLeftCorner(3,3);
		     inertia *= object -> getInertiaMatrix();

	// Select two vectors from the objects inertia matrix
	     uint vectors_left  = 2;
	for (uint column        = 0; column < 3; column += 1)
	{
		Vector3f v1 = orientation.block(0,0,3,1);

		if( v1.dot( inertia.col(column) ) != 1)
		{
			orientation.block(0,vectors_left,3,1) = inertia.col(column);

			vectors_left -= 1;

			if (vectors_left == 0) break;
		}
	}

	// Check if two new vectors were included
	if ( vectors_left != 0)
	{
		cout << endl << "[Error] Function: moveHandRelativeToObject   |   The two vectors from the object's inertia matrix were not selected.";

		exit(-1);
	}

	// Make the orientation matrix orthonormalized
	orientation  = gramSchmidtNormalization3f(orientation);
	orientation *= offset.topLeftCorner(3,3);

	// Check if the new orientation is mirrored or not
		// If so, correct it
	if ( ( orientation.determinant() - (-1) <  0.01) &&
		 ( orientation.determinant() - (-1) > -0.01) )
	{
		orientation.col(1) *= -1;
	}

	// Update new hand pose
	new_pose.topLeftCorner (3,3) = orientation;
	new_pose.topRightCorner(3,1) = new_pos - orientation * offset.topRightCorner(3,1);

	// Set updated pose
	if (update) hand -> setGlobalPose(new_pose);

	return new_pose;
}


/**************************************************************************************************
 *  Procecure                                                                                     *
 *                                                                                                *
 *  Description: moveHandRelativeToObject                                                         *
 *  Class      : ApproachMovementSpace                                                            *
 **************************************************************************************************/
Matrix4f ApproachMovementSpace::moveHandRelativeToObject(Vector6f approach_parameters, bool update)
{
	setParameters(approach_parameters);

	Matrix4f new_pose  = moveHandRelativeToObject( approach_parameters(POSI_THETA),
					 							   approach_parameters(POSI_PHI  ),
					 	                           approach_parameters(POSI_RHO  ), false);

	Vector3f delta_pos = Vector3f(approach_parameters(HAND_DX), approach_parameters(HAND_DY), 0.0f);
             delta_pos = new_pose.topLeftCorner(3,3) * delta_pos;

    // Update New Pose
    new_pose.topLeftCorner (3,3)  = new_pose.topLeftCorner (3,3) * calculateRotationMatrixEulerZYZ(approach_parameters(HAND_ANGLEZ),0,0);
    new_pose.topRightCorner(3,1) += delta_pos;

    // Set updated pose
    if (update) eef  -> getRobot() -> setGlobalPose(new_pose);

    return new_pose;
}


/**************************************************************************************************
 *  Procecure                                                                                     *
 *                                                                                                *
 *  Description: moveHandDistanceClass                                                            *
 *  Class      : ApproachMovementSpace                                                            *
 **************************************************************************************************/
Eigen::Matrix4f ApproachMovementSpace::moveHandDistanceClass(uint mode, bool update)
{
	Matrix4f new_pose;
			 new_pose.setIdentity();
	Matrix4f palm_wrist_transformation;
			 palm_wrist_transformation.setIdentity();
	Matrix3f rotation, offset;

	offset <<  0,  0,  1,
	           0, -1,  0,
	           1,  0,  0;

	palm_wrist_transformation.topRightCorner(3,1) = - Vector3f(60,0,-25); // 60, 10, 25

	switch (mode)
	{
		case RIFR :  // Right-Grip Thumb-Front
		{
			rotation <<  1,  0,  0,
			             0,  1,  0,
			             0,  0,  1;
			break;
		}

		case RIUP : // Right-Grip Thump-Up
		{
			rotation <<  1,  0,  0,
			             0,  0, -1,
			             0,  1,  0;
			break;
        }

		case RIBA : // Right-Grip Thump-Back
		{
			rotation <<  1,  0,  0,
			             0, -1,  0,
			             0,  0, -1;
			break;
        }

        case RIDO : // Right-Grip Thump-Down
		{
			rotation <<  1,  0,  0,
			             0,  0,  1,
			             0, -1,  0;
			break;
        }

        case FRLE : // Front-Grip Thumb-Left
        {
			rotation <<  0,  0,  1,
						 0,  1,  0,
						-1,  0,  0;

        	break;
        }

        case FRUP :
        {
			rotation <<  0,  1,  0,
						 0,  0, -1,
						-1,  0,  0;

        	break;
        }

        case FRRI :
        {
			rotation <<  0,  0, -1,
						 0, -1,  0,
						-1,  0,  0;

        	break;
        }

        case FRDO :
        {
			rotation <<  0, -1,  0,
						 0,  0,  1,
						-1,  0,  0;

        	break;
        }

        case LEBA :
        {
			rotation << -1,  0,  0,
			             0,  1,  0,
			             0,  0, -1;
			break;
		}

        case LEUP :
        {
			rotation << -1,  0,  0,
			             0,  0, -1,
			             0, -1,  0;
			break;
		}


        case LEFR :
        {
			rotation << -1,  0,  0,
			             0, -1,  0,
			             0,  0,  1;
			break;
		}

        case LEDO :
        {
			rotation << -1,  0,  0,
			             0,  0,  1,
			             0,  1,  0;
			break;
		}

        case BARI :
        {
			rotation <<  0,  0, -1,
						 0,  1,  0,
						 1,  0,  0;

        	break;
        }

        case BAUP :
        {
			rotation <<  0, -1,  0,
						 0,  0, -1,
						 1,  0,  0;

        	break;
        }

        case BALE :
        {
			rotation <<  0,  0,  1,
						 0, -1,  0,
						 1,  0,  0;

        	break;
        }

        case BADO :
        {
			rotation <<  0,  1,  0,
						 0,  0,  1,
						 1,  0,  0;

        	break;
        }

        case TOFR :
        {
			rotation <<  0, -1,  0,
						 1,  0,  0,
						 0,  0,  1;

        	break;
        }

        case TOLE :
        {
			rotation <<  0,  0,  1,
						 1,  0,  0,
						 0,  1,  0;

        	break;
        }

        case TOBA :
        {
			rotation <<  0,  1,  0,
						 1,  0,  0,
						 0,  0, -1;

        	break;
        }

        case TORI :
        {
			rotation <<  0,  0, -1,
						 1,  0,  0,
						 0, -1,  0;

        	break;
        }

        case BOFR :
        {
			rotation <<  0, -1,  0,
						-1,  0,  0,
						 0,  0, -1;

        	break;
        }

        case BORI :
        {
			rotation <<  0,  0, -1,
						-1,  0,  0,
						 0,  1,  0;

        	break;
        }

        case BOBA :
        {
			rotation <<  0,  1,  0,
						-1,  0,  0,
						 0,  0,  1;

        	break;
        }

        case BOLE :
        {
			rotation <<  0,  0,  1,
						-1,  0,  0,
						 0, -1,  0;

        	break;
        }

        default :
        {
        	return new_pose;
        }
	}

	// Calculate Default Orientation
	rotation = oobb -> getOrientation() * rotation * offset;

	// Calculate Parameter Scaling
	Vector3f scaling = oobb -> calculateScaling(rotation);

	approach_parameters[POS_DX] = ((approach_parameters[POS_DX] - 500.0f) * (scaling(0) + 50) / 1000.0);
	approach_parameters[POS_DY] = ((approach_parameters[POS_DY] - 500.0f) * (scaling(1) +  0) / 1000.0);
	approach_parameters[POS_DZ] = ((approach_parameters[POS_DZ] -   0.0f)                     /   25.0) * (1 + ((scaling(2) / 50))) + (scaling(2) / 3);
	approach_parameters[ANG_Z1] = ((approach_parameters[ANG_Z1] - 500.0f) * 1 * M_PI          / 2000.0);
	approach_parameters[ANG_Y1] = ((approach_parameters[ANG_Y1] - 500.0f) * 1 * M_PI          / 2000.0);
	approach_parameters[ANG_Z2] = ((approach_parameters[ANG_Z2] - 500.0f) * 1 * M_PI          / 2000.0);

	// Assertion Verification
	if ( fabs(rotation.determinant() - 1) > 0.01 )
    {
        cout << endl << "Determinant: " << rotation.determinant();
        cout << endl << "[ERROR] Rotation Matrix is not a rotation matrix : ApproachMovementSpace::moveHandDistanceClass." << endl;

        exit(-1);
    }

	// Update new pose
	new_pose.topLeftCorner (3,3) = rotation * calculateRotationSimpleXYZ( approach_parameters[ANG_Z1],
                                                                          approach_parameters[ANG_Y1],
	                                                                      approach_parameters[ANG_Z2] );

	new_pose.topRightCorner(3,1) = oobb -> getCenter() + rotation * Vector3f( approach_parameters[POS_DX],
	                                                                          approach_parameters[POS_DY],
	                                                                          approach_parameters[POS_DZ] );

	// New pose will be used for the wrist. Transform new_pose for palm to new_pose for wrist
	new_pose = new_pose * palm_wrist_transformation;

    // Set updated pose
    if (update) eef  -> getRobot() -> setGlobalPose(new_pose);

    return new_pose;
}


/**************************************************************************************************
 *  Procecure                                                                                     *
 *                                                                                                *
 *  Description: moveHandDistanceClass                                                            *
 *  Class      : ApproachMovementSpace                                                            *
 **************************************************************************************************/
Eigen::Matrix4f ApproachMovementSpace::moveHandDistanceClass(uint mode, Vector6f parameters, bool update)
{
	// Set parameters
	setParameters(parameters);

	// Update new pose
	Matrix4f new_pose = moveHandDistanceClass(mode, false);

    // Set updated pose
    if (update) eef  -> getRobot() -> setGlobalPose(new_pose);

    return new_pose;
}


/**************************************************************************************************
 *  Procecure                                                                                     *
 *                                                                                                *
 *  Description: createNewApproachPose                                                            *
 *  Class      : ApproachMovementSpace                                                            *
 **************************************************************************************************/
Eigen::Matrix4f ApproachMovementSpace::createNewApproachPose(void)
{
	Matrix4f ones;

	float theta = (rand() % 10000) * 1 * M_PI / 10000.0f;
	float phi   = (rand() % 10000) * 2 * M_PI / 10000.0f;
	float rho   = (rand() % 10000) * 1 *    1 /   100.0f;

	moveHandRelativeToObject(theta, phi, rho);

	ones.setRandom();
	return ones;
}

/**************************************************************************************************
 *  Procecure                                                                                     *
 *                                                                                                *
 *  Description: createNewApproachPose                                                            *
 *  Class      : ApproachMovementSpace                                                            *
 **************************************************************************************************/
Vector3f ApproachMovementSpace::normalizeVector3f(Vector3f vector1)
{
	// cout << endl << "Non-Normalized Vector: " << endl << vector1 << endl;

	float norm = sqrt(vector1.dot(vector1));

	for(uint index = 0; index < 3; index += 1)
	{
		vector1[index] /= norm;
	}

	// cout << endl << "Normalized Vector: "      << endl << vector1 << endl;

	return vector1;
}


/**************************************************************************************************
 *  Procecure                                                                                     *
 *                                                                                                *
 *  Description: gramSchmidtNormalization                                                         *
 *  Class      : ApproachMovementSpace                                                            *
 **************************************************************************************************/
Matrix3f ApproachMovementSpace::gramSchmidtNormalization3f(Matrix3f matrix)
{
	// cout << endl << "Non-Normalized Matrix: " << endl << matrix << endl;

	uint matrix_size = matrix.diagonalSize();

	for(uint column = 0; column < matrix_size; column += 1)
	{
		// Step 1: Normalize vector
		matrix.col(column) = normalizeVector3f( matrix.col(column) );

		// Step 2: Eliminate the projection of this vector
		for(uint column_proj = column + 1; column_proj < matrix_size; column_proj += 1)
		{
			Vector3f vj = matrix.col(column_proj);

			matrix.col(column_proj) -= vj.dot( matrix.col(column) ) * matrix.col(column);
		}
	}

	// cout << endl << "Normalized Matrix: " << endl << matrix << endl;

	return matrix;
}


/**************************************************************************************************
 *  Procecure                                                                                     *
 *                                                                                                *
 *  Description: swapColumns                                                                      *
 *  Class      : ApproachMovementSpace                                                            *
 **************************************************************************************************/
Matrix3f ApproachMovementSpace::swapColumns(Matrix3f matrix, uint column1, uint column2)
{
	Vector3f aux        = matrix.col(column1);

	matrix.col(column1) = matrix.col(column2);
	matrix.col(column2) = aux;

	return matrix;
}


/**************************************************************************************************
 *  Procecure                                                                                     *
 *                                                                                                *
 *  Description: getEEFGlobalPose                                                                 *
 *  Class      : ApproachMovementSpace                                                            *
 **************************************************************************************************/
Matrix4f ApproachMovementSpace::getEEFGlobalPose(void)
{
    return eef  -> getRobot() -> getGlobalPose();
}

} // End: namespace GraspStudio