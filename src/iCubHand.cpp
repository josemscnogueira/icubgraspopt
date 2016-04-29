/**************************************************************************************************
 *  File:    iCubHand.cpp                                                                         *
 *  Author:  Jose Miguel Nogueira, josemscnogueira@gmail.com                                      *
 *                                                                                                *
 *  History: 20141203   File Creation.                                                            *
 *           20141213   Hand can be controlled through the physical joints, motor joints or       *
 *                      thorugh a set of pre-trained grasps.                                      *
 *                      Hand now closes for a specific grasp preshape. It checks for collisions.  *
 **************************************************************************************************/

#define DEBUG_MODE

/**************************************************************************************************
 *  Include Files                                                                                 *
 **************************************************************************************************/
#include <iCubHand.h>


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
 *  Namespace: iCub                                                                               *
 **************************************************************************************************/
namespace iCub
{


/**************************************************************************************************
 *  Comparator Function for std::Sort                                                             *
 *                                                                                                *
 *      Description: Used to sort the hand nodes into alphabetic order                            *
 **************************************************************************************************/
struct nodeCompare
{
    bool operator() (const RobotNodePtr &node1, const RobotNodePtr &node2)
    {
        if ( strcmp(node1 -> getName().c_str(), node2 -> getName().c_str()) > 0 )
        {
            return false;
        }
        else
        {
            return true;
        }
    }
};


/**************************************************************************************************
 *  Procecure                                                                                     *
 *                                                                                                *
 *  Description: initializeVariables                                                              *
 *  Class      : iCubHand                                                                         *
 **************************************************************************************************/
void iCubHand::initializeConstVariables()
{
    // Joint Mapping
    joints.maps.actuacted_to_physical << 0.08, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00,  // Index  adduction/abduction
                                         0.00, 0.00, 0.00, 0.00, 1.00, 0.00, 0.00, 0.00, 0.00,  // Index  proximal
                                         0.00, 0.00, 0.00, 0.00, 0.00, 0.60, 0.00, 0.00, 0.00,  // Index  distal 1
                                         0.00, 0.00, 0.00, 0.00, 0.00, 0.40, 0.00, 0.00, 0.00,  // Index  distal 2

                                         0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00,  // Middle adduction/abduction
                                         0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 1.00, 0.00, 0.00,  // Middle proximal
                                         0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.60, 0.00,  // Middle distal 1
                                         0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.40, 0.00,  // Middle distal 2

                                        -0.10, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00,  // Pinky  adduction/abduction
                                         0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 1.00,  // Pinky  proximal
                                         0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.90,  // Pinky  distal 1
                                         0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.60,  // Pinky  distal 2

                                        -0.05, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00,  // Ring   adduction/abduction
                                         0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 1.00,  // Ring   proximal
                                         0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.90,  // Ring   distal 1
                                         0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.60,  // Ring   distal 2

                                         0.00, 1.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00,  // Thumb  opposition
                                         0.00, 0.00, 1.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00,  // Thumb  proximal
                                         0.00, 0.00, 0.00, 0.60, 0.00, 0.00, 0.00, 0.00, 0.00,  // Thumb  distal 1
                                         0.00, 0.00, 0.00, 0.40, 0.00, 0.00, 0.00, 0.00, 0.00;  // Thumb  distal 2

    joints.offsets << 0.00,  // Index  adduction/abduction
                      0.00,  // Index  proximal
                      0.00,  // Index  distal 1
                      0.00,  // Index  distal 2

                      0.00,  // Middle adduction/abduction
                      0.00,  // Middle proximal
                      0.00,  // Middle distal 1
                      0.00,  // Middle distal 2

                      0.00,  // Pinky  adduction/abduction
                      0.00,  // Pinky  proximal
                      0.00,  // Pinky  distal 1
                      0.00,  // Pinky  distal 2

                      0.05,  // Ring   adduction/abduction
                      0.00,  // Ring   proximal
                      0.00,  // Ring   distal 1
                      0.00,  // Ring   distal 2

                      0.00,  // Thumb  opposition
                      0.00,  // Thumb  proximal
                      0.00,  // Thumb  distal 1
                      0.00;  // Thumb  distal 2

    grasps.resize(NUMBER_OF_GRASPS);

    // Grasp 1 : Tripod
    grasps[0].mean <<  26.2886,  59.7419,  37.8144,  24.3002,  11.6116,  69.9347,  12.0496,  83.6492,  0.0000; // Transposed

    grasps[0].map  <<   0.5458,   0.0099,   0.0862,   0.1861,   0.2184,  -0.2744,   0.6946,  -0.2335,   0.0000,
                        0.3584,   0.2236,   0.2568,   0.3497,  -0.2268,   0.6150,   0.0545,   0.4480,   0.0000,
                       -0.1426,   0.1110,   0.1195,   0.7022,  -0.4674,  -0.3442,  -0.1823,  -0.2998,   0.0000,
                       -0.2864,   0.3238,  -0.1791,   0.4689,   0.7388,   0.0830,  -0.0585,   0.0713,   0.0000,
                       -0.6382,   0.1947,   0.0758,  -0.0677,  -0.2407,  -0.0078,   0.6579,   0.2317,   0.0000,
                       -0.2105,-  0.6421,   0.6710,   0.1566,   0.2538,   0.0495,   0.0033,   0.0421,   0.0000,
                       -0.0247,-  0.6180,  -0.6514,   0.3159,  -0.1160,   0.1692,   0.1757,   0.1427,   0.0000,
                       -0.1383,   0.0231,  -0.0011,  -0.0368,  -0.0283,   0.6245,   0.1192,  -0.7576,   0.0000,
                        0.0000,   0.0000,   0.0000,   0.0000,   0.0000,   0.0000,   0.0000,   0.0000,   0.0000;

    grasps[0].components.primary   = 0;
    grasps[0].components.auxiliary = 1;

    // Grasp 2 : Palmar Pinch
    grasps[1].mean <<  32.3464,  67.4556,  35.4004,   3.6400,  46.4911,  12.3593,   0.0000,   0.0000,  0.0000; // Transposed

    grasps[1].map  <<  -0.2996,  -0.2127,  -0.6816,  -0.1524,  -0.5952,  -0.1514,   0.0000,   0.0000,  0.0000, // Closure
                        0.1449,  -0.1581,  -0.4946,   0.5231,   0.2618,   0.6060,   0.0000,   0.0000,  0.0000,
                       -0.0962,   0.0343,   0.4354,   0.0317,  -0.6314,   0.6327,   0.0000,   0.0000,  0.0000,
                       -0.1833,  -0.4333,   0.2812,   0.7209,  -0.1595,  -0.3931,   0.0000,   0.0000,  0.0000,
                       -0.8396,  -0.2320,   0.0972,  -0.1882,   0.3877,   0.2144,   0.0000,   0.0000,  0.0000,
                       -0.3761,   0.8289,  -0.1130,   0.3835,  -0.0514,  -0.0949,   0.0000,   0.0000,  0.0000,
                        0.0000,   0.0000,   0.0000,   0.0000,   0.0000,   0.0000,   0.0000,   0.0000,  0.0000,
                        0.0000,   0.0000,   0.0000,   0.0000,   0.0000,   0.0000,   0.0000,   0.0000,  0.0000,
                        0.0000,   0.0000,   0.0000,   0.0000,   0.0000,   0.0000,   0.0000,   0.0000,  0.0000;

    grasps[1].components.primary   = 0;
    grasps[1].components.auxiliary = 5;

    // Grasp 3 : Lateral
    grasps[2].mean <<  38.0536,  12.7450,  31.3871,  38.4039,  29.5085, 127.4811,   0.0000,   0.0000,  0.0000; // Transposed

    grasps[2].map  <<  -0.1161,   0.0097,  -0.0330,   0.2252,  -0.2050,   0.9448,   0.0000,   0.0000,  0.0000,
                       -0.4299,  -0.0654,   0.0155,  -0.8425,  -0.3066,   0.0826,   0.0000,   0.0000,  0.0000,
                       -0.8566,   0.2130,   0.0168,   0.2784,   0.3664,  -0.0937,   0.0000,   0.0000,  0.0000,
                       -0.2333,  -0.2016,   0.0207,   0.4007,  -0.8098,  -0.2971,   0.0000,   0.0000,  0.0000,
                       -0.1073,  -0.9358,   0.1908,   0.0383,   0.2685,   0.0523,   0.0000,   0.0000,  0.0000, // Closure
                       -0.0434,  -0.1840,  -0.9806,  -0.0002,   0.0435,  -0.0282,   0.0000,   0.0000,  0.0000,
                       -0.0000,   0.0000,   0.0000,   0.0000,   0.0000,   0.0000,   0.0000,   0.0000,  0.0000,
                       -0.0000,   0.0000,   0.0000,   0.0000,   0.0000,   0.0000,   0.0000,   0.0000,  0.0000,
                       -0.0000,   0.0000,   0.0000,   0.0000,   0.0000,   0.0000,   0.0000,   0.0000,  0.0000;

    grasps[2].components.primary   = 0;
    grasps[2].components.auxiliary = 5;

    // Grasp 4 : Writing Tripod
    grasps[3].mean  <<  33.4857,  58.4530,  47.2315,  26.0594,  39.0467,  48.7596,  39.9037,  73.3922,  0.0000; // Transposed

    grasps[3].map  <<    0.7181,   0.0926,  -0.0667,   0.1009,  -0.3764,   0.2816,   0.2028,  -0.4461,  0.0000,
                        -0.1938,  -0.1550,  -0.2216,   0.5430,  -0.0912,  -0.4637,  -0.2899,  -0.5358,  0.0000,
                        -0.4543,  -0.1902,   0.3255,  -0.2586,   0.0667,   0.2334,   0.3981,  -0.6060,  0.0000,
                        -0.2400,   0.5103,   0.1756,   0.6786,   0.0574,   0.3340,   0.2471,   0.1217,  0.0000,
                        -0.1739,   0.1193,   0.5204,  -0.0885,  -0.7213,  -0.0144,  -0.3893,   0.0695,  0.0000,
                        -0.0500,  -0.7222,  -0.0284,   0.3222,  -0.3318,   0.0890,   0.3635,   0.3480,  0.0000,
                        -0.3306,   0.3292,  -0.6170,  -0.2366,  -0.4601,  -0.1239,   0.3419,   0.0403,  0.0000,
                         0.2018,   0.1632,   0.3968,   0.0031,   0.0124,  -0.7184,   0.5046,   0.0655,  0.0000,
                         0.0000,   0.0000,   0.0000,   0.0000,   0.0000,   0.0000,   0.0000,   0.0000,  0.0000;

    grasps[3].components.primary   = 0;
    grasps[3].components.auxiliary = 7;

    // Grasp 5 : Parallel Extension
    grasps[4].mean <<  44.9143,  73.4473,  48.3727,   3.1020,  44.2198,   7.0201,  41.7612,   6.0629,  0.0000; // Transposed

    grasps[4].map  <<   0.1508,   0.0493,  -0.3222,  -0.1610,   0.6771,  -0.0001,   0.6207,   0.0367,  0.0000,
                        0.0733,  -0.0026,  -0.9232,  -0.0092,  -0.2431,  -0.1484,  -0.2284,  -0.0946,  0.0000,
                       -0.0261,   0.4457,  -0.0415,  -0.6297,  -0.1579,   0.6126,  -0.0400,  -0.0248,  0.0000,
                        0.0396,  -0.0478,   0.1669,  -0.5828,   0.0027,  -0.5753,  -0.0413,  -0.5440,  0.0000,
                       -0.3159,  -0.5040,  -0.0491,  -0.4712,   0.0076,  -0.1117,  -0.0768,   0.6351,  0.0000,
                       -0.6709,  -0.3757,  -0.1012,   0.0879,  -0.0246,   0.3162,   0.2191,  -0.4921,  0.0000,
                       -0.6471,   0.6296,  -0.0381,   0.0899,   0.1246,  -0.3590,  -0.0354,   0.1726,  0.0000,
                       -0.0332,  -0.0713,  -0.0160,   0.0051,   0.6643,   0.1742,  -0.7100,  -0.1336,  0.0000,
                       -0.6500,   0.0000,   0.0000,   0.0000,   0.0000,   0.0000,   0.0000,   0.0000,  0.0000;

    grasps[4].components.primary   = 0;
    grasps[4].components.auxiliary = 4;


    // Grasp 6 : Adduction Grip
    grasps[5].mean <<  28.7286,   0.0000,   0.0000,   0.0000,  18.3773,  52.9301,  19.3645,  50.8644,  0.0000; // Transposed

    grasps[5].map  <<   0.7086,   0.0723,   0.0047,  -0.7007,  -0.0409,   0.0000,    0.0000,   0.0000,   0.0000,
                        0.2203,   0.4133,   0.5164,   0.2292,   0.6792,   0.0000,    0.0000,   0.0000,   0.0000,
                       -0.4335,   0.7285,   0.2102,  -0.3415,  -0.3473,   0.0000,    0.0000,   0.0000,   0.0000,
                       -0.3900,  -0.5317,   0.5744,  -0.4552,   0.1670,   0.0000,    0.0000,   0.0000,   0.0000,
                       -0.3306,   0.1026,  -0.5992,  -0.3642,   0.6233,   0.0000,    0.0000,   0.0000,   0.0000,
                        0.0000,   0.0000,   0.0000,   0.0000,   0.0000,   0.0000,    0.0000,   0.0000,   0.0000,
                        0.0000,   0.0000,   0.0000,   0.0000,   0.0000,   0.0000,    0.0000,   0.0000,   0.0000,
                        0.0000,   0.0000,   0.0000,   0.0000,   0.0000,   0.0000,    0.0000,   0.0000,   0.0000,
                        0.0000,   0.0000,   0.0000,   0.0000,   0.0000,   0.0000,    0.0000,   0.0000,   0.0000;

    grasps[5].components.primary   = 0;
    grasps[5].components.auxiliary = 4;

    // Grasp 7 : Tip Pinch
    grasps[6].mean <<  33.8421,  55.4833,  33.0964,  29.5443,  14.6397,  87.6561,   0.0000,   0.0000,  0.0000; // Transposed

    grasps[6].map  <<  -0.6235,   0.0736,   0.0282,   0.2316,   0.6905,  -0.2731,   0.0000,   0.0000,  0.0000,
                        0.2808,  -0.2672,  -0.4147,  -0.4921,   0.2176,  -0.6231,   0.0000,   0.0000,  0.0000,
                       -0.5915,  -0.0134,   0.0626,  -0.1271,  -0.6644,  -0.4342,   0.0000,   0.0000,  0.0000,
                        0.0751,  -0.1162,  -0.6427,   0.7207,  -0.1834,  -0.1217,   0.0000,   0.0000,  0.0000,
                       -0.3978,   0.0207,  -0.6139,  -0.4062,   0.0162,   0.5469,   0.0000,   0.0000,  0.0000,
                        0.1363,   0.9535,  -0.1825,  -0.0610,  -0.0243,  -0.1863,   0.0000,   0.0000,  0.0000,
                        0.0000,   0.0000,   0.0000,   0.0000,   0.0000,   0.0000,   0.0000,   0.0000,  0.0000,
                        0.0000,   0.0000,   0.0000,   0.0000,   0.0000,   0.0000,   0.0000,   0.0000,  0.0000,
                        0.0000,   0.0000,   0.0000,   0.0000,   0.0000,   0.0000,   0.0000,   0.0000,  0.0000;

    grasps[6].components.primary   = 0;
    grasps[6].components.auxiliary = 1;

    // Grasp 8 : Lateral Tripod
    grasps[7].mean <<  31.2885,  30.9818,  37.8362,  34.7004,  18.3315,  86.3367,  35.3239, 118.8301,  0.0000; // Transposed

    grasps[7].map  <<   0.0398,   0.0326,  -0.0827,   0.4921,  -0.1819,   0.6192,  -0.0567,   0.5733,  0.0000,
                        0.4528,  -0.1938,   0.0603,   0.3705,  -0.2579,   0.2550,  -0.0644,  -0.6934,  0.0000,
                       -0.7332,   0.0529,   0.4532,   0.4329,   0.0558,  -0.0084,  -0.0790,  -0.2394,  0.0000,
                       -0.1119,   0.1455,  -0.2147,  -0.0257,   0.4346,   0.3969,   0.7216,  -0.2289,  0.0000,
                       -0.3447,   0.0866,  -0.1758,  -0.5550,  -0.2742,   0.5360,  -0.3456,  -0.2300,  0.0000,
                       -0.2670,   0.1149,  -0.8033,   0.3242,  -0.2332,  -0.3068,  -0.0231,  -0.1271,  0.0000,
                       -0.2303,  -0.8821,  -0.0429,  -0.0962,  -0.2374,  -0.0029,   0.3023,   0.1002,  0.0000,
                       -0.0029,   0.3723,   0.2451,  -0.1006,  -0.7228,  -0.1122,   0.5043,   0.0424,  0.0000,
                        0.0000,   0.0000,   0.0000,   0.0000,   0.0000,   0.0000,   0.0000,   0.0000,  0.0000;

    grasps[7].components.primary   = 0;
    grasps[7].components.auxiliary = 1;

    // Grasp 9 : PowerGrip
    grasps[8].mean <<  33.5561,  51.8757,  10.0000,   5.0000,   5.0000,   5.0000,   5.0000,   5.0000,  5.0000; // Transposed

    grasps[8].map  <<  -0.2000,   0.0000,   0.0000,   0.0000,   0.0000,   0.0000,   0.0000,   0.0000,  0.0000,
                        0.0000,  -0.9232,   0.0000,   0.0000,   0.0000,   0.0000,   0.0000,   0.0000,  0.0000,
                       -0.0500,  -0.0415,   0.0000,   0.0000,   0.0000,   0.0000,   0.0000,   0.0000,  0.0000,
                       -0.9000,   0.1669,   0.0000,   0.0000,   0.0000,   0.0000,   0.0000,   0.0000,  0.0000,
                       -0.3500,  -0.0491,   0.0000,   0.0000,   0.0000,   0.0000,   0.0000,   0.0000,  0.0000,
                       -0.4000,  -0.1012,   0.0000,   0.0000,   0.0000,   0.0000,   0.0000,   0.0000,  0.0000,
                       -0.3500,  -0.0381,   0.0000,   0.0000,   0.0000,   0.0000,   0.0000,   0.0000,  0.0000,
                       -0.4000,  -0.0160,   0.0000,   0.0000,   0.0000,   0.0000,   0.0000,   0.0000,  0.0000,
                       -0.3200,   0.0000,   0.0000,   0.0000,   0.0000,   0.0000,   0.0000,   0.0000,  0.0000;

    grasps[8].components.primary   = 0;
    grasps[8].components.auxiliary = 1;

    // Convert mean from degrees to radians
    for (int index = 0; index < NUMBER_OF_GRASPS; index++)
    {
        grasps[index].mean *=   M_PI  / 180.000f;
    }

    // Actor to Joint Map
    actor_to_joint_mask.resize(5);

    actor_to_joint_mask[0] = 0b00000000000000001111;
    actor_to_joint_mask[1] = 0b11110000000000000000;
    actor_to_joint_mask[2] = 0b00001111000000000000;
    actor_to_joint_mask[3] = 0b00000000000011110000;
    actor_to_joint_mask[4] = 0b00000000111100000000;
}


/**************************************************************************************************
 *  Procecure                                                                                     *
 *                                                                                                *
 *  Description: Constructor                                                                      *
 *  Class      : iCubHand                                                                         *
 **************************************************************************************************/
iCubHand::iCubHand(void)
{
    initializeConstVariables();

    allow_object_trespass     = false;
    grasp_quality_initialized = false;

    verbose                   = PRINT_NOTHING;

    state                     = OPENED;
}

/**************************************************************************************************
 *  Procecure                                                                                     *
 *                                                                                                *
 *  Description: Constructor                                                                      *
 *  Class      : iCubHand                                                                         *
 **************************************************************************************************/
iCubHand::iCubHand(RobotPtr robot) : iCubHand()
{
    loadHand(robot);
}


/**************************************************************************************************
 *  Procecure                                                                                     *
 *                                                                                                *
 *  Description: updateHandNodes                                                                  *
 *  Class      : iCubHand                                                                         *
 **************************************************************************************************/
void iCubHand::loadHand(RobotPtr robot)
{
    // Load EndEffector from Robot
    endeffector = robot -> getEndEffector("Left Hand GraspOpt");

    vector<RobotNodePtr> nodes = endeffector -> getAllNodes();

    // Update hand nodes
    updateHandNodes();
}



/**************************************************************************************************
 *  Procecure                                                                                     *
 *                                                                                                *
 *  Description: updateHandNodes                                                                  *
 *  Class      : iCubHand                                                                         *
 **************************************************************************************************/
void iCubHand::updateHandNodes(void)
{
    // Return if there is no EndEffector
    if(!endeffector) return;

    // Update Nodes
    nodes = endeffector -> getAllNodes();

    // Remove those who don't corresponde to actuators
    for(vector<RobotNodePtr>::iterator node = nodes.begin(); node != nodes.end(); )
    {
        if ((*node) -> isTranslationalJoint() || (*node) -> isRotationalJoint())
        {
            node++;
        }
        else
        {
            nodes.erase(node);
        }
    }

    // Sort the nodes
    sort(nodes.begin(), nodes.end(), nodeCompare());

    // Update Joint Values
    getJointValues();

    // Print
    if (verbose == PRINT_HAND_NODES)
    {
        cout << endl << "Number of hand nodes: " << nodes.size() << endl;

        for(vector<RobotNodePtr>::iterator node = nodes.begin(); node != nodes.end(); node++) cout << (*node) -> getName() << endl;


        vector<EndEffectorActorPtr> actors;
        endeffector -> getActors   (actors);

        cout << "Number of Actors: " << actors.size();
        for (uint index = 0; index < actors.size(); index += 1) actors[index] -> print();
    }
}


/**************************************************************************************************
 *  Procecure                                                                                     *
 *                                                                                                *
 *  Description: getJointValues                                                                   *
 *  Class      : iCubHand                                                                         *
 **************************************************************************************************/
void iCubHand::getJointValues(void)
{
    for(uint index = 0; index < nodes.size(); ++index)
    {
        joints.values(index, 0) = nodes[index] -> getJointValue();
    }
}


/**************************************************************************************************
 *  Procecure                                                                                     *
 *                                                                                                *
 *  Description: getJointValues                                                                   *
 *  Class      : iCubHand                                                                         *
 **************************************************************************************************/
void iCubHand::setJointValues(void)
{
    for(uint index = 0; index < nodes.size(); ++index)
    {
        nodes[index] -> setJointValue(joints.values(index, 0));
    }
}


/**************************************************************************************************
 *  Procecure                                                                                     *
 *                                                                                                *
 *  Description: controlHand                                                                      *
 *  Class      : iCubRobot                                                                        *
 **************************************************************************************************/
void iCubHand::controlHand(Vector09dof actuated_joints, int mode)
{
    // Return if there is no hand
    if (actuated_joints.size() != 9) return;

    // Update Joint Values
    switch (mode)
    {
        case TRYPOD             :
        case PALMAR_PINCH       :
        case LATERAL            :
        case WRITING_TRIPOD     :
        case PARALLEL_EXTENSION :
        case ADDUCTION_GRIP     :
        case TIP_PINCH          :
        case LATERAL_TRIPOD     :
        case POWER_GRIP         : joints.values =                  joints.maps.actuacted_to_physical * (grasps[mode - 1].map * actuated_joints + grasps[mode - 1].mean); break;
        case JOINTS_MODE        : joints.values = joints.offsets + joints.maps.actuacted_to_physical * (                       actuated_joints);                         break;
    }

    // Update Nodes
    setJointValues();
}


/**************************************************************************************************
 *  Procecure                                                                                     *
 *                                                                                                *
 *  Description: closeGrasp                                                                       *
 *  Class      : iCubRobot                                                                        *
 **************************************************************************************************/
GraspQualityMeasureWrenchSpacePtr iCubHand::closeGrasp(uint grasp, Vector09dof auxiliary_components, SceneObjectSetPtr obstacles, uint &number_of_contacts, float step, float open_offeset)
{
    Vector09dof                        component_values;
    Vector20dof                        joints_prev;
    uint32_t                           active_mask;

    // Clear Contacts
    contacts.clear();

    // Calculate Component Values
    component_values[grasps[grasp].components.primary] = M_PI - open_offeset;

    for (int index = 0; index < auxiliary_components.size(); index += 1)
    {
        if (index != grasps[grasp].components.primary)
        {
            component_values[index] = auxiliary_components[index];
        }
    }

    // Set the Initial Joint State
    joints.values = joints.maps.actuacted_to_physical * (grasps[grasp].map * component_values + grasps[grasp].mean);
    joints_prev   = joints.values;
    active_mask   = 0b11111111111111111111;

    // Check for initial collisions
    checkIfColides    (active_mask, obstacles);
    checkIfSelfColides(active_mask           );

    // Close the hand using the most energetic component -> primary
    while ( (active_mask != 0) && (component_values[grasps[grasp].components.primary] > - M_PI) )
    {
        uint32_t mask_select;

        // Update previous joint values
        joints_prev = joints.values;

        // Update the primary component grasp with the specified step
        component_values[grasps[grasp].components.primary] -= step;

        // Calculate the joint values
        joints.values = joints.maps.actuacted_to_physical * (grasps[grasp].map * component_values + grasps[grasp].mean);

        // Update the joint values that are colliding
        mask_select = 0b10000000000000000000;

        for (uint index = 0; index < 20; index += 1)
        {
            if ( (mask_select & active_mask) == 0 )
            {
                joints.values(index, 0) = joints_prev(index, 0);
            }

            mask_select = mask_select >> 1;
        }

        // Update Nodes
        setJointValues();

        // Check for collisions
        checkIfColides    (active_mask, obstacles);
        checkIfSelfColides(active_mask           );

        // Update the joint values that are colliding
        mask_select = 0b10000000000000000000;

        for (uint index = 0; index < 20; index += 1)
        {
            if ( (mask_select & active_mask) == 0 )
            {
                joints.values(index, 0) = joints_prev(index, 0);
            }

            mask_select = mask_select >> 1;
        }
    }

    // Update Nodes
    setJointValues();

    // Check the number of contacts
    contacts = getContacts(obstacles);

    number_of_contacts = contacts.size();

    if(number_of_contacts > 1)
    {

        SceneObjectPtr lego = obstacles -> getSceneObject(0);

        // Set the Grasp Quality Measure
        if ( !grasp_quality_initialized )
        {
            grasp_quality.reset(new GraspQualityMeasureWrenchSpace(lego));
            grasp_quality -> calculateObjectProperties();

            grasp_quality_initialized = true;
        }

        // Get Contact Points
        grasp_quality -> setContactPoints(contacts);

        // Calculate Grasp Quality
        grasp_quality -> calculateGraspQuality();

        // Print
        // setVerbose(PRINT_GRASP_RESULT);

        if ( verbose == PRINT_GRASP_RESULT )
        {
            if ( grasp_quality -> isGraspForceClosure() ) cout << endl << "The Grasp is force closure."    ;
            else                                          cout << endl << "The Grasp is non force closure.";

            cout     << endl << "Volume: "             << grasp_quality -> getVolumeGraspMeasure();
            cout     << endl << "Quality measure: "    << grasp_quality -> getGraspQuality()      ;
        }
    }

    // Update Hand State
    state = CLOSED;

    return grasp_quality;
}

/**************************************************************************************************
 *  Procecure                                                                                     *
 *                                                                                                *
 *  Description: closeGrasp                                                                       *
 *  Class      : iCubRobot                                                                        *
 **************************************************************************************************/
void iCubHand::openGrasp(uint grasp, Vector09dof auxiliary_components, float open_offeset)
{
    Vector09dof                        component_values;

    // Calculate Component Values
    component_values[grasps[grasp].components.primary] = M_PI - open_offeset;

    for (int index = 0; index < auxiliary_components.size(); index += 1)
    {
        if (index != grasps[grasp].components.primary)
        {
            component_values[index] = auxiliary_components[index];
        }
    }

    // Set the Initial Joint State
    joints.values = joints.maps.actuacted_to_physical * (grasps[grasp].map * component_values + grasps[grasp].mean);

    setJointValues();

    //Update Hand State
    state = OPENED;
}


/**************************************************************************************************
 *  Procecure                                                                                     *
 *                                                                                                *
 *  Description: checkIfSelfColides                                                               *
 *  Class      : iCubRobot                                                                        *
 **************************************************************************************************/
bool iCubHand::checkIfSelfColides(uint32_t &active_mask)
{
    bool                        collision = false;
    vector<EndEffectorActorPtr> actors;

    // Get Actors
    endeffector -> getActors(actors);

    // Check every combination of actors
    for (    uint index1 =          0; index1 < (actors.size() - 1); index1 += 1)
    {
        for (uint index2 = index1 + 1; index2 < (actors.size() - 0); index2 += 1)
        {
            if ( actors[index1] -> isColliding(actors[index2]) )
            {
                active_mask &= ~actor_to_joint_mask[index1];
                active_mask &= ~actor_to_joint_mask[index2];

                collision = true;
            }
        }
    }

    return collision;
}


/**************************************************************************************************
 *  Procecure                                                                                     *
 *                                                                                                *
 *  Description: checkIfSelfColides                                                               *
 *  Class      : iCubRobot                                                                        *
 **************************************************************************************************/
bool iCubHand::checkIfColides(uint32_t &active_mask, SceneObjectSetPtr obstacles)
{
    bool                        collision  = false;
    vector<EndEffectorActorPtr> actors;

    // Get Actors
    endeffector -> getActors(actors);

    // Check every actor with the environment
    for(uint index = 0; index < actors.size(); index += 1)
    {
        if ( actors[index] -> isColliding(obstacles))
        {
            active_mask &= ~actor_to_joint_mask[index];

            collision    =  true;
        }
    }

    return collision;
}


/**************************************************************************************************
 *  Procecure                                                                                     *
 *                                                                                                *
 *  Description: getContacts                                                               *
 *  Class      : iCubRobot                                                                        *
 **************************************************************************************************/
EndEffector::ContactInfoVector iCubHand::getContacts(SceneObjectSetPtr obstacles, float step)
{
    vector<EndEffectorActorPtr>                 actors;
    EndEffector::ContactInfoVector              result;
                                                result.clear();
    // Get Actors
    endeffector -> getActors(actors);

    // Get Contact Points for each actor
    for (uint index = 0; index < actors.size(); index += 1)
    {
        actors[index] -> moveActorCheckCollision(endeffector, result, obstacles, step);
    }

    return result;
}


/**************************************************************************************************
 *  Procecure                                                                                     *
 *                                                                                                *
 *  Description: getState                                                                         *
 *  Class      : iCubRobot                                                                        *
 **************************************************************************************************/
uint iCubHand::getState(void)
{
    return state;
}


/**************************************************************************************************
 *  Procecure                                                                                     *
 *                                                                                                *
 *  Description: setVerbose                                                                       *
 *  Class      : iCubRobot                                                                        *
 **************************************************************************************************/
void iCubHand::setVerbose(uint32_t verbose)
{
    this -> verbose = verbose;
}


/**************************************************************************************************
 *  Procecure                                                                                     *
 *                                                                                                *
 *  Description: getEEF                                                                           *
 *  Class      : iCubRobot                                                                        *
 **************************************************************************************************/
EndEffectorPtr iCubHand::getEEF(void)
{
    return endeffector;
}


/**************************************************************************************************
 *  Procecure                                                                                     *
 *                                                                                                *
 *  Description: getNumberOfAuxiliaryGraspParameters                                              *
 *  Class      : iCubHand                                                                         *
 **************************************************************************************************/
uint iCubHand::getNumberOfAuxiliaryGraspParameters(uint mode)
{
    if(mode < 0)
    {
        cout << endl << endl << "[ERROR] iCubHand::Invalid Grasp in getNumberOfAuxiliaryGraspParameters." << endl;

        exit(-1);
    }

    return grasps[mode].components.auxiliary;
}


/**************************************************************************************************
 *  Procecure                                                                                     *
 *                                                                                                *
 *  Description: getContactPoints                                                                 *
 *  Class      : iCubHand                                                                         *
 **************************************************************************************************/
EndEffector::ContactInfoVector iCubHand::getContactPoints(void)
{
    return contacts;
}


/**************************************************************************************************
 *  Procecure                                                                                     *
 *                                                                                                *
 *  Description: calculateNumberOfDimensionsForLearning                                           *
 *  Class      : iCubHand                                                                         *
 **************************************************************************************************/
uint iCubHand::calculateNumberOfDimensionsForLearning(uint mode)
{
    return (getNumberOfAuxiliaryGraspParameters(mode) + 3 + 3);
}


/**************************************************************************************************
 *  Procecure                                                                                     *
 *                                                                                                *
 *  Description: getEEFGlobalPose                                                                 *
 *  Class      : iCubHand                                                                         *
 **************************************************************************************************/
Matrix4f iCubHand::getEEFGlobalPose(void)
{
    return endeffector -> getRobot() -> getGlobalPose();
}


/**************************************************************************************************
 *  Procecure                                                                                     *
 *                                                                                                *
 *  Description: getEEFJointConfig                                                                *
 *  Class      : iCubHand                                                                         *
 **************************************************************************************************/
RobotConfigPtr iCubHand::getEEFJointConfig(void)
{
    return endeffector -> getConfiguration();
}


/**************************************************************************************************
 *  Procecure                                                                                     *
 *                                                                                                *
 *  Description: printEEFState                                                                    *
 *  Class      : iCubHand                                                                         *
 **************************************************************************************************/
void iCubHand::printEEFState()
{
    cout << endl;
    cout << endl << "                     >>>>> End Effector State <<<<<                        ";

    cout << endl << "***************************************************************************";
    cout << endl << " >>> Global pose";
    cout << endl << "***************************************************************************";
    cout << endl << getEEFGlobalPose();

    cout << endl << "***************************************************************************";
    cout << endl << " >>> EEF Config";
    cout << endl << "***************************************************************************";
    cout << endl << getEEFJointConfig() -> toXML();

    printContactInfoVector(contacts);
}


/**************************************************************************************************
 *  Procecure                                                                                     *
 *                                                                                                *
 *  Description: contactInfoToString                                                              *
 *  Class      : iCubHand                                                                         *
 **************************************************************************************************/
void iCubHand::printContactInfo(EndEffector::ContactInfo contact)
{
    cout << endl << " >>> Finger      = "   << contact.actor -> getName();
    cout << endl << " >>> Distance    = "   << contact.distance;
    cout << endl << " >>> Direction   = ( " << contact.approachDirectionGlobal   (0) << " , "
                                            << contact.approachDirectionGlobal   (1) << " , "
                                            << contact.approachDirectionGlobal   (2) << ")";
    cout << endl << " >>> PointFinger = ( " << contact.contactPointFingerGlobal  (0) << " , "
                                            << contact.contactPointFingerGlobal  (1) << " , "
                                            << contact.contactPointFingerGlobal  (2) << ")";
    cout << endl <<" >>> PointObject = ( "  << contact.contactPointObstacleGlobal(0) << " , "
                                            << contact.contactPointObstacleGlobal(1) << " , "
                                            << contact.contactPointObstacleGlobal(2) << ")";
}

/**************************************************************************************************
 *  Procecure                                                                                     *
 *                                                                                                *
 *  Description: contactInfoToString                                                              *
 *  Class      : iCubHand                                                                         *
 **************************************************************************************************/
void iCubHand::printContactInfoVector(EndEffector::ContactInfoVector contacts)
{
    cout << endl << "***************************************************************************";
    cout << endl << " >>> Contacts";
    cout << endl << "***************************************************************************";

    for(uint index = 0; index < contacts.size(); index += 1)
    {
        cout << endl << "ContactInfo[" << index << "]";

        printContactInfo(contacts[index]);

        cout << endl << "*************************************";
    }
}

} // End: namespace iCub
