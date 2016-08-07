/**************************************************************************************************
 *  File:    iCubOptimizable.hpp                                                                  *
 *  Author:  Jose Miguel Nogueira, josemscnogueira@gmail.com                                      *
 *                                                                                                *
 *  History: @TODO incomplete                                                                     *
 **************************************************************************************************/


#ifndef _ICUBOPTIMIZABLE_HPP_
#define _ICUBOPTIMIZABLE_HPP_

/**************************************************************************************************
 *  Include Files                                                                                 *
 **************************************************************************************************/
#include <iCubGraspQuality.hpp>


/**************************************************************************************************
 *  Class: GraspOptimization                                                                      *
 **************************************************************************************************/
class iCubOptimizable : public TGPOptimizable
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    // Constructors
    iCubOptimizable();

    // Destructors
    ~iCubOptimizable(void) {};
};

#endif // _iCubOptimizable_h_
