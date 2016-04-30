/**************************************************************************************************
 *  File:    OptFuncs.cpp                                                                         *
 *  Author:  Jose Miguel Nogueira, josemscnogueira@gmail.com                                      *
 *                                                                                                *
 *  History:                                                                                      *
 **************************************************************************************************/


/**************************************************************************************************
 *  Include Files                                                                                 *
 **************************************************************************************************/
#include <OptFuncs.hpp>

#include <tgpnode.hpp>
#include <boost/numeric/ublas/assignment.hpp>

namespace bayesopt {

double Martinez::evaluate(vectord x)
{
    if (x.size() != 2) return ymin;

    double result = 0.0;

    x(0) = x(0) * 7 - 1;
    x(1) = x(1) * 8 - 4;

    vectord mu   (2  ); mu    <<= 1.5, 0.0;
    matrixd sigma(2,2); sigma <<= 1.0, 0.0, 0.0, 1.0;
    double  p                   = 0.2;

    result += bayesopt::TGPNode::mgpdf(x,mu,sigma,p);

    mu    <<= 3.0, 3.0;
    sigma <<= 5.0, 0.0, 0.0, 1.0;
    p       = 0.5;

    result += bayesopt::TGPNode::mgpdf(x,mu,sigma,p);

    mu    <<= 4.0, -1.0;
    sigma <<= 2.0, -0.5, 0.5, 4.0;
    p       = 0.5;

    result += bayesopt::TGPNode::mgpdf(x,mu,sigma,p);

    mu    <<= 0.0, -2.0;
    sigma <<= 0.3, 0, 0, 0.3;
    p       = 0.1;

    result += bayesopt::TGPNode::mgpdf(x,mu,sigma,p);

    mu    <<= 2.0, 2.0;
    sigma <<= 10.0, 0, 0, 10.0;
    p       = 0.5;

    result += bayesopt::TGPNode::mgpdf(x,mu,sigma,p);

    mu    <<= 2.0, -1.0;
    sigma <<= 5.0, 0, 0, 5.0;
    p       = 0.5;

    result += bayesopt::TGPNode::mgpdf(x,mu,sigma,p);

    return -result;
}

} // End of namespace bayesopt
