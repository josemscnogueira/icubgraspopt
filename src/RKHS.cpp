/**************************************************************************************************
 *  File:    RKHS.cpp                                                                             *
 *  Author:  Jose Miguel Nogueira, josemscnogueira@gmail.com                                      *
 *                                                                                                *
 *  History:                                                                                      *
 **************************************************************************************************/

/**************************************************************************************************
 *  Include Files                                                                                 *
 **************************************************************************************************/
#include <RKHS.hpp>

#include <chrono>


namespace bayesopt {

/**************************************************************************************************
 *  Procecure                                                                                     *
 *                                                                                                *
 *  Description: Constructor                                                                      *
 *  Class      : RKHS                                                                             *
 **************************************************************************************************/
RKHS::RKHS(void)
{
    ymax        =  7;
    ymin        = -4;
    lower_bound = vectord(1, 0);
    upper_bound = vectord(1, 1);

    // Python path
    setenv("PYTHONPATH", "./function-rkhs/", 1);

    // Initialize Python interface
    Py_Initialize();

    // Create Python Objects
    PyObject*  module_string = PyString_FromString   ((char*)"rkhs");
              _module        = PyImport_Import       (module_string);

    // Module Check
    if (_module != NULL) {_function = PyObject_GetAttrString(_module,(char*)"rkhs_synth"); }
    else                 { PyErr_Print(); exit(-1); }

    // Create Random Number Engine
    _randEngine.seed(std::chrono::system_clock::now().time_since_epoch().count());

    // Update name
    name = std::string("RKHS_1D");
    dim  = 1;

    // Eliminate PyObjects
    Py_DECREF(module_string);
}


/**************************************************************************************************
 *  Procecure                                                                                     *
 *                                                                                                *
 *  Description: Destructor                                                                       *
 *  Class      : RKHS                                                                             *
 **************************************************************************************************/
RKHS::~RKHS(void)
{
    // Eliminate PyObjects
    Py_DECREF(_module);
    Py_DECREF(_function);

    // Finalize Python Interface
    Py_Finalize();
}


/**************************************************************************************************
 *  Procecure                                                                                     *
 *                                                                                                *
 *  Description: evaluate                                                                         *
 *  Class      : RKHS                                                                             *
 **************************************************************************************************/
double RKHS::evaluate(double x)
{
    // Function Argument
    PyObject* args   = PyTuple_Pack(1,PyFloat_FromDouble(x));

    if (args == NULL) { PyErr_Print(); exit(-1); }

    // Call Function
    PyObject* result = PyObject_CallObject(_function, args);

    if (result == NULL) { PyErr_Print(); exit(-1); }

    // Convert to double
    double y = PyFloat_AsDouble(result);

    // Eliminate PyObjects
    Py_DECREF(args);
    Py_DECREF(result);

    return -y;
}


/**************************************************************************************************
 *  Procecure                                                                                     *
 *                                                                                                *
 *  Description: evaluate                                                                         *
 *  Class      : RKHS                                                                             *
 **************************************************************************************************/
double RKHS::evaluate(vectord x)
{
    if (x.size() == 1) return evaluate(x(0));
    else               return 0.0;
}


/**************************************************************************************************
 *  Procecure                                                                                     *
 *                                                                                                *
 *  Description: initSamples                                                                      *
 *  Class      : RKHS                                                                             *
 **************************************************************************************************/
void RKHS::initSamples(vecOfvec& xx, vectord& yy)
{
    // Set XX
    xx.clear();
    xx.push_back(vectord(1, 0.1));
    xx.push_back(vectord(1, 0.5));
    xx.push_back(vectord(1, 0.9));

    yy = vectord(xx.size(), 0);

    for (uint sample = 0; sample < xx.size(); sample += 1)
    {
        yy[sample] = evaluate(xx[sample]);
    }
}


/**************************************************************************************************
 *  Procecure                                                                                     *
 *                                                                                                *
 *  Description: evaluateRandom                                                                   *
 *  Class      : RKHS                                                                             *
 **************************************************************************************************/
void RKHS::evaluateRandom(vecOfvec& xx, vectord& yy, uint number, double min, double max)
{
    if (max <= min) return;

    // Random number variables
    RealDistribution real_distribution    (min, max);
    Generator        real_number_generator(_randEngine, real_distribution);

    // Output variables
    xx.clear();
    yy = vectord(number, 0);

    // Fill outputs
    for(uint index = 0; index < number; index += 1)
    {
        double x = real_number_generator();

        xx.push_back(vectord(1, x));
        yy(index) = evaluate(x);
    }
}

/**************************************************************************************************
 *  Procecure                                                                                     *
 *                                                                                                *
 *  Description: getOptParams                                                                     *
 *  Class      : RKHS                                                                             *
 **************************************************************************************************/
void RKHS::getOptParams(TgpParameters& tgp_params, Parameters& opt_params)
{
    TGPOptimizable::getOptParams(tgp_params, opt_params);

    opt_params.n_iterations      = 45;
    opt_params.n_init_samples    =  5;
    opt_params.crit_params[0]    =  1;    // exp
    opt_params.crit_params[1]    =  0.01; // bias
    opt_params.noise             =  1e-6;
    opt_params.sigma_s           =  3.93;

    tgp_params.min_data_per_leaf =  8;
}

} // End of namespace bayesopt
