/**
* This file is part of Simox.
*
* Simox is free software; you can redistribute it and/or modify
* it under the terms of the GNU Lesser General Public License as
* published by the Free Software Foundation; either version 2 of
* the License, or (at your option) any later version.
*
* Simox is distributed in the hope that it will be useful, but
* WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU Lesser General Public License for more details.
*
* You should have received a copy of the GNU Lesser General Public License
* along with this program. If not, see <http://www.gnu.org/licenses/>.
*
* @package    VirtualRobot
* @author     Nikolaus Vahrenkamp
* @copyright  2012 Nikolaus Vahrenkamp
*             GNU Lesser General Public License
*
*/
#ifndef _bayesopt_Common_h_
#define _bayesopt_Common_h_

#include <bayesoptbase.hpp>
#include <bayesopt.hpp>
#include <conditionalbayesprocess.hpp>
#include <criteria_functors.hpp>
#include <dataset.hpp>
#include <dll_stuff.h>
#include <gauss_distribution.hpp>
#include <gaussian_process_hierarchical.hpp>
#include <gaussian_process.hpp>
#include <gaussian_process_ml.hpp>
#include <gaussian_process_normal.hpp>
#include <inneroptimization.hpp>
#include <kernel_functors.hpp>
#include <kernelregressor.hpp>
#include <kernels>
#include <mcmc_sampler.hpp>
#include <mean_atomic.hpp>
#include <mean_combined.hpp>
#include <mean_functors.hpp>
#include <nonparametricprocess.hpp>
#include <optimizable.hpp>
#include <parameters.h>
#include <posterior_empirical.hpp>
#include <posterior_fixed.hpp>
#include <posterior_mcmc.hpp>
#include <posteriormodel.hpp>
#include <prob_distribution.hpp>
#include <randgen.hpp>
#include <specialtypes.hpp>
#include <student_t_distribution.hpp>
#include <student_t_process_jef.hpp>
#include <student_t_process_nig.hpp>

#include <criteria/criteria_a_opt.hpp>
#include <criteria/criteria_combined.hpp>
#include <criteria/criteria_distance.hpp>
#include <criteria/criteria_ei.hpp>
#include <criteria/criteria_expected.hpp>
#include <criteria/criteria_hedge.hpp>
#include <criteria/criteria_lcb.hpp>
#include <criteria/criteria_mi.hpp>
#include <criteria/criteria_poi.hpp>
#include <criteria/criteria_prod.hpp>
#include <criteria/criteria_sum.hpp>
#include <criteria/criteria_thompson.hpp>

#include <kernels/kernel_atomic.hpp>
#include <kernels/kernel_combined.hpp>
#include <kernels/kernel_const.hpp>
#include <kernels/kernel_gaussian.hpp>
#include <kernels/kernel_hamming.hpp>
#include <kernels/kernel_linear.hpp>
#include <kernels/kernel_matern.hpp>
#include <kernels/kernel_polynomial.hpp>
#include <kernels/kernel_prod.hpp>
#include <kernels/kernel_rq.hpp>
#include <kernels/kernel_sum.hpp>



#endif // _bayesopt_Common_h_
