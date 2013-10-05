// Savitzky-Golay filter
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#include "robotcontrol/model/golay.h"

#include <ros/console.h>

namespace robotcontrol
{

const double GolayCoeff<0, 9>::Coefficients[] = {-21, 14, 39, 54, 59, 54, 39, 14, -21};
const double GolayCoeff<0, 9>::Normalization = 231;

const double GolayCoeff<0, 5>::Coefficients[] = {-3, 12, 17, 12, -3};
const double GolayCoeff<0, 5>::Normalization = 35;

// HACK: This is a cubic approximation!
const double GolayCoeff<1, 7>::Coefficients[] = {22, -67, -58, 0, 58, 67, -22};
const double GolayCoeff<1, 7>::Normalization = 252;

const double GolayCoeff<1, 9>::Coefficients[] = {-4, -3, -2, -1, 0, 1, 2, 3, 4};
const double GolayCoeff<1, 9>::Normalization = 60;

const double GolayCoeff<1, 5>::Coefficients[] = {-2, -1, 0, 1, 2};
const double GolayCoeff<1, 5>::Normalization = 10;

const double GolayCoeff<2, 9>::Coefficients[] = {28, 7, -8, -17, -20, -17, -8, 7, 28};
const double GolayCoeff<2, 9>::Normalization = 462;

const double GolayCoeff<2, 5>::Coefficients[] = {2, -1, -2, -1, 2};
const double GolayCoeff<2, 5>::Normalization = 7;

}
