/**
 * @class      RPTrainingPoint
 *
 * @brief      Kernel logistic regression classifier training point.
 *
 * @copyright  Manfredas Zabarauskas, 2013. All rights reserved. This project is released under
 *             CC BY-NC-SA (Creative Commons Attribution-NonCommercial-ShareAlike) license.
 */
#ifndef KERNEL_LOGISTIC_REGRESSION_TRAINING_POINT_HPP_
#define KERNEL_LOGISTIC_REGRESSION_TRAINING_POINT_HPP_

struct RPTrainingPoint
{
    std::vector<double> features;  /**< Vector of features representing a given training point. */
    bool is_positive;              /**< Flag indicating whether a given training point is
                                        positive or negative.                                   */
};

#endif /* KERNEL_LOGISTIC_REGRESSION_TRAINING_POINT_HPP_ */
