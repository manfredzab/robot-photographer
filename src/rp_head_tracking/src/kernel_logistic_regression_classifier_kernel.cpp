/**
 * @copyright  Manfredas Zabarauskas, 2013. All rights reserved. This project is released under
 *             CC BY-NC-SA (Creative Commons Attribution-NonCommercial-ShareAlike) license.
 */
#include <kernel_logistic_regression_classifier_kernel.hpp>

void RPKLRClassifierKernel::buildKernelMatrix(const std::vector<RPTrainingPoint>& training_points)
{
    int training_point_count = training_points.size();
    kernel_matrix = cv::Mat(training_point_count, training_point_count, CV_64F);

    for (int i = 0; i < training_point_count; i++)
    {
        for (int j = 0; j < training_point_count; j++)
        {
            kernel_matrix.at<double>(i, j) = this->k(training_points[i].features, training_points[j].features);
        }
    }
}
