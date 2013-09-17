/**
 * @class      RPKLRClassifierKernel
 *
 * @brief      Kernel logistic regression classifier kernel class.
 *
 * @copyright  Manfredas Zabarauskas, 2013. All rights reserved. This project is released under
 *             CC BY-NC-SA (Creative Commons Attribution-NonCommercial-ShareAlike) license.
 */
#ifndef KERNEL_LOGISTIC_REGRESSION_CLASSIFIER_KERNEL_HPP_
#define KERNEL_LOGISTIC_REGRESSION_CLASSIFIER_KERNEL_HPP_

// STL includes
#include <vector>

// OpenCV includes
#include <opencv2/core/core.hpp>

// RPHeadTracking includes
#include <kernel_logistic_regression_training_point.hpp>

class RPKLRClassifierKernel
{
    public:
        cv::Mat kernel_matrix; /**< Kernel matrix. */

        /**
         * Default virtual destructor.
         */
        virtual ~RPKLRClassifierKernel() {};

        /**
         * Kernel function.
         * @param x First input vector.
         * @param y Second input vector.
         * @returns Kernel function application result.
         */
        virtual double k(const std::vector<double>& x, const std::vector<double>& y) = 0;

        /**
         * Builds kernel matrix.
         * @training_points List of training points.
         */
        void buildKernelMatrix(const std::vector<RPTrainingPoint>& training_points);
};

/**
 * @class  RPKLRClassifierLinearKernel
 * @brief  Kernel logistic regression classifier linear kernel class.
 */
class RPKLRClassifierLinearKernel : public RPKLRClassifierKernel
{
    public:
        virtual double k(const std::vector<double>& x, const std::vector<double>& y)
        {
            double dot_product = 0.0;

            for (unsigned i = 0; i < x.size(); i++)
            {
                dot_product += x[i] * y[i];
            }

            return dot_product;
        }
};

/**
 * @class  RPKLRClassifierGaussianKernel
 * @brief  Kernel logistic regression classifier Gaussian (radial basis function, RBF) kernel class.
 */
class RPKLRClassifierGaussianKernel : public RPKLRClassifierKernel
{
    private:
        double standard_deviation;

    public:
        /**
         * Constructs a Gaussian (RBF) kernel with a given standard deviation.
         * @param standard_deviation  Standard deviation.
         */
        RPKLRClassifierGaussianKernel(double standard_deviation) : standard_deviation(standard_deviation) { };

        virtual double k(const std::vector<double>& x, const std::vector<double>& y)
        {
            double norm_squared = 0.0;
            for (unsigned i = 0; i < x.size(); i++)
            {
                double difference = x[i] - y[i];
                norm_squared += difference * difference;
            }

            return exp(-norm_squared / (2.0 * standard_deviation * standard_deviation));
        }
};

#endif /* KERNEL_LOGISTIC_REGRESSION_CLASSIFIER_KERNEL_HPP_ */
