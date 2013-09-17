/**
 * @class      RPKernelLogisticRegressionClassifier
 *
 * @brief      Kernel logistic regression classifier class.
 *
 * @copyright  Manfredas Zabarauskas, 2013. All rights reserved. This project is released under
 *             CC BY-NC-SA (Creative Commons Attribution-NonCommercial-ShareAlike) license.
 */
#ifndef KERNEL_LOGISTIC_REGRESSION_CLASSIFIER_HPP_
#define KERNEL_LOGISTIC_REGRESSION_CLASSIFIER_HPP_

// C++ includes
#include <cmath>

// STL includes
#include <vector>

// RPHeadTracking includes
#include <kernel_logistic_regression_classifier_kernel.hpp>
#include <kernel_logistic_regression_training_point.hpp>

// Constants
#define ETA 0.1 // Gradient step size
#define GAMMA 100.0 // Gaussian prior standard deviation
#define DELTA 0.5 // Gaussian kernel size

class RPKernelLogisticRegressionClassifier
{
    private:
        std::vector<RPTrainingPoint> training_points;  /**< Training points.                     */

        std::vector<double> weights;                   /**< Current weights.                     */
        std::vector<double> gradient;                  /**< Current gradient.                    */
        std::vector<double> step_sizes;                /**< Current step sizes.                  */

        RPKLRClassifierGaussianKernel kernel;          /**< Classifier's kernel function.        */

        /**
         * Calculates sigmoid function.
         * @param z Input for sigmoid function.
         * @returns Sigmoid function applied to the input.
         */
        double sigmoid(double z)
        {
            return 1.0 / (1.0 + exp(-z));
        }

        /**
         * Calculates sign function.
         * @param z Input for sign function.
         * @returns Sign function applied to the input.
         */
        int sgn(double z)
        {
            return (0.0 < z) - (z < 0.0);
        }

        /**
         * Performs a single gradient training step using an RProp variance algorithm.
         */
        double gradientStep();

    public:
        /**
         * Default kernel logistic regression classifier's constructor.
         */
        RPKernelLogisticRegressionClassifier() : kernel(DELTA) { };

        /**
         * Adds a training sample.
         * @param features     Input training sample.
         * @param is_positive  Training sample positive/negative flag.
         */
        void addTrainingSample(std::vector<double>& features, bool is_positive);

        /**
         * Trains the KLR classifier using an RProp variance algorithm for a given period of time.
         * @param time_limit_s  Training time limit in secods.
         */
        void train(double time_limit_s);

        /**
         * Classifies a given input vector as positive/negative.
         * @param x  Input vector to be classified.
         * @returns Classification of a given input (true - positive, false - negative).
         */
        bool classify(const std::vector<double>& x);

        /**
         * Calculates the probability for a given input vector to be positive.
         * @param x  Input vector whose probability to be positive needs to be calculated.
         * @returns Probability that a given input vector is positive.
         */
        double probability(const std::vector<double>& x);

        /**
         * Resets the state of the kernel logistic regression classifier.
         */
        void reset();
};

#endif /* KERNEL_LOGISTIC_REGRESSION_CLASSIFIER_HPP_ */
