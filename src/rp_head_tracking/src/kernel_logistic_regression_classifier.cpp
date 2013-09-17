/**
 * @copyright  Manfredas Zabarauskas, 2013. All rights reserved. This project is released under
 *             CC BY-NC-SA (Creative Commons Attribution-NonCommercial-ShareAlike) license.
 */
#include <kernel_logistic_regression_classifier.hpp>

// ROS includes
#include <ros/ros.h>


void RPKernelLogisticRegressionClassifier::addTrainingSample(std::vector<double>& features, bool is_positive)
{
    weights.push_back(0.0);
    gradient.push_back(0.0);
    step_sizes.push_back(ETA);

    training_points.push_back(RPTrainingPoint());

    RPTrainingPoint& last_point = training_points.back();
    last_point.features = features;
    last_point.is_positive = is_positive;
}


bool RPKernelLogisticRegressionClassifier::classify(const std::vector<double>& x)
{
    return (probability(x) >= 0.5);
}


double RPKernelLogisticRegressionClassifier::probability(const std::vector<double>& x)
{
    double weight_sum = 0.0;
    for (unsigned i = 0; i < training_points.size(); i++)
    {
        weight_sum += weights[i] * kernel.k(x, training_points[i].features);
    }

    return sigmoid(weight_sum);
}


double RPKernelLogisticRegressionClassifier::gradientStep()
{
    double cumulative_step_size = 0.0;

    std::vector<double> new_gradient(gradient.size(), 0.0);

    for (unsigned i = 0; i < training_points.size(); i++)
    {
        double gradient_i = 0.0;
        for (unsigned n = 0; n < training_points.size(); n++)
        {
            double t_n = training_points[n].is_positive ? 1.0 : 0.0;

            // Sigmoid calculation
            double sigmoid_argument = 0.0;
            for (unsigned k = 0; k < training_points.size(); k++)
            {
                std::vector<double>& x_k = training_points[k].features;

                sigmoid_argument += weights[k] * kernel.kernel_matrix.at<double>(k, n);
            }

            gradient_i += kernel.kernel_matrix.at<double>(i, n) * (weights[n] / (GAMMA * GAMMA) - t_n + sigmoid(sigmoid_argument));
        }

        new_gradient[i] = gradient_i;
    }

    // RProp variance algorithm
    for (unsigned i = 0; i < training_points.size(); i++)
    {
        if (new_gradient[i] * gradient[i] > 0.0)
        {
            step_sizes[i] *= 1.2;
        }
        else if (new_gradient[i] * gradient[i] < 0.0)
        {
            step_sizes[i] *= 0.5;
            new_gradient[i] = 0.0;
        }

        double partial_step_size = step_sizes[i] * sgn(new_gradient[i]);

        weights[i] -= partial_step_size;
        cumulative_step_size += partial_step_size * partial_step_size;
    }

    gradient = new_gradient;

    return sqrt(cumulative_step_size);
}


void RPKernelLogisticRegressionClassifier::train(double time_limit_s)
{
    kernel.buildKernelMatrix(training_points);

    ros::Duration maximum_duration(time_limit_s);
    ros::Time start_time = ros::Time::now();

    while (ros::Time::now() - start_time < maximum_duration)
    {
        gradientStep();
    }
}


void RPKernelLogisticRegressionClassifier::reset()
{
    weights.clear();
    gradient.clear();
    step_sizes.clear();
    training_points.clear();
}
