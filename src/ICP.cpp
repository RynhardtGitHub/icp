#include "ICP.h"
#include <iostream>

// Find the closest points in the target cloud for each point in the source cloud
void ICP::findCorrespondences(const PointCloud& source, const PointCloud& target, std::vector<int>& closestIndices, double threshold) {
    closestIndices.resize(source.size());
    for (size_t i = 0; i < source.size(); ++i) {
        double minDist = std::numeric_limits<double>::max();
        bool set = false;
        for (size_t j = 0; j < target.size(); ++j) {
            double dist = std::pow(source[i].x - target[j].x, 2) + std::pow(source[i].y - target[j].y, 2);
            if (dist < minDist && dist <= threshold) {
                minDist = dist;
                closestIndices[i] = j;
                set = true;
            } else {
                if (!set) closestIndices[i] = -1;
            }
        }
    }
}

// Compute the centroid of a point cloud
Point2D ICP::computeCentroid(const PointCloud& points) {
    Point2D centroid{0, 0};
    for (const auto& point : points) {
        centroid.x += point.x;
        centroid.y += point.y;
    }
    centroid.x /= points.size();
    centroid.y /= points.size();
    return centroid;
}

void ICP::computeOptimalTransformation(const PointCloud& source, const PointCloud& target, double& theta, Point2D& translation) {
    // Step 1: Compute centroids of both point sets
    Point2D centroid_src = computeCentroid(source);
    Point2D centroid_tgt = computeCentroid(target);

    // Step 2: Center the points around their respective centroids
    auto centered_src = source;
    auto centered_tgt = target;
    for (int i = 0; i < centered_src.size(); i++) {
        centered_src[i].x -= centroid_src.x;
        centered_src[i].y -= centroid_src.y;
    }
    for (int i = 0; i < centered_tgt.size(); i++) {
        centered_tgt[i].x -= centroid_tgt.x;
        centered_tgt[i].y -= centroid_tgt.y;
    }

    // Step 3: Compute the cross-covariance matrix elements
    double Sxx = 0.0, Sxy = 0.0, Syx = 0.0, Syy = 0.0;
    for (size_t i = 0; i < centered_src.size(); ++i) {
        Sxx += centered_src[i].x * centered_tgt[i].x;
        Sxy += centered_src[i].x * centered_tgt[i].y;
        Syx += centered_src[i].y * centered_tgt[i].x;
        Syy += centered_src[i].y * centered_tgt[i].y;
    }

    // Step 4: Compute rotation angle theta using SVD simplification
    theta = atan2(Sxy - Syx, Sxx + Syy);

    // Step 5: Compute the translation vector
    translation.x = centroid_tgt.x - (centroid_src.x * cos(theta) - centroid_src.y * sin(theta));
    translation.y = centroid_tgt.y - (centroid_src.x * sin(theta) + centroid_src.y * cos(theta));
}

// Apply the transformation (rotation and translation) to a point
Point2D ICP::applyTransformation(const Point2D& point, double rotationAngle, const Point2D& translation) {
    Point2D transformedPoint;
    transformedPoint.x = point.x * std::cos(rotationAngle) - point.y * std::sin(rotationAngle) + translation.x;
    transformedPoint.y = point.x * std::sin(rotationAngle) + point.y * std::cos(rotationAngle) + translation.y;
    return transformedPoint;
}

// Main ICP alignment function that returns the final 3x3 transformation matrix
std::vector<std::vector<double>> ICP::align(PointCloud& source, const PointCloud& target, int maxIterations, double tolerance) {
    double cumulativeRotation = 0;
    Point2D cumulativeTranslation{0, 0};
    double prevError = std::numeric_limits<double>::max();

    for (int iter = 0; iter < maxIterations; ++iter) {
        // Step 1: Find correspondences
        std::vector<int> closestIndices;
        findCorrespondences(source, target, closestIndices);

        // Step 2: Compute optimal rotation and translation for this iteration
        double rotationAngle;
        Point2D translation;
        // computeOptimalTransformation(source, target, closestIndices, rotationAngle, translation);

        // Remove points without correspondence
        PointCloud sourceReplace = {};
        for (int i = 0; i < closestIndices.size(); i++) {
            if (closestIndices[i] != -1) sourceReplace.push_back(source[i]);
        }

        computeOptimalTransformation(sourceReplace, target, rotationAngle, translation);

        // Step 3: Accumulate the transformations
        cumulativeRotation += rotationAngle;
        cumulativeTranslation.x += translation.x;
        cumulativeTranslation.y += translation.y;

        // Step 4: Apply the transformation to the source points
        for (auto& point : source) {
            point = applyTransformation(point, rotationAngle, translation);
        }

        // Step 5: Compute the mean squared error
        double mse = 0;
        for (size_t i = 0; i < source.size(); ++i) {
            if (closestIndices[i] == -1) continue;
            const Point2D& p = source[i];
            const Point2D& q = target[closestIndices[i]];
            mse += std::pow(p.x - q.x, 2) + std::pow(p.y - q.y, 2);
        }
        mse /= source.size();

        // Check for convergence
        if (std::abs(prevError - mse) < tolerance) {
            std::cout << "ICP converged at iteration " << iter + 1 << std::endl;
            break;
        }
        prevError = mse;
    }

    // Step 6: Construct the final 3x3 transformation matrix
    std::vector<std::vector<double>> transformationMatrix = {
        {std::cos(cumulativeRotation), -std::sin(cumulativeRotation), cumulativeTranslation.x},
        {std::sin(cumulativeRotation),  std::cos(cumulativeRotation), cumulativeTranslation.y},
        {0,                           0,                          1}
    };

    return transformationMatrix;
}
