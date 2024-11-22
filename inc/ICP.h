#ifndef ICP_H
#define ICP_H

#include <vector>
#include <cmath>
#include <limits>

struct Point2D {
    double x;
    double y;
};

using PointCloud = std::vector<Point2D>;

class ICP {
public:
    // Main ICP function to align the source cloud to the target cloud
    // Returns the final 3x3 transformation matrix
    std::vector<std::vector<double>> align(PointCloud& source, const PointCloud& target, int maxIterations, double tolerance);

private:
    // Helper functions
    void findCorrespondences(const PointCloud& source, const PointCloud& target, std::vector<int>& closestIndices, double threshold = 0.05);
    Point2D computeCentroid(const PointCloud& points);
    void computeOptimalTransformation(const PointCloud& source, const PointCloud& target, double& theta, Point2D& translation);
    Point2D applyTransformation(const Point2D& point, double rotationAngle, const Point2D& translation);
};

#endif // ICP_H
