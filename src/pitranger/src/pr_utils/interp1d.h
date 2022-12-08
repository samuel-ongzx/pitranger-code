#include <vector>
#include <cfloat>
#include <math.h>

int findNearestNeighbourIndex(float value, std::vector<float> &x)
{
    float dist = FLT_MAX;
    int idx = -1;
    for (int i = 0; i < x.size(); ++i) {
        float newDist = value - x[i];
        if (newDist > 0 && newDist < dist) {
            dist = newDist;
            idx = i;
        }
    }

    return idx;
}

float interp1d(std::vector<float> x, std::vector<float> y, float x_new)
{
    float y_new;

    std::vector<float> dx, dy, slope, intercept;
    dx.reserve(x.size());
    dy.reserve(x.size());
    slope.reserve(x.size());
    intercept.reserve(x.size());
    
    for (int i = 0; i < x.size(); ++i) {
        if (i < x.size()-1) {
            dx.push_back(x[i+1] - x[i]);
            dy.push_back(y[i+1] - y[i]);
            slope.push_back(dy[i] / dx[i]);
            intercept.push_back(y[i] - x[i] * slope[i]);
        }
        else {
            dx.push_back(dx[i-1]);
            dy.push_back(dy[i-1]);
            slope.push_back(slope[i-1]);
            intercept.push_back(intercept[i-1]);
        }
    }

    int idx = findNearestNeighbourIndex(x_new, x);
    y_new = slope[idx] * x_new + intercept[idx];

    return y_new;

}