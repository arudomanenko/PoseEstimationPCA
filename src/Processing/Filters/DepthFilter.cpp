#include "DepthFilter.h"

void DepthFilter::Apply(rs2::depth_frame &depthFrame)
{
    depthFrame = decimationFilter.process(depthFrame);
    depthFrame = spatialFilter.process(depthFrame);
    depthFrame = temporalFilter.process(depthFrame);
    depthFrame = holeFillingFilter.process(depthFrame);
}