#ifndef WOODPLANKDETECTOR_DEPTHFILTER_H
#define WOODPLANKDETECTOR_DEPTHFILTER_H

#include <librealsense2/rs.hpp>

/**
    * @class DepthFilter
    * Class for postprocess realsense frames
*/
class DepthFilter
{
public:
    /**
    * Constructor for DepthFilter
    * @param decimationMagnitude - magnitude parameter for decimation filter
    * @param spatialMagnitude - magnitude parameter for spatial filter
    * @param spatialAlpha - alpha parameter for spatial filter
    * @param spatialDelta - delta parameter for spatial filter
    * @param holeFilling - parameter for hole filling filter
    */
    explicit DepthFilter(
            float decimationMagnitude = 4, float spatialMagnitude = 5, float spatialAlpha = 1,
            float spatialDelta = 50, float holeFilling = 3)
    {
        decimationFilter.set_option(RS2_OPTION_FILTER_MAGNITUDE, decimationMagnitude);

        spatialFilter.set_option(RS2_OPTION_FILTER_MAGNITUDE, spatialMagnitude);
        spatialFilter.set_option(RS2_OPTION_FILTER_SMOOTH_ALPHA, spatialAlpha);
        spatialFilter.set_option(RS2_OPTION_FILTER_SMOOTH_DELTA, spatialDelta);
        spatialFilter.set_option(RS2_OPTION_HOLES_FILL, holeFilling);
    }

    /**
    * Apply RealsenseFilters for current frame
    * @param depthFrame - input frame from realsense camera
    */
    void Apply(rs2::depth_frame &depthFrame);
private:
    rs2::decimation_filter decimationFilter;
    rs2::spatial_filter spatialFilter;
    rs2::temporal_filter temporalFilter;
    rs2::hole_filling_filter holeFillingFilter;
};


#endif //WOODPLANKDETECTOR_DEPTHFILTER_H
