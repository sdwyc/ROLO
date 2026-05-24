#include <rolo/utility.h>
#include <rot_gicp/gicp/rot_vgicp.hpp>
#include <rot_gicp/gicp/impl/rot_vgicp_impl.hpp>

template class fast_gicp::RotVGICP<pcl::PointXYZ, pcl::PointXYZ>;
template class fast_gicp::RotVGICP<pcl::PointXYZI, pcl::PointXYZI>;
template class fast_gicp::RotVGICP<pcl::PointNormal, pcl::PointNormal>;
template class fast_gicp::RotVGICP<PointXYZIRGB, PointXYZIRGB>;
