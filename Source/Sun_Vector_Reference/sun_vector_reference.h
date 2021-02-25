#ifndef	SUN_VECTOR_REFERENCE_H_
#define SUN_VECTOR_REFERENCE_H_
#include <Eigen/Dense>

ret_val SunReference(uint16_t year, uint16_t month, uint16_t day, double UT, Eigen::Vector3d &sun_reference_km, Eigen::Vector3d &sun_reference_au);

#endif /*SUN_VECTOR_REFERENCE_H_*/