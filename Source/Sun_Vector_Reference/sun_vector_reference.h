#ifndef	SUN_VECTOR_REFERENCE_H_
#define SUN_VECTOR_REFERENCE_H_
#include <Eigen/Dense>

/*This script combines two MATLAB functions from within my Main simulation
to output a single estimated sun vector (in units of AU and km) between
the Earth and the Sun's body centres in the ECI frame. The ECI frame is
aligned with respect to Earth's EQUATORIAL PLANE (X and Y lie within it,
with X in the vernal equinox direction) and not it's heliocentric
ecliptic plane. This is an important distinction to make in regards to
validation: if using JPL ephemeris data specify the reference frame as
"frame" in regards to J2000. Equations are referenced by equation number

User-defined (and, eventually, GPS/RTC-defined) inputs are:
	   Y - Year, four digits
	   M - Month, 0-12
	   D - Day, 0-31
	   UT - Universal time (UTC), decimal hours
Outputs will be printed to the command line as formatted text:
	   r_S_I in AU - distance from Earth to Sun in ECI frame, units of AU
	   r_S_I in km - distance from Earth to Sun in ECI frame, units of km

n.b. AU stands for Astronomical Unit and is equivalent to the average
distance between the Earth and Sun.

Irrelevant equations are commented out but retained for consistency.*/

extern Eigen::Vector3d sun_vector_reference(int Y, int M, int D, double UT);

#endif /*SUN_VECTOR_REFERENCE_H_*/