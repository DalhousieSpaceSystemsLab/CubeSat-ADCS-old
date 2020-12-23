#include <stdio.h>
#include <math.h>

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

int  M, D, H, Min, S;
double UT, J0, T0, S_lon_m, M_s, Y, lon_S_ec, epsilon, r_S, r_S_I[3][1], r_S_Ikm[3][1], e_ES[3][1];

#define M_PI 3.14159265358979323846

double sind(double x) {
    return sin(x * M_PI / 180);
}

double cosd(double x) {
    return cos(x * M_PI / 180);
}

int main(void) {

    /*-------------------------------------------------------------------------
    Date has been hardcoded, but set it how you want!
    Separate date (as is done manually in my program):*/

    Y = 2019;
    M = 11;
    D = 27;

    //Current universal time in hours, chosen arbitarily
    UT = 14.55365; // 14:33:13 UTC

   /*  Values for validation for** THIS SPECIFIC TIME ONLY** are:
     JD 2458815.106250000, A.D. 2019 - Nov - 27 14 : 33 : 00.0000

         r_Earth->Sun(AU)
         | -0.41626905 |
         | -0.82085774 | [AU]
         | -0.35584133 |

         r_Earth->Sun(km)
         | -62272964.0803042 |
         | -122798570.6136345 | [km]
         | -53233105.5879802 |


    -------------------------------------------------------------------------
    Greenwich sidereal time, as outlined in Curtis' Orbital Mechanics for
    Engineering Students Ch. 5 p. 259. This procedure for determining the
    Julian day number and century is useful for coarser estimation, but is
    currently irrelevant.

    Manually calculate Julian day number since January 1st, 4163 BC (Curtis 5.48)
    J0 = 367*Y - fix(7*(Y + fix((M + 9)/12))/4) + fix(275*M/9) + D + 1721013.5;

    Time in Julian centuries, w/r/t J2000 (Curtis 5.49)
    T0 = (J0 - 2451545)/36525;

    Greenwich sidereal time at 0 h universal time in degrees (Curtis 5.50)
    THETA_G0 = 100.4606184 + 36000.77004*T0 + 0.000387933*T0^2 - 2.583e-8*T0^3;
    Maintain range of 0 to 360 degrees
    mod(THETA_G0,360);

    Greenwich sidereal time with respect to the current universal time
    THETA_G = THETA_G0 + 360.98564724*UT/24;
    Maintain range of 0 to 360 degrees
    THETA_G = mod(THETA_G,360);

    Markley demonstrates a more precise Julian Date (including time) in
    Fundamentals of Spacecraft Attitude Determination and Control pp. 33-34
    This is used to calculate the ACCURATE time in Julian centuries input needed for
    the sun vector estimation algorithm to maintain ephemeridial accuracy
    with values from JPL.*/

    //Separate time into hours, minutes, and seconds:
    H = (int)UT; // 0 to 24
    Min = (int)((UT - H) * 60);
    S = ((UT - H) * 60 - Min) * 60;

    //Calculate accurate-to-time Julian date, with leap-seconds conceptually
    //ignored (Markley 2.68):
    J0 = 1721013.5 + (367 * Y) - (int)((7.0 / 4.0) * (Y + (int)((M + 9) / 12))) + (int)(275 * M / 9) + D + (((60.0 * H) + Min + (S / 60.0)) / 1440.0);

    //Time in Julian centuries, w/r/t above Julian date (Markley 11.49)
    T0 = (J0 - 2451545.0) / 36525.0;

    /*From "Fundamentals of Astrodynamics and Applications" by D. Vallado, the
    position of the sun relative to the Earth can be determined "imprecisely"
    (versus measured ephemerides values from JPL etc.) via the following
    J2000 derivation with 0.01 deg accuracy (p. 265).
    (The same equations are given in Fundamentals of Spacecraft Attitude
    Determination and Control pp. 420-422. Equation numbers reference
    both)*/

    //Mean longitude of the sun in degrees w/r/t time in Julian centuries (Markley 11.48a)
    S_lon_m = 280.460 + (36000.771 * T0);

    // Sun mean anomaly, deg, "low-precision" w/r/t " " " (Markley 11.48b)
    M_s = 357.5277233 + (35999.05034 * T0);

    // "Reduce both [...] to the range of 0 deg to 360 deg"
    S_lon_m = fmod(S_lon_m, 360.0); // deg
    M_s = fmod(M_s, 360.0); // deg

    /* Ecliptic longitude via "equation of centre" (Vallado p. 81) where 0.016708617 is
    the "mean eccentricity of Earth's orbit around the sun". Unlike the
    textbook, the equation is solved to the fourth order.
    lon_S_ec = M_s + 2*0.016708617*sind(M_s) + 5/4*0.016708617^2*sind(2*M_s) + ...
        0.016708617^3/12*(13*sind(3*M_s) - 3*sind(M_s)) + 0.016708617^4/96*(103*sind(4*M_s) - 44*sind(2*M_s));*/

        // Or, use the simplified formula (Markley 11.50)
    lon_S_ec = S_lon_m + (1.914666471 * sind(M_s)) + (0.019994643 * sind(2.0 * M_s)); // deg

    // ****** Assuming the ecliptic latitude of the sun is 0 deg ******

    // Obliquity of the ecliptic is approximated as (Markley 11.51):
    epsilon = 23.439291 - (0.0130042 * T0); // deg

    // Magnitude of Earth's positon w/r/t sun, in astronomical units (AU) (Markley 11.53)
    r_S = 1.000140612 - (0.016708617 * cosd(M_s)) - (0.000139589 * cosd(2.0 * M_s)); // AU

    // Vector in the J2000 ECI frame (assuming "mean equinox" is synonymous with
    // vernal equinox) (Markley 11.52)

    r_S_I[0][0] = r_S * cosd(lon_S_ec); // AU
    r_S_I[1][0] = r_S * cosd(epsilon) * sind(lon_S_ec); // AU
    r_S_I[2][0] = r_S * sind(epsilon) * sind(lon_S_ec); // AU


    // Convert from AU to kilometres 
    for (int i = 0; i < 3; i++) {
        r_S_Ikm[i][0] = r_S_I[i][0] * (149597870700.0) * 1e-3; // 149597870700 metres = 1 AU
    }

    /*// Obtain unit vector:
    for (int i = 0; i < 3; i++) {
        e_ES[i][0] = r_S_I[i][0] / r_S; // m
    }*/

    // Output
    printf("\n     r_Earth->Sun (AU)             r_Earth->Sun (km)\n\n");
    for (int i = 0; i < 3; i++) {
        printf(" |   %9.8e   |       |   %9.7e    |\n", r_S_I[i][0], r_S_Ikm[i][0]);
    }

}