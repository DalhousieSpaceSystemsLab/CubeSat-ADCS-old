#include <gtest/gtest.h>
#include <iostream>
#include <fstream>
#include "Eigen/Core"
#include "../IGRF/igrf.h"
#include "../Errors.h"

using namespace std;

//  Test for IGRF function prototype and inputs
TEST(ADCS_test, ADCS_002_001) {
    Eigen::Vector3d mag_reference(3);
    mag_reference << 0, 0, 0;       //  Initialize matrix
    Eigen::Vector3d& mag = mag_reference;

    //  This should succeed
    uint8_t i = MagReference(-30.56120405, -9.857491524, 577, 2020, 11, 17, mag);
    //  Check for return value
    EXPECT_EQ(i, 0);

    //  These should fail
    //  Arg 1 out of bounds
    i = MagReference(-90, -9.857491524, 577, 2020, 11, 17, mag);
    EXPECT_EQ(i, 2);

    //  Arg 2 out of bounds
    i = MagReference(-30.56120405, 181, 577, 2020, 11, 17, mag);
    EXPECT_EQ(i, 2);

    //  Arg 3 out of bounds
    i = MagReference(-30.56120405, -9.857491524, 601, 2020, 11, 17, mag);
    EXPECT_EQ(i, 2);

    //  Arg 4 out of bounds
    i = MagReference(-30.56120405, -9.857491524, 577, 2025, 11, 17, mag);
    EXPECT_EQ(i, 2);

    //  Arg 5 out of bounds
    i = MagReference(-30.56120405, -9.857491524, 601, 2020, 13, 17, mag);
    EXPECT_EQ(i, 2);

    //  Arg 6 out of bounds
    i = MagReference(-30.56120405, -9.857491524, 601, 2000, 2, 30, mag);
    EXPECT_EQ(i, 2);
}

//  Test for IGRF accuracy
TEST(ADCS_test, ADCS_002_002) {
    const double ACCURACY = 0.0000000001;   //  1e-10
    Eigen::Vector3d mag_reference(3);
    mag_reference << 0, 0, 0;       //  Initialize matrix
    Eigen::Vector3d& mag = mag_reference;
    double geo, phi, h, y, m, d, bx, by, bz;

    ifstream IGRFvalues;
    ofstream IGRFout;
	IGRFvalues.open("../../Matlab/IGRF13/Outputs/validation_output_matlab_nocompare.txt", ifstream::in);
    IGRFout.open("../../Matlab/IGRF13/Outputs/validation_output_cpp_compare.txt", ifstream::out);
    IGRFvalues.setf(std::ios::fixed, std:: ios::floatfield);
    IGRFvalues.precision(25);
    IGRFvalues.width(25);
    IGRFout.setf(std::ios::fixed, std:: ios::floatfield);
    IGRFout.precision(25);
    IGRFout.width(25);

    IGRFout << "lat_geodetic" << '\t' << "phi" << '\t' << "H" << '\t' << "year" << '\t' << "month" << '\t' << "day" << '\t' 
        << "matlab_bx" << '\t' << "matlab_by" << '\t' << "matlab_bz" << '\t' << "cpp_bx" << '\t' << "cpp_by" << '\t' << "cpp_bz" << '\t'
        << "err_bx" << '\t' << "err_by" << '\t' << "err_bz" << endl;

    while(IGRFvalues) {
        IGRFvalues >> geo >> phi >> h >> y >> m >> d >> bx >> by >> bz;
        // cout << geo << '\t' << phi << '\t' << h << '\t' << y << '\t' << m << '\t' << d << '\t' << bx << '\t' << by << '\t' << bz << endl;
        ret_val i = MagReference(geo, phi, h, y, m, d, mag);

        // Check for return value
        EXPECT_EQ(i, 0);

        //  Test for accuracy
        EXPECT_NEAR(bx, mag_reference(0, 0), ACCURACY);
        EXPECT_NEAR(by, mag_reference(1, 0), ACCURACY);
        EXPECT_NEAR(bz, mag_reference(2, 0), ACCURACY);

        IGRFout << geo << '\t' << phi << '\t' << h << '\t' << y << '\t' << m << '\t' << d << '\t' << bx << '\t' << by << '\t' << bz << '\t';
        IGRFout << mag_reference(0, 0) << '\t' << mag_reference(1, 0) << '\t' << mag_reference(2, 0) << '\t';
        IGRFout << bx - mag_reference(0, 0) << '\t' << by - mag_reference(1, 0) << '\t' << bz - mag_reference(2, 0) << endl;
    }

    IGRFvalues.close();
    IGRFout.close();
}
