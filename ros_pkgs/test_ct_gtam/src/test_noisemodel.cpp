#include <gtsam/linear/NoiseModel.h>
#include <gtsam/base/TestableAssertions.h>

#include <CppUnitLite/TestHarness.h>
#include <CppUnitLite/TestRegistry.h>

#include <boost/assign/std/vector.hpp>

#include <iostream>
#include <limits>

using namespace std;
using namespace gtsam;
using namespace noiseModel;
using namespace boost::assign;

static const double kSigma = 2, kInverseSigma = 1.0 / kSigma,
                    kVariance = kSigma * kSigma, prc = 1.0 / kVariance;
static const Matrix R = Matrix3::Identity() * kInverseSigma;
static const Matrix kCovariance = Matrix3::Identity() * kVariance;
static const Vector3 kSigmas(kSigma, kSigma, kSigma);

TEST( NoiseModel, MixedQR )
{
  // Call Constrained version, with first and third row treated as constraints
  // Naming the 6 variables u,v,w,x,y,z, we have
  // u = -z
  // w = -x
  // And let's have simple priors on variables
  Matrix Ab(5,6+1);
  Ab <<
      1,0,0,0,0,1,  0, // u+z = 0
      0,0,0,0,1,0,  0, // y^2
      0,0,1,1,0,0,  0, // w+x = 0
      0,1,0,0,0,0,  0, // v^2
      0,0,0,0,0,1,  0; // z^2
  Vector mixed_sigmas = (Vector(5) << 0, 1, 0, 1, 1).finished();
  SharedDiagonal constrained = noiseModel::Constrained::MixedSigmas(mixed_sigmas);

  // Expected result
  Vector expectedSigmas = (Vector(5) << 0, 1, 0, 1, 1).finished();
  SharedDiagonal expectedModel = noiseModel::Diagonal::Sigmas(expectedSigmas);
  Matrix expectedRd(5, 6+1);
  expectedRd << 1, 0, 0, 0, 0, 1, 0,  //
                0, 1, 0, 0, 0, 0, 0,  //
                0, 0, 1, 1, 0, 0, 0,  //
                0, 0, 0, 0, 1, 0, 0,  //
                0, 0, 0, 0, 0, 1, 0;  //

  SharedDiagonal actual = constrained->QR(Ab);
  EXPECT(assert_equal(*expectedModel,*actual,1e-6));
  EXPECT(linear_dependent(expectedRd,Ab,1e-6)); // Ab was modified in place !!!
}

/* ************************************************************************* */
TEST( NoiseModel, MixedQR2 )
{
  // Let's have three variables x,y,z, but x=z and y=z
  // Hence, all non-constraints are really measurements on z
  Matrix Ab(11,3+1);
  Ab <<
      1,0,0,  0, //
      0,1,0,  0, //
      0,0,1,  0, //
     -1,0,1,  0, // x=z
      1,0,0,  0, //
      0,1,0,  0, //
      0,0,1,  0, //
     0,-1,1,  0, // y=z
      1,0,0,  0, //
      0,1,0,  0, //
      0,0,1,  0; //

  Vector sigmas(11);
  sigmas.setOnes();
  sigmas[3] = 0;
  sigmas[7] = 0;
  SharedDiagonal constrained = noiseModel::Constrained::MixedSigmas(sigmas);

  // Expected result
  Vector3 expectedSigmas(0,0,1.0/3);
  SharedDiagonal expectedModel = noiseModel::Constrained::MixedSigmas(expectedSigmas);
  Matrix expectedRd(11, 3+1);
  expectedRd.setZero();
  expectedRd.row(0) << -1,  0, 1,  0;  // x=z
  expectedRd.row(1) <<  0, -1, 1,  0;  // y=z
  expectedRd.row(2) <<  0,  0, 1,  0;  // z=0 +/- 1/3

  SharedDiagonal actual = constrained->QR(Ab);
  EXPECT(assert_equal(*expectedModel,*actual,1e-6));
  EXPECT(assert_equal(expectedRd,Ab,1e-6)); // Ab was modified in place !!!
}


TEST( NoiseModel, Julia_WLS_test )
{
  // Let's have three variables x,y,z, but x=z and y=z
  // Hence, all non-constraints are really measurements on z
  int m = 14; 
  int n = 9;
  Matrix Ab(14,9+1);
  Ab.setZero();
  Ab(1 -1, 1-1)  =  0.993311;
  Ab(10-1, 1-1)  =  0.262992;
  Ab(12-1, 1-1)  =  0.526443;
  Ab(2 -1, 2-1)  =  0.745487;
  Ab(10-1, 2-1)  =  0.465019;
  Ab(13-1, 2-1)  =  0.275519;
  Ab(3 -1, 3-1)  =  0.661141;
  Ab(10-1, 3-1)  =  0.461823;
  Ab(13-1, 3-1)  =  0.951861;
  Ab(14-1, 3-1)  =  0.288737;
  Ab(4 -1, 4-1)  =  0.651704;
  Ab(11-1, 4-1)  =  0.661232;
  Ab(14-1, 4-1)  =  0.194568;
  Ab(5 -1, 5-1)  =  0.879331;
  Ab(10-1, 5-1)  =  0.393193;
  Ab(12-1, 5-1)  =  0.990741;
  Ab(6 -1, 6-1)  =  0.530274;
  Ab(11-1, 6-1)  =  0.550334;
  Ab(7 -1, 7-1)  =  0.457682;
  Ab(10-1, 7-1)  =  0.580782;
  Ab(8 -1, 8-1)  =  0.501377;
  Ab(11-1, 8-1)  =  0.768359;
  Ab(13-1, 8-1)  =  0.519525;
  Ab(14-1, 8-1)  =  0.514863;
  Ab(9 -1, 9-1)  =  0.142741;
  // b
  Ab(0   ,   9)  = 0.9933108108234802; 
  Ab(1   ,   9)  = 0.7454871476588798;
  Ab(2   ,   9)  = 0.6611414180110222;
  Ab(3   ,   9)  = 0.6517038058007115; 
  Ab(4   ,   9)  = 0.879331023254256 ;
  Ab(5   ,   9)  = 0.5302737333545098;
  Ab(6   ,   9)  = 0.4576815193975728; 
  Ab(7   ,   9)  = 0.5013773128016713; 
  Ab(8   ,   9)  = 0.14274056435599253; 
  Ab(9   ,   9)  = 0.9981360570779374; 
  Ab(10  ,   9)  = 0.6036816885930896; 
  Ab(11  ,   9)  = 0.7587747975618098; 
  Ab(12  ,   9)  = 0.590953157912963 ;
  Ab(13  ,   9)  = 0.7220856092527972;

  Vector sigmas(14);
  sigmas.setOnes();
  sigmas[ 9] = 0;
  sigmas[10] = 0;
  sigmas[11] = 0;
  sigmas[12] = 0;
  sigmas[13] = 0;
  SharedDiagonal constrained = noiseModel::Constrained::MixedSigmas(sigmas);

  // Expected result
  
  Vector x(9);
  x <<   0.7393321998299509,
  0.22699216397390404,
 -0.095885911417461,
  0.697431671663542,
  0.372808057221239,
 -1.4061250694280565,
  1.0254310980718055,
  1.1923500538487568,
  1.000002842839277;
  std::cout << Ab.block(0,0,m,n)*x - Ab.block(0,9,m,1) << std::endl;  
  SharedDiagonal actual = constrained->QR(Ab);
  std::cout << Ab << std::endl;
  
}



/* ************************************************************************* */
int main() {  TestResult tr; return TestRegistry::runAllTests(tr); }
/* ************************************************************************* */