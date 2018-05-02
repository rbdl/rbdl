//==============================================================================
/* 
 * RBDL - Rigid Body Dynamics Library: Addon : muscle
 * Copyright (c) 2016 Matthew Millard <matthew.millard@iwr.uni-heidelberg.de>
 *
 * Licensed under the zlib license. See LICENSE for more details.
 */

#include "Millard2016TorqueMuscle.h"
#include "TorqueMuscleFunctionFactory.h"
#include "csvtools.h"

#include <limits>

#include <cmath>
#include <cstdio>
#include <iostream>
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <vector>
 
#include <ostream>

static double EPSILON = std::numeric_limits<double>::epsilon();
static double SQRTEPSILON = sqrt(EPSILON);

using namespace RigidBodyDynamics::Math;
using namespace RigidBodyDynamics::Addons::Muscle;
using namespace RigidBodyDynamics::Addons::Geometry;
using namespace std;

/*************************************************************
 Table Access Structure Names
*************************************************************/
const double gravity = 9.81; //Needed for the strength scaling used
                             //by Anderson et al.

const char* DataSet::names[] = {  "Anderson2007",
                                  "Gymnast"};

const char* GenderSet::names[] = {"Male",
                                  "Female"};

const char* AgeGroupSet::names[] = {"Young18To25",
                                    "Middle55To65",
                                    "SeniorOver65"};

const char* JointTorqueSet::names[] = { "HipExtension"               ,
                                        "HipFlexion"                 ,
                                        "KneeExtension"              ,
                                        "KneeFlexion"                ,
                                        "AnkleExtension"             ,
                                        "AnkleFlexion"               ,
                                        "ElbowExtension"             ,
                                        "ElbowFlexion"               ,
                                        "ShoulderExtension"          ,
                                        "ShoulderFlexion"            ,
                                        "WristExtension"             ,
                                        "WristFlexion"               ,
                                        "ShoulderHorizontalAdduction",
                                        "ShoulderHorizontalAbduction",
                                        "ShoulderInternalRotation"   ,
                                        "ShoulderExternalRotation"   ,
                                        "WristUlnarDeviation"        ,
                                        "WristRadialDeviation"       ,
                                        "WristPronation"             ,
                                        "WristSupination"            ,
                                        "LumbarExtension"            ,
                                        "LumbarFlexion" };

const char* Anderson2007::GenderNames[] = {"Male","Female"};

const char* Anderson2007::AgeGroupNames[] = { "Young18To25",
                                              "Middle55To65",
                                              "SeniorOver65"};

const char* Anderson2007::JointTorqueNames[] = {"HipExtension"  ,
                                                "HipFlexion"    ,
                                                "KneeExtension" ,
                                                "KneeFlexion"   ,
                                                "AnkleExtension",
                                                "AnkleFlexion" };

const char* Gymnast::GenderNames[] = {"Male"};
const char* Gymnast::AgeGroupNames[] = {"Young18To25"};
const char* Gymnast::JointTorqueNames[] =
                                      { "HipExtension"               ,
                                        "HipFlexion"                 ,
                                        "KneeExtension"              ,
                                        "KneeFlexion"                ,
                                        "AnkleExtension"             ,
                                        "AnkleFlexion"               ,
                                        "ElbowExtension"             ,
                                        "ElbowFlexion"               ,
                                        "ShoulderExtension"          ,
                                        "ShoulderFlexion"            ,
                                        "WristExtension"             ,
                                        "WristFlexion"               ,
                                        "ShoulderHorizontalAdduction",
                                        "ShoulderHorizontalAbduction",
                                        "ShoulderInternalRotation"   ,
                                        "ShoulderExternalRotation"   ,
                                        "WristUlnarDeviation"        ,
                                        "WristRadialDeviation"       ,
                                        "WristPronation"             ,
                                        "WristSupination"            ,
                                        "LumbarExtension"            ,
                                        "LumbarFlexion"};

/*************************************************************
 Coefficient Tables
*************************************************************/

/*
This data is taken from Table 3 of 

Anderson, D. E., Madigan, M. L., & Nussbaum, M. A. (2007). 
Maximum voluntary joint torque as a function of joint angle 
and angular velocity: model development and application to 
the lower limb. Journal of biomechanics, 40(14), 3105-3113.

Each row contains the coefficients for the active and 
passive torque characteristics for a specific joint,
direction, gender and age group. Each row corresponds
to a single block taken from Table 3, as read from 
left to right top to bottom. The first 4 columns have
been added to describe the joint, direction, gender
and age group.

Column labels:
Parameter Set Meta Data
  0: joint:     hip0_knee1_ankle2, 
  1: direction: ext0_flex1, 
  2: gender:    male0_female1, 
  3: age:       age18to25_0_55to65_1_g65_2, 

Active Torque-Angle and Torque-Velocity Curves
  4:  c1, 
  5:  c2, 
  6:  c3, 
  7:  c4, 
  8:  c5, 
  9:  c6, 
Passive Torque-Angle Curves  
  10: b1,
  11: k1, 
  12: b2,
  13: k2, 
*/
double const Millard2016TorqueMuscle::Anderson2007Table3Mean[36][14] = {
{0,0,0,0,0.161,0.958,0.932,1.578,3.19,0.242,-1.21,-6.351,0.476,5.91         },      
{0,0,1,0,0.181,0.697,1.242,1.567,3.164,0.164,-1.753,-6.358,0.239,3.872      },         
{0,0,0,1,0.171,0.922,1.176,1.601,3.236,0.32,-2.16,-8.073,0.108,4.593        },       
{0,0,1,1,0.14,0.83,1.241,1.444,2.919,0.317,-1.361,-7.128,0.013,6.479        },       
{0,0,0,2,0.144,0.896,1.125,1.561,3.152,0.477,-2.671,-7.85,0.092,5.192       },        
{0,0,1,2,0.138,0.707,1.542,1.613,3.256,0.36,-0.758,-7.545,0.018,6.061       },        
{0,1,0,0,0.113,0.738,-0.214,2.095,4.267,0.218,1.21,-6.351,-0.476,5.91       },        
{0,1,1,0,0.127,0.65,-0.35,2.136,4.349,0.156,1.753,-6.358,-0.239,3.872       },        
{0,1,0,1,0.107,0.712,-0.192,2.038,4.145,0.206,2.16,-8.073,-0.108,4.593      },         
{0,1,1,1,0.091,0.812,-0.196,2.145,4.366,0.186,1.361,-7.128,-0.013,6.479     },          
{0,1,0,2,0.101,0.762,-0.269,1.875,3.819,0.296,2.671,-7.85,-0.092,5.192      },         
{0,1,1,2,0.081,0.625,-0.422,2.084,4.245,0.196,0.758,-7.545,-0.018,6.061     },          
{1,0,0,0,0.163,1.258,1.133,1.517,3.952,0.095,0,0,-6.25,-4.521               },
{1,0,1,0,0.159,1.187,1.274,1.393,3.623,0.173,0,0,-8.033,-5.25               },
{1,0,0,1,0.156,1.225,1.173,1.518,3.954,0.266,0,0,-12.83,-5.127              }, 
{1,0,1,1,0.128,1.286,1.141,1.332,3.469,0.233,0,0,-6.576,-4.466              }, 
{1,0,0,2,0.137,1.31,1.067,1.141,3.152,0.386,0,0,-10.519,-5.662              }, 
{1,0,1,2,0.124,1.347,1.14,1.066,2.855,0.464,0,0,-8.8,-6.763                 },
{1,1,0,0,0.087,0.869,0.522,2.008,5.233,0.304,0,0,6.25,-4.521                },
{1,1,1,0,0.08,0.873,0.635,1.698,4.412,0.175,0,0,8.033,-5.25                 },
{1,1,0,1,0.081,0.986,0.523,1.83,4.777,0.23,0,0,12.83,-5.127                 },
{1,1,1,1,0.06,0.967,0.402,1.693,4.41,0.349,0,0,6.576,-4.466                 },
{1,1,0,2,0.069,0.838,0.437,1.718,4.476,0.414,0,0,10.519,-5.662              }, 
{1,1,1,2,0.06,0.897,0.445,1.121,2.922,0.389,0,0,8.8,-6.763                  },
{2,0,0,0,0.095,1.391,0.408,0.987,3.558,0.295,-0.0005781,-5.819,0.967,6.09   },            
{2,0,1,0,0.104,1.399,0.424,0.862,3.109,0.189,-0.005218,-4.875,0.47,6.425    },           
{2,0,0,1,0.114,1.444,0.551,0.593,2.128,0.35,-0.001311,-10.943,0.377,8.916   },            
{2,0,1,1,0.093,1.504,0.381,0.86,3.126,0.349,-2.888e-05,-17.189,0.523,7.888  },             
{2,0,0,2,0.106,1.465,0.498,0.49,1.767,0.571,-5.693e-05,-21.088,0.488,7.309  },             
{2,0,1,2,0.125,1.299,0.58,0.587,1.819,0.348,-2.35e-05,-12.567,0.331,6.629   },            
{2,1,0,0,0.033,1.51,-0.187,0.699,1.94,0.828,0.0005781,-5.819,-0.967,6.09    },           
{2,1,1,0,0.027,1.079,-0.302,0.864,2.399,0.771,0.005218,-4.875,-0.47,6.425   },            
{2,1,0,1,0.028,1.293,-0.284,0.634,1.759,0.999,0.001311,-10.943,-0.377,8.916 },              
{2,1,1,1,0.024,1.308,-0.254,0.596,1.654,1.006,2.888e-05,-17.189,-0.523,7.888},               
{2,1,0,2,0.029,1.419,-0.174,0.561,1.558,1.198,5.693e-05,-21.088,-0.488,7.309},               
{2,1,1,2,0.022,1.096,-0.369,0.458,1.242,1.401,2.35e-05,-12.567,-0.331,6.629 }};


//See the description for the mean data. This table constains the
//parameter standard deviations
double const Millard2016TorqueMuscle::Anderson2007Table3Std[36][14] = {
{0,0,0,0,0.049,0.201,0.358,0.286,0.586,0.272,0.66,0.97,0.547,4.955       },        
{0,0,1,0,0.047,0.13,0.418,0.268,0.542,0.175,1.93,2.828,0.292,1.895       },        
{0,0,0,1,0.043,0.155,0.195,0.306,0.622,0.189,1.297,2.701,0.091,0.854     },          
{0,0,1,1,0.032,0.246,0.365,0.223,0.45,0.14,1.294,2.541,0.02,2.924        },       
{0,0,0,2,0.039,0.124,0.077,0.184,0.372,0.368,0.271,3.402,0.111,1.691     },          
{0,0,1,2,0.003,0.173,0.279,0.135,0.273,0.237,0.613,0.741,0.031,2.265     },          
{0,1,0,0,0.025,0.217,0.245,0.489,0.995,0.225,0.66,0.97,0.547,4.955       },        
{0,1,1,0,0.033,0.178,0.232,0.345,0.702,0.179,1.93,2.828,0.292,1.895      },         
{0,1,0,1,0.02,0.248,0.274,0.318,0.652,0.088,1.297,2.701,0.091,0.854      },         
{0,1,1,1,0.016,0.244,0.209,0.375,0.765,0.262,1.294,2.541,0.02,2.924      },         
{0,1,0,2,0.025,0.151,0.234,0.164,0.335,0.102,0.271,3.402,0.111,1.691     },          
{0,1,1,2,0.008,0.062,0.214,0.321,0.654,0.28,0.613,0.741,0.031,2.265      },         
{1,0,0,0,0.04,0.073,0.073,0.593,1.546,0.171,0,0,2.617,0.553              },                
{1,0,1,0,0.028,0.084,0.181,0.38,0.989,0.27,0,0,3.696,1.512               },               
{1,0,0,1,0.031,0.063,0.048,0.363,0.947,0.06,0,0,2.541,2.148              },                
{1,0,1,1,0.016,0.094,0.077,0.319,0.832,0.133,0,0,1.958,1.63              },                
{1,0,0,2,0.017,0.127,0.024,0.046,0.04,0.124,0,0,1.896,1.517              },                
{1,0,1,2,0.018,0.044,0.124,0.128,0.221,0.129,0,0,6.141,0.742             },                 
{1,1,0,0,0.015,0.163,0.317,1.364,3.554,0.598,0,0,2.617,0.553             },                 
{1,1,1,0,0.015,0.191,0.287,0.825,2.139,0.319,0,0,3.696,1.512             },                 
{1,1,0,1,0.017,0.138,0.212,0.795,2.067,0.094,0,0,2.541,2.148             },                 
{1,1,1,1,0.015,0.21,0.273,0.718,1.871,0.143,0,0,1.958,1.63               },               
{1,1,0,2,0.022,0.084,0.357,0.716,1.866,0.201,0,0,1.896,1.517             },                 
{1,1,1,2,0.005,0.145,0.21,0.052,0.135,0.078,0,0,6.141,0.742              },                
{2,0,0,0,0.022,0.089,0.083,0.595,2.144,0.214,0.001193,7.384,0.323,1.196  },             
{2,0,1,0,0.034,0.19,0.186,0.487,1.76,0.213,0.01135,6.77,0.328,1.177      },         
{2,0,0,1,0.029,0.136,0.103,0.165,0.578,0.133,0.003331,10.291,0.403,3.119 },              
{2,0,1,1,0.026,0.235,0.143,0.448,1.613,0.27,3.562e-05,7.848,0.394,1.141  },             
{2,0,0,2,0.035,0.136,0.132,0.262,0.944,0.313,3.164e-05,1.786,0.258,0.902 },              
{2,0,1,2,0.006,0.095,0.115,0.258,0.423,0.158,2.535e-05,10.885,0.247,2.186},               
{2,1,0,0,0.005,0.19,0.067,0.108,0.301,0.134,0.001193,7.384,0.323,1.196   },            
{2,1,1,0,0.006,0.271,0.171,0.446,1.236,0.206,0.01135,6.77,0.328,1.177    },           
{2,1,0,1,0.005,0.479,0.178,0.216,0.601,0.214,0.003331,10.291,0.403,3.119 },              
{2,1,1,1,0.002,0.339,0.133,0.148,0.41,0.284,3.562e-05,7.848,0.394,1.141  },             
{2,1,0,2,0.002,0.195,0.056,0.188,0.521,0.29,3.164e-05,1.786,0.258,0.902  },             
{2,1,1,2,0.003,0.297,0.109,0.089,0.213,0.427,2.535e-05,10.885,0.247,2.186}};               

/*
  This table contains parameters for the newly made torque muscle curves:

 1. maxIsometricTorque              Nm
 2. maxAngularVelocity              rad/s
 3. angleAtOneActiveNormTorque      rad
 4. angularWidthActiveTorque        rad
 5. tvAtMaxEccentricVelocity        Nm/Nm
 6. tvAtHalfMaxConcentricVelocity   Nm/Nm
 7. angleAtZeroPassiveTorque        rad
 8. angleAtOneNormPassiveTorque     rad

*/



double const Millard2016TorqueMuscle::GymnastWholeBody[22][12] =
{ {0,0,0,0,175.746, 9.02335,  1.06465,  1.05941,  1.1,     0.163849,  0.79158,   1.5708   },
  {0,1,0,0,157.293, 9.18043,  0.733038, 1.21999,  1.11905, 0.25,     -0.0888019,-0.515207 },
  {1,0,0,0,285.619, 19.2161,  0.942478, 0.509636, 1.13292, 0.115304,  2.00713,   2.70526  },
  {1,1,0,0,98.7579, 16.633,   1.02974,  1.11003,  1.12,    0.19746,   0,        -0.174533 },
  {2,0,0,0,127.561, 11.7646,  0.408,    0.660752, 1.159,   0.410591,  0.0292126, 0.785398 },
  {2,1,0,0,44.3106, 17.2746, -0.187,    0.60868,  1.2656,  0.112303, -0.523599, -1.0472   },
  {3,0,0,0,127.401, 16.249,   2.14675,  1.83085,  1.1,     0.250134,  2.8291,    3.55835  },
  {3,1,0,0,91.1388, 19.0764,  0.890118, 1.2898,   1.23011, 0.249656, -0.523599, -1.0472   },
  {5,0,0,0,15.5653, 36.5472,  1.55334,  1.38928,  1.16875, 0.115356,  1.0472,    1.5708   },
  {5,1,0,0,39.2252, 36.3901,  0.663225, 1.71042,  1.14,    0.115456, -0.793739, -1.49714  },
  {3,2,0,0,128.496, 18.05,    0.839204, 1.28041,  1.25856, 0.214179,  1.5708,    2.26893  },
  {3,3,0,0,94.6839, 18    ,  -0.277611, 2.37086,  1.23042, 0.224227, -0.523599, -1.0472   },
  {3,5,0,0,50.522 , 19.47,   -1.18761,  2.80524,  1.27634, 0.285399,  1.39626,   1.91986  },
  {3,4,0,0,43.5837, 18,      -0.670796, 1.98361,  1.35664, 0.229104, -1.0472,   -1.5708   },
  {4,1,0,0,101.384, 18.1,     0.33,     3.62155,  1.37223, 0.189909,  0,        -0.174533 },
  {4,0,0,0,69.8728, 18.45,    1.64319,  1.30795,  1.31709, 0.189676,  2.0944,    2.61799  },
  {5,6,0,0,13.5361, 35.45,   -0.209204, 1.33735,  1.23945, 0.250544, -0.785398, -1.5708   },
  {5,7,0,0,12.976 , 27.88,   -0.212389, 0.991803, 1.3877,  0.207506,  0.785398,  1.5708   },
  {5,9,0,0,31.4217, 18.02,    0.43,     1.47849,  1.34817, 0.196913,  0,         -0.523599},
  {5,8,0,0,23.8345, 21.77,   -1.14319,  2.56082,  1.31466, 0.2092,    0.349066,  0.872665 },
  {6,0,0,0,687.864, 7.98695,  1.5506,   1.14543,  1.1,     0.150907,  0.306223,  1.35342  },
  {6,1,0,0,211.65 , 19.2310,       0,   6.28319,  1.1,     0.150907,  0,        -0.785398 }};


/*
 Original lumbar parameters
  {6,0,0,0,687.864, 1.0472,   1.5506,   1.14543,  1.1,     0.45,      0.306223,  1.35342  },
  {6,1,0,0,211.65 , 0.523599, 0,        6.28319,  1.1,     0.45,      0,        -0.785398 }};
*/

/*************************************************************
 Map that goes from a single joint-torque-direction index to
 the pair of joint and direction indicies used in the tables
*************************************************************/

const static struct JointSet{
  enum item { Hip = 0,
              Knee,
              Ankle,
              Shoulder,
              Elbow,
              Wrist,
              Lumbar,
              Last};
  JointSet(){}
} JointSet;


struct DirectionSet{
  enum item {
      Extension = 0,
      Flexion,
      HorizontalAdduction,
      HorizontalAbduction,
      ExternalRotation,
      InternalRotation,
      Supination,
      Pronation,
      RadialDeviation,
      UlnarDeviation,
      Last
  };
  DirectionSet(){}
} DirectionSet;


const static int JointTorqueMap[22][3] = {
  {(int)JointTorqueSet::HipExtension                , (int)JointSet::Hip      , (int)DirectionSet::Extension          },
  {(int)JointTorqueSet::HipFlexion                  , (int)JointSet::Hip      , (int)DirectionSet::Flexion            },
  {(int)JointTorqueSet::KneeExtension               , (int)JointSet::Knee     , (int)DirectionSet::Extension          },
  {(int)JointTorqueSet::KneeFlexion                 , (int)JointSet::Knee     , (int)DirectionSet::Flexion            },
  {(int)JointTorqueSet::AnkleExtension              , (int)JointSet::Ankle    , (int)DirectionSet::Extension          },
  {(int)JointTorqueSet::AnkleFlexion                , (int)JointSet::Ankle    , (int)DirectionSet::Flexion            },
  {(int)JointTorqueSet::ElbowExtension              , (int)JointSet::Elbow    , (int)DirectionSet::Extension          },
  {(int)JointTorqueSet::ElbowFlexion                , (int)JointSet::Elbow    , (int)DirectionSet::Flexion            },
  {(int)JointTorqueSet::ShoulderExtension           , (int)JointSet::Shoulder , (int)DirectionSet::Extension          },
  {(int)JointTorqueSet::ShoulderFlexion             , (int)JointSet::Shoulder , (int)DirectionSet::Flexion            },
  {(int)JointTorqueSet::WristExtension              , (int)JointSet::Wrist    , (int)DirectionSet::Extension          },
  {(int)JointTorqueSet::WristFlexion                , (int)JointSet::Wrist    , (int)DirectionSet::Flexion            },
  {(int)JointTorqueSet::ShoulderHorizontalAdduction , (int)JointSet::Shoulder , (int)DirectionSet::HorizontalAdduction},
  {(int)JointTorqueSet::ShoulderHorizontalAbduction , (int)JointSet::Shoulder , (int)DirectionSet::HorizontalAbduction},
  {(int)JointTorqueSet::ShoulderInternalRotation    , (int)JointSet::Shoulder , (int)DirectionSet::InternalRotation   },
  {(int)JointTorqueSet::ShoulderExternalRotation    , (int)JointSet::Shoulder , (int)DirectionSet::ExternalRotation   },
  {(int)JointTorqueSet::WristUlnarDeviation         , (int)JointSet::Wrist    , (int)DirectionSet::UlnarDeviation     },
  {(int)JointTorqueSet::WristRadialDeviation        , (int)JointSet::Wrist    , (int)DirectionSet::RadialDeviation    },
  {(int)JointTorqueSet::WristPronation              , (int)JointSet::Wrist    , (int)DirectionSet::Pronation          },
  {(int)JointTorqueSet::WristSupination             , (int)JointSet::Wrist    , (int)DirectionSet::Supination         },
  {(int)JointTorqueSet::LumbarExtension             , (int)JointSet::Lumbar   , (int)DirectionSet::Extension          },
  {(int)JointTorqueSet::LumbarFlexion               , (int)JointSet::Lumbar   , (int)DirectionSet::Flexion            }};


/*************************************************************
 Constructors
*************************************************************/

Millard2016TorqueMuscle::
    Millard2016TorqueMuscle( )
        :angleOffset(1.0),
        signOfJointAngle(1.0),
        signOfJointTorque(1.0),
        signOfConcentricAnglularVelocity(1.0),
        muscleName("empty")
{
  muscleCurvesAreDirty = true;
  useTabularMaxActiveIsometricTorque = true;
  useTabularOmegaMax = true;
  passiveTorqueScale = 1.0;

}

Millard2016TorqueMuscle::Millard2016TorqueMuscle(
    DataSet::item dataSet,
    const SubjectInformation &subjectInfo,
    int jointTorque,
    double  jointAngleOffsetRelativeToDoxygenFigures,
    double  signOfJointAngleRelativeToDoxygenFigures,
    double  signOfJointTorque,
    const   std::string& name
    ):angleOffset(jointAngleOffsetRelativeToDoxygenFigures),
  signOfJointAngle(signOfJointAngleRelativeToDoxygenFigures),
  signOfJointTorque(signOfJointTorque),
  signOfConcentricAnglularVelocity(signOfJointTorque),
  muscleName(name),
  dataSet(dataSet)
{

    subjectHeightInMeters   = subjectInfo.heightInMeters;
    subjectMassInKg         = subjectInfo.massInKg;
    passiveCurveAngleOffset = 0.;
    passiveTorqueScale      = 1.0;
    betaMax = 0.1;

    int gender                    = (int) subjectInfo.gender;
    int ageGroup                  = (int) subjectInfo.ageGroup;

    int joint           = -1;
    int jointDirection  = -1;

    for(int i=0; i < JointTorqueSet::Last; ++i){
      if(JointTorqueMap[i][0] == jointTorque){
        joint           = JointTorqueMap[i][1];
        jointDirection  = JointTorqueMap[i][2];
      }
    }

    if(joint == -1 || jointDirection == -1){
        cerr << "Millard2016TorqueMuscle::"
               << "Millard2016TorqueMuscle:"
               << muscleName
               << ": A jointTorqueDirection of " << jointTorque
               << " does not exist.";
        assert(0);
        abort();
    }

    if( abs(abs(signOfJointAngle)-1) >  EPSILON){

        cerr << "Millard2016TorqueMuscle::"
               << "Millard2016TorqueMuscle:"
               << muscleName
               << ": signOfJointAngleRelativeToAnderson2007 must be [-1,1] not "
               << signOfJointAngle;
        assert(0);
        abort();
    }

    if( abs(abs(signOfConcentricAnglularVelocity)-1) >  EPSILON){
        cerr << "Millard2016TorqueMuscle::"
               << "Millard2016TorqueMuscle:"
               << muscleName
               << ": signOfJointAngularVelocityDuringConcentricContraction "
               << "must be [-1,1] not "
               << signOfConcentricAnglularVelocity;
        assert(0);
        abort();
    }

    if( abs(abs(signOfJointTorque)-1) >  EPSILON){
        cerr << "Millard2016TorqueMuscle::"
               << "Millard2016TorqueMuscle:"
               << muscleName
               << ": signOfJointTorque must be [-1,1] not "
               << signOfJointTorque;
        assert(0);
        abort();
    }


    if(subjectHeightInMeters <= 0){
        cerr   << "Millard2016TorqueMuscle::"
               << "Millard2016TorqueMuscle:"
               << muscleName
               << ": subjectHeightInMeters > 0, but it's "
               << subjectHeightInMeters;
        assert(0);
        abort();
    }

    if(subjectMassInKg <= 0){
        cerr   << "Millard2016TorqueMuscle::"
               << "Millard2016TorqueMuscle:"
               << muscleName
               << ": subjectMassInKg > 0, but it's "
               << subjectMassInKg;
        assert(0);
        abort();
    }



    int idx         = -1;
    int jointIdx    = 0;
    int dirIdx      = 1;
    int genderIdx   = 2;
    int ageIdx      = 3;

    switch(dataSet){
      case DataSet::Anderson2007:
      {
        c1c2c3c4c5c6Anderson2007.resize(6);
        b1k1b2k2Anderson2007.resize(4);

        for(int i=0; i<36;++i){

            if( abs(Anderson2007Table3Mean[i][jointIdx]
                      -(double)joint) < EPSILON
             && abs(Anderson2007Table3Mean[i][dirIdx]
                       -(double)jointDirection) < EPSILON
             && abs(Anderson2007Table3Mean[i][genderIdx]
                       -(double)gender) < EPSILON
             && abs(Anderson2007Table3Mean[i][ageIdx]
                       -(double)ageGroup) < EPSILON){
                idx = i;
            }
        }

        if(idx != -1){
          for(int i=0; i<6; ++i){
              c1c2c3c4c5c6Anderson2007[i] = Anderson2007Table3Mean[idx][i+4];
          }
          for(int i=0; i<4; ++i){
              b1k1b2k2Anderson2007[i] = Anderson2007Table3Mean[idx][i+10];
          }
        }

      } break;

    case DataSet::Gymnast:
      {
        gymnastParams.resize(8);
        for(int i=0; i<JointTorqueSet::Last;++i){

            if( abs(GymnastWholeBody[i][jointIdx]
                      -(double)joint) < EPSILON
             && abs(GymnastWholeBody[i][dirIdx]
                       -(double)jointDirection) < EPSILON
             && abs(GymnastWholeBody[i][genderIdx]
                       -(double)gender) < EPSILON
             && abs(GymnastWholeBody[i][ageIdx]
                       -(double)ageGroup) < EPSILON){
                idx = i;
            }
        }

        if(idx != -1){
          for(int i=0; i<8; ++i){
              gymnastParams[i] = GymnastWholeBody[idx][i+4];
          }
        }
      } break;

      default:
      {
        cerr << "Millard2016TorqueMuscle::"
             << "Millard2016TorqueMuscle:"
             << muscleName
             << "The requested DataSet does not exist.";
        assert(0);
        abort();
      }
    };



    if(idx == -1){
        cerr << "Millard2016TorqueMuscle::"
               << "Millard2016TorqueMuscle:"
               << muscleName
               << "The combination of data set (" << dataSet << ")"
               << " joint (" << joint << ")"
               << " joint direction (" << jointDirection << ")"
               << " gender, (" << gender << ")"
               << "and age " << ageGroup << ")"
               << "could not be found";

        assert(0);
        abort();
    }



    muscleCurvesAreDirty = true;
    useTabularMaxActiveIsometricTorque = true;
    useTabularOmegaMax = true;    
    updateTorqueMuscleCurves();


}


/*************************************************************
 Muscle Model Code
*************************************************************/


double Millard2016TorqueMuscle::
    calcJointTorque(    double jointAngle,
                        double jointAngularVelocity,
                        double activation) const 
{

  if(muscleCurvesAreDirty){
    Millard2016TorqueMuscle* mutableThis =
        const_cast<Millard2016TorqueMuscle* >(this);
    mutableThis->updateTorqueMuscleCurves();
  }


    double fiberAngle    = calcFiberAngle(jointAngle);
    double fiberVelocity = calcFiberAngularVelocity(
                            jointAngularVelocity);
    double ta = taCurve.calcValue(fiberAngle);
    double tp = tpCurve.calcValue(fiberAngle);
    double fiberVelocityNorm = fiberVelocity/omegaMax;
    double tv = tvCurve.calcValue(fiberVelocityNorm);
    //double beta = betaMax*betaCurve.calcValue(fiberVelocityNorm);

    double jointTorque =  maxActiveIsometricTorque*(
                              activation*ta*tv
                            + tp*(1 - betaMax*fiberVelocityNorm));

    return jointTorque*signOfJointTorque;
}

void Millard2016TorqueMuscle::
    calcActivation(double jointAngle,
                   double jointAngularVelocity,
                   double jointTorque,
                   TorqueMuscleSummary &tms) const
{
  if(muscleCurvesAreDirty){
    Millard2016TorqueMuscle* mutableThis =
        const_cast<Millard2016TorqueMuscle* >(this);
    mutableThis->updateTorqueMuscleCurves();
  }

  double activation = 0.;

  tms.activation = activation;
  tms.fiberActiveTorqueAngleMultiplier = 0.;
  tms.fiberActiveTorqueAngularVelocityMultiplier = 0.;
  tms.fiberNormalizedDampingTorque = 0.;
  tms.fiberPassiveTorqueAngleMultiplier = 0.;
  tms.fiberTorque = 0.;
  tms.jointTorque = 0.;

  if(jointTorque*signOfJointTorque > SQRTEPSILON){
    double fiberAngle    = calcFiberAngle(jointAngle);
    double fiberVelocity = calcFiberAngularVelocity(
                            jointAngularVelocity);
    double ta = taCurve.calcValue(fiberAngle);
    double tp = tpCurve.calcValue(fiberAngle);
    double fiberVelocityNorm = fiberVelocity/omegaMax;
    double tv = tvCurve.calcValue(fiberVelocityNorm);
    //double beta = betaMax*betaCurve.calcValue(fiberVelocityNorm);

    double fiberTorque = jointTorque*signOfJointTorque;

    /*
    double jointTorque =  maxActiveIsometricTorque*(
                              activation*ta*tv
                            + tp
                            - beta*fiberVelocityNorm);
    */
    activation = ((fiberTorque/maxActiveIsometricTorque)
                - tp*(1-betaMax*fiberVelocityNorm))
                 /(ta*tv);

    tms.activation = activation;
    tms.fiberActiveTorqueAngleMultiplier = ta;
    tms.fiberActiveTorqueAngularVelocityMultiplier = tv;
    tms.fiberNormalizedDampingTorque = -tp*betaMax*fiberVelocityNorm;
    tms.fiberPassiveTorqueAngleMultiplier = tp;
    tms.fiberTorque = fiberTorque;
    tms.jointTorque = jointTorque;

  }


}

double Millard2016TorqueMuscle::
    calcMaximumActiveIsometricTorqueScalingFactor(
          double jointAngle,
          double jointAngularVelocity,
          double activation,
          double jointTorque) const
{

  if(muscleCurvesAreDirty){
    Millard2016TorqueMuscle* mutableThis =
        const_cast<Millard2016TorqueMuscle* >(this);
    mutableThis->updateTorqueMuscleCurves();
  }

  double scaleFactor = 0.;

  if(jointTorque*signOfJointTorque > SQRTEPSILON){
    double fiberAngle    = calcFiberAngle(jointAngle);
    double fiberVelocity = calcFiberAngularVelocity(
                            jointAngularVelocity);
    double ta = taCurve.calcValue(fiberAngle);
    double tp = tpCurve.calcValue(fiberAngle);
    double fiberVelocityNorm = fiberVelocity/omegaMax;
    double tv = tvCurve.calcValue(fiberVelocityNorm);
    //double beta = betaMax*betaCurve.calcValue(fiberVelocityNorm);

    double fiberTorqueUpd = jointTorque*signOfJointTorque;

    double maxActiveIsometricTorqueUpd =
           fiberTorqueUpd/( activation*ta*tv
                          + tp*(1-betaMax*fiberVelocityNorm));


    scaleFactor = maxActiveIsometricTorqueUpd
                        /maxActiveIsometricTorque;
  }
  return scaleFactor;
}


void Millard2016TorqueMuscle::
    calcTorqueMuscleInfo(double jointAngle,
                        double jointAngularVelocity,
                        double activation,
                        TorqueMuscleInfo& tmi) const
{


  if(muscleCurvesAreDirty){
    Millard2016TorqueMuscle* mutableThis =
        const_cast<Millard2016TorqueMuscle* >(this);
    mutableThis->updateTorqueMuscleCurves();
  }

    double fiberAngle    = calcFiberAngle(jointAngle);
    double fiberAngularVelocity = calcFiberAngularVelocity(jointAngularVelocity);
    double ta = taCurve.calcValue(fiberAngle);
    double tp = tpCurve.calcValue(fiberAngle);

    double omegaNorm  = fiberAngularVelocity/omegaMax;
    double D_wn_w     = 1.0/omegaMax;
    double tv         = tvCurve.calcValue(omegaNorm);

    double beta        = betaMax;//betaMax*betaCurve.calcValue(omegaNorm);
    double D_beta_D_wn = 0;

    double tb         = -tp*beta*omegaNorm;
    double D_tb_Dwn   = -tp*beta;//-beta - D_beta_D_wn*omegaNorm;

    double D_ta_DfiberAngle = taCurve.calcDerivative(fiberAngle,1);
    double D_tp_DfiberAngle = tpCurve.calcDerivative(fiberAngle,1);
    double D_tv_DfiberAngularVelocity
        = tvCurve.calcDerivative(omegaNorm,1)*D_wn_w;
    double D_tb_DfiberAngularVelocity = D_tb_Dwn*D_wn_w;
    double D_tb_DfiberAngle = -D_tp_DfiberAngle*beta*omegaNorm;

    double D_fiberAngle_D_jointAngle = signOfJointAngle;
    double D_tv_DfiberAngularVelocity_D_jointAngularVelocity = 
                              signOfConcentricAnglularVelocity;


    tmi.jointAngle               = jointAngle;
    tmi.jointAngularVelocity     = jointAngularVelocity;
    tmi.fiberAngle               = fiberAngle;
    //tmi.tendonAngle              = 0.;
    tmi.fiberAngularVelocity     = fiberAngularVelocity;
    //tmi.tendonAngularVelocity    = 0.;

    tmi.fiberPassiveTorqueAngleMultiplier            = tp;
    tmi.fiberActiveTorqueAngleMultiplier             = ta;
    tmi.fiberActiveTorqueAngularVelocityMultiplier   = tv;
    //tmi.tendonTorqueAngleMultiplier                  = activation*ta*tv + tp;

    tmi.activation           = activation;
    tmi.fiberActiveTorque    = maxActiveIsometricTorque*(activation*ta*tv);
    tmi.fiberPassiveTorque   = maxActiveIsometricTorque*(tb+tp);
    tmi.fiberDampingTorque   = maxActiveIsometricTorque*(tb);
    tmi.fiberNormDampingTorque = tb;
    tmi.fiberTorque          =   tmi.fiberActiveTorque
                                  + tmi.fiberPassiveTorque;
    //tmi.tendonTorque         =   tmi.fiberActiveTorque
    //                              + tmi.fiberPassiveTorque;

    tmi.jointTorque          = signOfJointTorque*tmi.fiberTorque;

    tmi.fiberStiffness       = maxActiveIsometricTorque*(
                                    activation*D_ta_DfiberAngle*tv 
                                    + D_tp_DfiberAngle);

    //tmi.tendonStiffness      = std::numeric_limits<double>::infinity();
    tmi.jointStiffness       = signOfJointTorque
                                    *tmi.fiberStiffness
                                    *D_fiberAngle_D_jointAngle;

    tmi.fiberActivePower     = tmi.fiberActiveTorque
                                    * tmi.fiberAngularVelocity;
    tmi.fiberPassivePower    = tmi.fiberPassiveTorque
                                    * tmi.fiberAngularVelocity;
    tmi.fiberPower           = tmi.fiberActivePower
                                + tmi.fiberPassivePower;
    //tmi.tendonPower        = 0.;
    tmi.jointPower           = tmi.jointTorque * jointAngularVelocity;


    //tau = signTq*tauIso*(a * ta(theta) * tv(thetaDot) + tp(theta) )
    // Dtau_Da = tauMaxIso*(ta(theta) * tv(thetaDot) )
    tmi.DjointTorqueDactivation =
      signOfJointTorque
        *maxActiveIsometricTorque
        *(ta * tv);

    //Dtau_Domega = signTq*tauIso*(a * ta(theta) * Dtv_DthetaDot(thetaDot)*  )    
    tmi.DjointTorqueDjointAngularVelocity =
        signOfJointTorque
        * maxActiveIsometricTorque
        * ( activation
            * ta
            * D_tv_DfiberAngularVelocity
            * D_tv_DfiberAngularVelocity_D_jointAngularVelocity
            + D_tb_DfiberAngularVelocity);

    tmi.DjointTorqueDjointAngle           =
        signOfJointTorque
        * maxActiveIsometricTorque
        * ( activation
            *D_ta_DfiberAngle
            * tv
            + D_tp_DfiberAngle
            + D_tb_DfiberAngle
         )* D_fiberAngle_D_jointAngle;

}


/*************************************************************
 Get / Set Functions
*************************************************************/

double Millard2016TorqueMuscle::
    getJointTorqueSign() const
{
  if(muscleCurvesAreDirty){
    Millard2016TorqueMuscle* mutableThis =
        const_cast<Millard2016TorqueMuscle* >(this);
    mutableThis->updateTorqueMuscleCurves();
  }
  return signOfJointTorque;
}

double Millard2016TorqueMuscle::
    getJointAngleSign() const
{
  if(muscleCurvesAreDirty){
    Millard2016TorqueMuscle* mutableThis =
        const_cast<Millard2016TorqueMuscle* >(this);
    mutableThis->updateTorqueMuscleCurves();
  }
  return signOfJointAngle;
}

double Millard2016TorqueMuscle::
    getJointAngleOffset() const
{
  if(muscleCurvesAreDirty){
    Millard2016TorqueMuscle* mutableThis =
        const_cast<Millard2016TorqueMuscle* >(this);
    mutableThis->updateTorqueMuscleCurves();
  }
  return angleOffset;
}


double Millard2016TorqueMuscle::
    getNormalizedDampingCoefficient() const
{
  if(muscleCurvesAreDirty){
    Millard2016TorqueMuscle* mutableThis =
        const_cast<Millard2016TorqueMuscle* >(this);
    mutableThis->updateTorqueMuscleCurves();
  }
  return betaMax;
}

void Millard2016TorqueMuscle::
    setNormalizedDampingCoefficient(double betaUpd)
{
  if(betaUpd < 0){
      cerr << "Millard2016TorqueMuscle::"
             << "setNormalizedDampingCoefficient:"
             << muscleName
             << "betaMax is " << betaUpd
             << " but betaMax must be > 0 "
             << endl;
      assert(0);
      abort();
  }

  betaMax = betaUpd;
}




double Millard2016TorqueMuscle::
    getMaximumActiveIsometricTorque() const 
{  
  if(muscleCurvesAreDirty){
    Millard2016TorqueMuscle* mutableThis =
        const_cast<Millard2016TorqueMuscle* >(this);
    mutableThis->updateTorqueMuscleCurves();
  }
  return maxActiveIsometricTorque;
}

double Millard2016TorqueMuscle::
    getMaximumConcentricJointAngularVelocity() const
{
  if(muscleCurvesAreDirty){
    Millard2016TorqueMuscle* mutableThis =
        const_cast<Millard2016TorqueMuscle* >(this);
    mutableThis->updateTorqueMuscleCurves();
  }
  return calcJointAngularVelocity( omegaMax );
}

void Millard2016TorqueMuscle::
    setMaximumActiveIsometricTorque(double maxIsoTorque)
{
    muscleCurvesAreDirty               = true;
    useTabularMaxActiveIsometricTorque = false;
    maxActiveIsometricTorque           = maxIsoTorque;
}    

void Millard2016TorqueMuscle::
    setMaximumConcentricJointAngularVelocity(double maxAngularVelocity){
  if(fabs(maxAngularVelocity) < SQRTEPSILON){
    cerr << "Millard2016TorqueMuscle::"
         << "setMaximumJointAngularVelocity:"
         << muscleName
         << " The value of maxJointAngularVelocity needs to be greater "
            " than sqrt(epsilon), but it is "
         << maxAngularVelocity;
    assert(0);
    abort();
  }

    muscleCurvesAreDirty = true;
    useTabularOmegaMax   = false;
    omegaMax             = fabs(maxAngularVelocity);
}

double Millard2016TorqueMuscle::
    getJointAngleAtMaximumActiveIsometricTorque() const
{
  if(muscleCurvesAreDirty){
    Millard2016TorqueMuscle* mutableThis =
        const_cast<Millard2016TorqueMuscle* >(this);
    mutableThis->updateTorqueMuscleCurves();
  }
  return calcJointAngle(angleAtOneNormActiveTorque);
}

double Millard2016TorqueMuscle::
    getJointAngleAtOneNormalizedPassiveIsometricTorque() const
{
  if(muscleCurvesAreDirty){
    Millard2016TorqueMuscle* mutableThis =
        const_cast<Millard2016TorqueMuscle* >(this);
    mutableThis->updateTorqueMuscleCurves();
  }
   return calcJointAngle(angleAtOneNormPassiveTorque);
}


double Millard2016TorqueMuscle::
    getPassiveTorqueScale() const
{
  if(muscleCurvesAreDirty){
    Millard2016TorqueMuscle* mutableThis =
        const_cast<Millard2016TorqueMuscle* >(this);
    mutableThis->updateTorqueMuscleCurves();
  }
  return passiveTorqueScale;
}

void Millard2016TorqueMuscle::
    setPassiveTorqueScale(double passiveTorqueScaling)
{
    muscleCurvesAreDirty = true;
    passiveTorqueScale = passiveTorqueScaling;
}


double Millard2016TorqueMuscle::
    getPassiveCurveAngleOffset() const
{
  if(muscleCurvesAreDirty){
    Millard2016TorqueMuscle* mutableThis =
        const_cast<Millard2016TorqueMuscle* >(this);
    mutableThis->updateTorqueMuscleCurves();
  }
  return passiveCurveAngleOffset;
}

void Millard2016TorqueMuscle::
    setPassiveCurveAngleOffset(double passiveCurveAngleOffsetVal)
{
    muscleCurvesAreDirty = true;
    passiveCurveAngleOffset = passiveCurveAngleOffsetVal;
}


void Millard2016TorqueMuscle::
    fitPassiveCurveAngleOffset(double jointAngleTarget,
                               double passiveTorqueTarget)
{
    muscleCurvesAreDirty = true;
    setPassiveCurveAngleOffset(0.0);

    if(passiveTorqueTarget < SQRTEPSILON){
      cerr   << "Millard2016TorqueMuscle::"
             << "fitPassiveTorqueScale:"
             << muscleName
             << ": passiveTorque " << passiveTorqueTarget
             << " but it should be greater than sqrt(eps)"
             << endl;
      assert(0);
      abort();
    }

    //Solve for the fiber angle at which the curve develops
    //the desired passiveTorque
    double normPassiveTorque = passiveTorqueTarget
                              /maxActiveIsometricTorque;
    //Ge a good initial guess

    VectorNd curveDomain = tpCurve.getCurveDomain();
    double angleRange = abs( curveDomain[1]-curveDomain[0]);
    double fiberAngle = 0.5*(curveDomain[0]+curveDomain[1]);
    double jointAngleCurr = calcJointAngle(fiberAngle);

    TorqueMuscleInfo tqInfo = TorqueMuscleInfo();

    calcTorqueMuscleInfo(jointAngleCurr,0.,0.,tqInfo);
    double err = tqInfo.fiberPassiveTorqueAngleMultiplier-normPassiveTorque;
    double jointAngleLeft   = 0;
    double jointAngleRight  = 0;
    double errLeft          = 0;
    double errRight         = 0;

    double h = 0.25*angleRange;

    //Get a good initial guess - necessary because these curves
    //can be *very* nonlinear.
    int iter    = 0;
    int iterMax = 10;
    int tol = sqrt(SQRTEPSILON);

    while(iter < iterMax && abs(err) > tol){
      jointAngleLeft  = jointAngleCurr-h;
      jointAngleRight = jointAngleCurr+h;

      calcTorqueMuscleInfo(jointAngleLeft,0.,0.,tqInfo);
      errLeft  = tqInfo.fiberPassiveTorqueAngleMultiplier
          - normPassiveTorque;

      calcTorqueMuscleInfo(jointAngleRight,0.,0.,tqInfo);
      errRight = tqInfo.fiberPassiveTorqueAngleMultiplier
          - normPassiveTorque;

      if(abs(errLeft)<abs(err) && abs(errLeft)<abs(errRight)){
        err = errLeft;
        jointAngleCurr = jointAngleLeft;
      }
      if(abs(errRight)<abs(err) && abs(errRight)<abs(errLeft)){
        err = errRight;
        jointAngleCurr = jointAngleRight;
      }
      h = h/2.0;
      ++iter;
    }


    //Use Newton's method to polish up this initial guess.
    iter = 0;
    err  = SQRTEPSILON*2;
    double derr = 0;
    double delta= 0;

    while(abs(err) > SQRTEPSILON && iter < iterMax){
      calcTorqueMuscleInfo(jointAngleCurr,0.,0.,tqInfo);
      err  = tqInfo.fiberPassiveTorqueAngleMultiplier
             -normPassiveTorque;
      derr =  signOfJointTorque*tqInfo.DjointTorqueDjointAngle
              / maxActiveIsometricTorque ;
      delta = -err/derr;
      jointAngleCurr += delta;
      ++iter;
    }

    if(abs(err)>SQRTEPSILON){
      cerr   << "Millard2016TorqueMuscle::"
             << "fitPassiveCurveAngleOffset:"
             << muscleName
             << ": failed to fit the passive curve offset "
             << " such that the curve develops the desired "
             << " passive torque at the given joint angle. This"
             << " should not be possible - contact the maintainer "
             << " of this addon."
             << endl;
      assert(0);
      abort();
    }
    double fiberAngleOffset = calcFiberAngle(jointAngleTarget)
                            - calcFiberAngle(jointAngleCurr);

    setPassiveCurveAngleOffset(fiberAngleOffset);

}

void Millard2016TorqueMuscle::
    fitPassiveTorqueScale(double jointAngleTarget,
                          double passiveTorqueTarget)
{
    muscleCurvesAreDirty = true;
    setPassiveTorqueScale(1.0);

    VectorNd curveDomain = tpCurve.getCurveDomain();
    VectorNd curveRange = VectorNd(2);
    curveRange[0] = tpCurve.calcValue(curveDomain[0]);
    curveRange[1] = tpCurve.calcValue(curveDomain[1]);
    double fiberAngle = calcFiberAngle(jointAngleTarget);

    if(  (fiberAngle < curveDomain[0] && (curveRange[0] < curveRange[1])) 
      || (fiberAngle > curveDomain[1] && (curveRange[1] < curveRange[0])) ){
      cerr   << "Millard2016TorqueMuscle::"
             << "fitPassiveTorqueScale:"
             << muscleName
             << ": joint angle is " << jointAngleTarget
             << " but it should be between "
             << calcJointAngle(curveDomain[0]) << " and "
             << calcJointAngle(curveDomain[1])
             << endl;
      assert(0);
      abort();
    }

    if(passiveTorqueTarget < SQRTEPSILON){
      cerr   << "Millard2016TorqueMuscle::"
             << "fitPassiveTorqueScale:"
             << muscleName
             << ": passiveTorque " << passiveTorqueTarget
             << " but it should be greater than sqrt(eps)"
             << endl;
      assert(0);
      abort();
    }

    double normPassiveTorque = passiveTorqueTarget/maxActiveIsometricTorque;
    int iter    = 0;
    int iterMax = 10;
    double err  = SQRTEPSILON*2;
    double derr = 0;
    double delta= 0;
    double scale = 1.0;
    setPassiveTorqueScale(scale);
    TorqueMuscleInfo tqInfo = TorqueMuscleInfo();

    while(abs(err) > SQRTEPSILON && iter < iterMax){
      setPassiveTorqueScale(scale);
      calcTorqueMuscleInfo(jointAngleTarget,0,0,tqInfo);
      err = tqInfo.fiberPassiveTorqueAngleMultiplier - normPassiveTorque;
      derr= tqInfo.fiberPassiveTorqueAngleMultiplier/scale;
      delta = -err/derr;
      scale += delta;

      if(scale < SQRTEPSILON){
        scale = SQRTEPSILON;
      }
      iter++;
    }

    if(abs(err) > SQRTEPSILON){
      cerr   << "Millard2016TorqueMuscle::"
             << "fitPassiveTorqueScale:"
             << muscleName
             << ": passiveTorqueScale could not be fit to"
             << " the data given. See the maintainer of this"
             << " addon for help."
             << endl;
      assert(0);
      abort();
    }


}

/*
const RigidBodyDynamics::Math::VectorNd&
    Millard2016TorqueMuscle::getParametersc1c2c3c4c5c6()
{
    return c1c2c3c4c5c6Anderson2007;
}

const RigidBodyDynamics::Math::VectorNd&
    Millard2016TorqueMuscle::getParametersb1k1b2k2()
{
    return b1k1b2k2Anderson2007;
}
*/

const SmoothSegmentedFunction& Millard2016TorqueMuscle::
    getActiveTorqueAngleCurve() const
{
  //This must be updated if the parameters have changed
  if(muscleCurvesAreDirty){
    Millard2016TorqueMuscle* mutableThis =
        const_cast<Millard2016TorqueMuscle* >(this);
    mutableThis->updateTorqueMuscleCurves();
  }
  return taCurve;
}

const SmoothSegmentedFunction& Millard2016TorqueMuscle::
getPassiveTorqueAngleCurve() const
{
  //This must be updated if the parameters have changed
  if(muscleCurvesAreDirty){
    Millard2016TorqueMuscle* mutableThis =
        const_cast<Millard2016TorqueMuscle* >(this);
    mutableThis->updateTorqueMuscleCurves();
  }
  return tpCurve;
}

const SmoothSegmentedFunction& Millard2016TorqueMuscle::
getTorqueAngularVelocityCurve() const
{
  //This must be updated if the parameters have changed
  if(muscleCurvesAreDirty){
    Millard2016TorqueMuscle* mutableThis =
        const_cast<Millard2016TorqueMuscle* >(this);
    mutableThis->updateTorqueMuscleCurves();
  }
  return tvCurve;
}


string Millard2016TorqueMuscle::getName(){
  return muscleName;
}

void Millard2016TorqueMuscle::setName(string &name){
  muscleCurvesAreDirty = true;
  muscleName = name;
}



/*************************************************************
 Utilities
*************************************************************/
//fa = signJointAngle*(ja - jaO)
// ja = signJointAngle*fa + ja0
//dja = signJointAngle*dfa

double Millard2016TorqueMuscle::
    calcFiberAngle(double jointAngle) const
{
    return signOfJointAngle*(jointAngle-angleOffset);
}

double Millard2016TorqueMuscle::
    calcJointAngle(double fiberAngle) const
{
    return fiberAngle*signOfJointAngle + angleOffset;
}


double Millard2016TorqueMuscle::
    calcFiberAngularVelocity(double jointAngularVelocity) const
{
    return signOfConcentricAnglularVelocity*jointAngularVelocity;
}

double Millard2016TorqueMuscle::
    calcJointAngularVelocity(double fiberAngularVelocity) const
{
    return signOfConcentricAnglularVelocity*fiberAngularVelocity;
}

void Millard2016TorqueMuscle::updateTorqueMuscleCurves()
{
  std::string tempName = muscleName;

  switch(dataSet){
    case DataSet::Anderson2007:
    {
      double c4 = c1c2c3c4c5c6Anderson2007[3];
      double c5 = c1c2c3c4c5c6Anderson2007[4];

      if(useTabularOmegaMax){
        omegaMax = abs( 2.0*c4*c5/(c5-3.0*c4) );
      }

      scaleFactorAnderson2007  = subjectHeightInMeters
                                  *subjectMassInKg
                                  *gravity;

      if(useTabularMaxActiveIsometricTorque){
        maxActiveIsometricTorque = scaleFactorAnderson2007
                              *c1c2c3c4c5c6Anderson2007[0];
      }

      angleAtOneNormActiveTorque = c1c2c3c4c5c6Anderson2007[2];

      TorqueMuscleFunctionFactory::
        createAnderson2007ActiveTorqueAngleCurve(
            c1c2c3c4c5c6Anderson2007[1],
            c1c2c3c4c5c6Anderson2007[2],
            tempName.append("_taCurve"),
            taCurve);

      tempName = muscleName;

      TorqueMuscleFunctionFactory::
        createAnderson2007ActiveTorqueVelocityCurve(
          c1c2c3c4c5c6Anderson2007[3],
          c1c2c3c4c5c6Anderson2007[4],
          c1c2c3c4c5c6Anderson2007[5],
          1.1,
          1.4,
          tempName.append("_tvCurve"),
          tvCurve);

      tempName = muscleName;

      double normMaxActiveIsometricTorque = maxActiveIsometricTorque
                                            /scaleFactorAnderson2007;

      TorqueMuscleFunctionFactory::
        createAnderson2007PassiveTorqueAngleCurve(
          scaleFactorAnderson2007,
          normMaxActiveIsometricTorque,
          b1k1b2k2Anderson2007[0],
          b1k1b2k2Anderson2007[1],
          b1k1b2k2Anderson2007[2],
          b1k1b2k2Anderson2007[3],
          tempName.append("_tpCurve"),
          tpCurve);

      tpCurve.shift(passiveCurveAngleOffset,0);
      tpCurve.scale(1.0,passiveTorqueScale);

      double k = 0;
      double b = 0;

      if(b1k1b2k2Anderson2007[0] > 0){
        b = b1k1b2k2Anderson2007[0];
        k = b1k1b2k2Anderson2007[1];
      }else if(b1k1b2k2Anderson2007[2] > 0){
        b = b1k1b2k2Anderson2007[2];
        k = b1k1b2k2Anderson2007[3];
      }

      if(abs(b) > 0 && passiveTorqueScale > SQRTEPSILON){
          angleAtOneNormPassiveTorque =
              (1/k)*log(abs(maxActiveIsometricTorque/b))
              + passiveCurveAngleOffset;
      }else{
          angleAtOneNormPassiveTorque =
              std::numeric_limits<double>::signaling_NaN();
      }


      //angleAtOneNormPassiveTorque
      //gymnastParams[Gymnast::PassiveAngleAtOneNormTorque]


    } break;
    case DataSet::Gymnast:
      {
        if(useTabularOmegaMax){
          omegaMax                  = gymnastParams[
                                        Gymnast::OmegaMax];
        }
        if(useTabularMaxActiveIsometricTorque){
          maxActiveIsometricTorque  = gymnastParams[
                                        Gymnast::TauMax];
        }
        angleAtOneNormActiveTorque = gymnastParams[
                                      Gymnast::ActiveAngleAtOneNormTorque];

        TorqueMuscleFunctionFactory::
            createGaussianShapedActiveTorqueAngleCurve(
              gymnastParams[Gymnast::ActiveAngleAtOneNormTorque],
              gymnastParams[Gymnast::ActiveAngularStandardDeviation],
              tempName.append("_taCurve"),
              taCurve);

        TorqueMuscleFunctionFactory::createPassiveTorqueAngleCurve(
              gymnastParams[Gymnast::PassiveAngleAtZeroTorque],
              gymnastParams[Gymnast::PassiveAngleAtOneNormTorque],
              tempName.append("_tpCurve"),
              tpCurve);

        tpCurve.shift(passiveCurveAngleOffset,0);
        tpCurve.scale(1.0,passiveTorqueScale);

        TorqueMuscleFunctionFactory::createTorqueVelocityCurve(
              gymnastParams[Gymnast::TvAtMaxEccentricVelocity],
              gymnastParams[Gymnast::TvAtHalfMaxConcentricVelocity],
              tempName.append("_tvCurve"),
              tvCurve);

        if(passiveTorqueScale > 0){
          angleAtOneNormPassiveTorque =
            gymnastParams[Gymnast::PassiveAngleAtOneNormTorque]
            + passiveCurveAngleOffset;
        }else{
          angleAtOneNormPassiveTorque =
                  std::numeric_limits<double>::signaling_NaN();
        }

      } break;
    default:
    {
      cerr << "Millard2016TorqueMuscle::"
             << "Millard2016TorqueMuscle:"
             << muscleName
             << "dataSet has a value of " << dataSet
             << " which is not a valid choice";
      assert(0);
      abort();
    }
  };



  //If the passiveScale is < 1 and > 0, then we must iterate to
  //find the true value of angleAtOneNormPassiveTorque;

  if(isfinite(angleAtOneNormPassiveTorque)
     && (passiveTorqueScale > 0 && passiveTorqueScale != 1.0) ){
    int iter = 1;
    int iterMax =100;
    double err = 10*SQRTEPSILON;
    double derr = 0;
    double delta = 0;

    while(abs(err) > SQRTEPSILON && iter < iterMax){

      err = tpCurve.calcValue(angleAtOneNormPassiveTorque)
            - 1.0;
      derr = tpCurve.calcDerivative(angleAtOneNormPassiveTorque,1);

      delta = -err/derr;
      angleAtOneNormPassiveTorque += delta;
      ++iterMax;
    }

    if(abs(err) > SQRTEPSILON){
      cerr << "Millard2016TorqueMuscle::"
             << "Millard2016TorqueMuscle:"
             << muscleName
             << "Internal Error: failed to solve for "
             <<"angleAtOneNormPassiveTorque. Consult the maintainer of this "
             <<"addon";
      assert(0);
      abort();
    }

  }

  //Update the damping curve.
  tempName = muscleName;
  TorqueMuscleFunctionFactory::createDampingBlendingCurve(
                -1.0,
                tempName.append("_dampingCurve"),
                betaCurve);

  muscleCurvesAreDirty = false;
}



void Millard2016TorqueMuscle::printJointTorqueProfileToFile(
        const std::string& path,
        const std::string& fileNameWithoutExtension,
        int numberOfSamplePoints)
{
    if(muscleCurvesAreDirty){
      updateTorqueMuscleCurves();
    }

    VectorNd activeDomain  = taCurve.getCurveDomain();
    VectorNd passiveDomain = tpCurve.getCurveDomain();
    VectorNd velocityDomain= tvCurve.getCurveDomain();

    double angleMin = activeDomain[0];
    double angleMax = activeDomain[1];

    if(tpCurve.calcValue(passiveDomain[0]) >= 0.99){
        angleMin = passiveDomain[0];
    }

    if(tpCurve.calcValue(passiveDomain[1]) >= 0.99){
        angleMax = passiveDomain[1];
    }

    double jointMin = signOfJointAngle*angleMin + angleOffset;
    double jointMax = signOfJointAngle*angleMax + angleOffset;

    if(jointMin > jointMax){
        double tmp = jointMin;
        jointMin=jointMax;
        jointMax=tmp;
    }
    double range = jointMax-jointMin;
    jointMin = jointMin -range*0.1;
    jointMax = jointMax +range*0.1;
    double jointDelta = (jointMax-jointMin)
                        /((double)numberOfSamplePoints-1.);

    double velMin = omegaMax*signOfConcentricAnglularVelocity*velocityDomain[0];
    double velMax = omegaMax*signOfConcentricAnglularVelocity*velocityDomain[1];

    if(velMin > velMax){
        double tmp = velMin;
        velMin = velMax;
        velMax = tmp;
    }
    double velRange = velMax-velMin;
    velMin = velMin-0.1*velRange;
    velMax = velMax+0.1*velRange;
    double velDelta = (velMax-velMin)/((double)numberOfSamplePoints-1.0);

    double angleAtMaxIsoTorque = angleAtOneNormActiveTorque;

    std::vector< std::vector < double > > matrix;
    std::vector < double > row(21);
    std::string header("jointAngle,"
                       "jointVelocity,"
                       "activation,"
                       "fiberAngle,"
                       "fiberAngularVelocity,"
                       "passiveTorqueAngleMultiplier,"
                       "activeTorqueAngleMultiplier,"
                       "torqueVelocityMultiplier,"
                       "activeTorque,"
                       "passiveTorque,"
                       "fiberTorque,"
                       "jointTorque,"
                       "fiberStiffness,"
                       "jointStiffness,"
                       "fiberActivePower,"
                       "fiberPassivePower,"
                       "fiberPower,"
                       "jointPower,"
                       "DjointTorqueDactivation,"
                       "DjointTorqueDjointAngularVelocity,"
                       "DjointTorqueDjointAngle");

    double activation  =1.0;
    double jointAngle  = 0.;
    double jointVelocity = 0.;


    for(int i=0; i<numberOfSamplePoints; ++i){
        jointAngle = jointMin + i*jointDelta;
        jointVelocity = 0.;

        calcTorqueMuscleInfo(jointAngle,
                             jointVelocity,
                             activation,
                             tmInfo);

        row.at(0) = jointAngle;
        row.at(1) = jointVelocity;
        row.at(2) = activation;

        row.at(3) = tmInfo.fiberAngle;
        row.at(4) = tmInfo.fiberAngularVelocity;
        row.at(5) = tmInfo.fiberPassiveTorqueAngleMultiplier;
        row.at(6) = tmInfo.fiberActiveTorqueAngleMultiplier;
        row.at(7) = tmInfo.fiberActiveTorqueAngularVelocityMultiplier;
        row.at(8) = tmInfo.fiberActiveTorque;
        row.at(9) = tmInfo.fiberPassiveTorque;
        row.at(10)= tmInfo.fiberTorque;
        row.at(11)= tmInfo.jointTorque;
        row.at(12)= tmInfo.fiberStiffness;
        row.at(13)= tmInfo.jointStiffness;
        row.at(14)= tmInfo.fiberActivePower;
        row.at(15)= tmInfo.fiberPassivePower;
        row.at(16)= tmInfo.fiberPower;
        row.at(17)= tmInfo.jointPower;
        row.at(18)= tmInfo.DjointTorqueDactivation;
        row.at(19)= tmInfo.DjointTorqueDjointAngularVelocity;
        row.at(20)= tmInfo.DjointTorqueDjointAngle;

        matrix.push_back(row);
    }

    std::string fullFilePath = path;
    if(!path.empty()){
        fullFilePath.append("/");
    }


    fullFilePath.append(fileNameWithoutExtension);
    fullFilePath.append("_variableLengthfixedVelocity");
    fullFilePath.append(".csv");
    printMatrixToFile(matrix,header,fullFilePath);

    matrix.clear();


    for(int i=0; i<numberOfSamplePoints; ++i){

        jointAngle = angleAtMaxIsoTorque;
        jointVelocity = velMin + i*velDelta;

        calcTorqueMuscleInfo(jointAngle,
                             jointVelocity,
                             activation,
                             tmInfo);

        row.at(0) = jointAngle;
        row.at(1) = jointVelocity;
        row.at(2) = activation;

        row.at(3) = tmInfo.fiberAngle;
        row.at(4) = tmInfo.fiberAngularVelocity;
        row.at(5) = tmInfo.fiberPassiveTorqueAngleMultiplier;
        row.at(6) = tmInfo.fiberActiveTorqueAngleMultiplier;
        row.at(7) = tmInfo.fiberActiveTorqueAngularVelocityMultiplier;
        row.at(8) = tmInfo.fiberActiveTorque;
        row.at(9) = tmInfo.fiberPassiveTorque;
        row.at(10)= tmInfo.fiberTorque;
        row.at(11)= tmInfo.jointTorque;
        row.at(12)= tmInfo.fiberStiffness;
        row.at(13)= tmInfo.jointStiffness;
        row.at(14)= tmInfo.fiberActivePower;
        row.at(15)= tmInfo.fiberPassivePower;
        row.at(16)= tmInfo.fiberPower;
        row.at(17)= tmInfo.jointPower;
        row.at(18)= tmInfo.DjointTorqueDactivation;
        row.at(19)= tmInfo.DjointTorqueDjointAngularVelocity;
        row.at(20)= tmInfo.DjointTorqueDjointAngle;

        matrix.push_back(row);
    }

    fullFilePath = path;
    if(!path.empty()){
        fullFilePath.append("/");
    }
    fullFilePath.append(fileNameWithoutExtension);
    fullFilePath.append("_fixedLengthVariableVelocity");
    fullFilePath.append(".csv");
    printMatrixToFile(matrix,header,fullFilePath);

    matrix.clear();

}



