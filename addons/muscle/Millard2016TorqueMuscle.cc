//==============================================================================
/* 
 * RBDL - Rigid Body Dynamics Library: Addon : forceElements
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
                                        "WristSupination"            };

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
                                        "WristSupination"            };

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

double const Millard2016TorqueMuscle::GymnastWholeBody[20][14] =
{ {0,0,0,0,0.14798,0.84726,1.0647,0.55326,1.4785,0.5058,0,0,0,0               }, 
  {0,1,0,0,0.13244,0.73574,0.73304,0.91804,2.2951,0.49808,0.60283,-10.8,0,0   }, 
  {1,0,0,0,0.24049,1.7613,0.94248,0.80006,2.2157,0.50718,0,0,0,0              }, 
  {1,1,0,0,0.083153,0.80863,1.0297,1.2608,3.2844,0.50349,0,0,0,0              }, 
  {2,0,0,0,0.1074,1.391,0.408,0.987,3.558,0.295,-0.0005781,-5.819,0.967,6.09  }, 
  {2,1,0,0,0.037309,1.51,-0.187,0.699,1.94,0.828,0.0005781,-5.819,-0.967,6.09 }, 
  {3,0,0,0,0.10727,0.49026,2.1468,1.6259,4.0644,0.50288,2.219e-08,6.315,0,0   }, 
  {3,1,0,0,0.076738,0.69592,0.89012,1.9045,4.7626,0.49695,0,0,0,0             }, 
  {5,0,0,0,0.013106,0.64609,1.5533,1.5224,4.216,0.4774,0,0,0,0                }, 
  {5,1,0,0,0.033027,0.52478,0.66323,1.5173,4.2014,0.49657,0.0021712,-6.547,0,0}, 
  {3,2,0,0,0.10819,0.76953,0.8392,1.5532,3.9754,0.30515,0,0,0,0               }, 
  {3,3,0,0,0.079723,0.41559,-0.27761,1.646,4.1745,0.28031,0,0,0,0             }, 
  {3,5,0,0,0.042539,0.35124,-1.1876,2.701,6.343,0.20675,0,0,0,0               }, 
  {3,4,0,0,0.036697,0.49673,-0.6708,1.5889,4.0515,0.33687,0,0,0,0             }, 
  {4,1,0,0,0.085364,0.27207,0.33,1.3059,3.4237,0.39774,0,0,0,0                }, 
  {4,0,0,0,0.058832,0.75333,1.6432,1.3456,3.5229,0.36698,0,0,0,0              }, 
  {5,6,0,0,0.011397,0.73677,-0.2092,3.6803,9.1426,0.16296,0,0,0,0             }, 
  {5,7,0,0,0.010926,0.99346,-0.21239,2.2211,5.7475,0.28206,0,0,0,0            }, 
  {5,9,0,0,0.026457,0.66643,0.43,1.3635,3.5528,0.37518,0,0,0,0                }, 
  {5,8,0,0,0.020068,0.38476,-1.1432,1.7767,4.5821,0.29678,0,0,0,0            }};

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


const static int JointTorqueMap[20][3] = {
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
  {(int)JointTorqueSet::WristSupination             , (int)JointSet::Wrist    , (int)DirectionSet::Supination         }
  };


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
  muscleName(name)
{

    double subjectHeightInMeters  = subjectInfo.heightInMeters;
    double subjectMassInKg        = subjectInfo.massInKg;
    int gender                    = (int) subjectInfo.gender;
    int ageGroup                  = (int) subjectInfo.ageGroup;

    int joint           = -1;
    int jointDirection  = -1;

    for(int i=0; i < 20; ++i){
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

    //int jointHip0Knee1Ankle2,
    //int directionExtension0Flexion1,
    //int genderMale0Female1,
    //int ageYoung0Mid1Senior2,

    //Go and find the coefficients the user wants
    c1c2c3c4c5c6.resize(6);
    b1k1b2k2.resize(4);


    switch(dataSet){
      case DataSet::Anderson2007:
      {
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
              c1c2c3c4c5c6[i] = Anderson2007Table3Mean[idx][i+4];
          }
          for(int i=0; i<4; ++i){
              b1k1b2k2[i] = Anderson2007Table3Mean[idx][i+10];
          }
        }

      } break;

    case DataSet::Gymnast:
      {

        for(int i=0; i<20;++i){

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
          for(int i=0; i<6; ++i){
              c1c2c3c4c5c6[i] = GymnastWholeBody[idx][i+4];
          }
          for(int i=0; i<4; ++i){
              b1k1b2k2[i] = GymnastWholeBody[idx][i+10];
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

    strengthScaleFactor      = subjectHeightInMeters*subjectMassInKg
                               *gravity;
    maxActiveIsometricTorque = strengthScaleFactor*c1c2c3c4c5c6[0];

    muscleCurvesAreDirty = true;
    updateTorqueMuscleCurves();
    passiveTorqueScale = 1.0;

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
    double fiberVelocity = calcFiberAngularVelocity(jointAngularVelocity);
    double ta = taCurve.calcValue(fiberAngle);
    double tp = tpCurve.calcValue(fiberAngle);
    double tv = tvCurve.calcValue(fiberVelocity);
    
    double jointTorque = signOfJointTorque
                        * maxActiveIsometricTorque*(
                              activation*ta*tv
                            + passiveTorqueScale*tp );
    return jointTorque;
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
    double tp = passiveTorqueScale*tpCurve.calcValue(fiberAngle);
    double tv = tvCurve.calcValue(fiberAngularVelocity);

    double D_ta_DfiberAngle = taCurve.calcDerivative(fiberAngle,1);
    double D_tp_DfiberAngle = passiveTorqueScale*tpCurve.calcDerivative(fiberAngle,1);
    double D_tv_DfiberAngularVelocity = tvCurve.calcDerivative(fiberAngularVelocity,1);

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
    tmi.fiberPassiveTorque   = maxActiveIsometricTorque*(tp);
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
            * D_tv_DfiberAngularVelocity_D_jointAngularVelocity);

    tmi.DjointTorqueDjointAngle           =
        signOfJointTorque
        * maxActiveIsometricTorque
        * ( activation
            *D_ta_DfiberAngle
            * tv
            + D_tp_DfiberAngle
         )* D_fiberAngle_D_jointAngle;

}


/*************************************************************
 Get / Set Functions
*************************************************************/

double Millard2016TorqueMuscle::
    getMaximumActiveIsometricTorque() const 
{  
    return maxActiveIsometricTorque;
}

void Millard2016TorqueMuscle::
    setMaximumActiveIsometricTorque(double maxIsoTorque)
{
    muscleCurvesAreDirty = true;
    maxActiveIsometricTorque = maxIsoTorque;
}    



double Millard2016TorqueMuscle::
    getPassiveTorqueScale() const
{
    return passiveTorqueScale;
}

void Millard2016TorqueMuscle::
    setPassiveTorqueScale(double passiveTorqueScaling)
{
    //This does not affect the curves.
    passiveTorqueScale = passiveTorqueScaling;
}

const RigidBodyDynamics::Math::VectorNd&
    Millard2016TorqueMuscle::getParametersC1C2C3C4C5C6()
{
    return c1c2c3c4c5c6;
}

const RigidBodyDynamics::Math::VectorNd&
    Millard2016TorqueMuscle::getParametersB1K1B2K2()
{
    return b1k1b2k2;
}


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
    calcFiberAngularVelocity(double jointAngularVelocity) const
{
    return signOfConcentricAnglularVelocity*jointAngularVelocity;
}

void Millard2016TorqueMuscle::updateTorqueMuscleCurves()
{
  std::string tempName = muscleName;

  TorqueMuscleFunctionFactory::
    createAnderson2007ActiveTorqueAngleCurve(
        c1c2c3c4c5c6[1],
        c1c2c3c4c5c6[2],
        tempName.append("_taCurve"),
        taCurve);

  tempName = muscleName;

  TorqueMuscleFunctionFactory::
    createAnderson2007ActiveTorqueVelocityCurve(
      c1c2c3c4c5c6[3],
      c1c2c3c4c5c6[4],
      c1c2c3c4c5c6[5],
      1.1,
      1.4,
      tempName.append("_tvCurve"),
      tvCurve);

  tempName = muscleName;

  double normMaxActiveIsometricTorque = maxActiveIsometricTorque
                                        /strengthScaleFactor;

  TorqueMuscleFunctionFactory::
    createAnderson2007PassiveTorqueAngleCurve(
      strengthScaleFactor,
      normMaxActiveIsometricTorque,
      b1k1b2k2[0],
      b1k1b2k2[1],
      b1k1b2k2[2],
      b1k1b2k2[3],
      tempName.append("_tpCurve"),
      tpCurve);

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

    double velMin = signOfConcentricAnglularVelocity*velocityDomain[0];
    double velMax = signOfConcentricAnglularVelocity*velocityDomain[1];

    if(velMin > velMax){
        double tmp = velMin;
        velMin = velMax;
        velMax = tmp;
    }
    double velRange = velMax-velMin;
    velMin = velMin-0.1*velRange;
    velMax = velMax+0.1*velRange;
    double velDelta = (velMax-velMin)/((double)numberOfSamplePoints-1.0);

    double angleAtMaxIsoTorque = c1c2c3c4c5c6[2];

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



