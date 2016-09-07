/*
 *  particle_filter.h
 *
 *  Created on: Mar 24, 2011
 *      Author: Frederik Hegger
 */

#ifndef TRACKINGPARTICLEFILTER_H_
#define TRACKINGPARTICLEFILTER_H_

/* standard deviations for gaussian sampling during initialization */
#define INIT_X_STD 0.05
#define INIT_Y_STD 0.05
//#define INIT_V_STD 0.1

/* standard deviations for gaussian sampling in transition model */
#define TRANS_X_STD 0.05
#define TRANS_Y_STD 0.05
//#define TRANS_V_STD 0.1

/* standard deviations for gaussian sampling in system model */
#define SYSTEM_X_STD 0.1
#define SYSTEM_Y_STD 0.1
//#define SYSTEM_V_STD 0.1

#define OBSERV_STD      0.3

/* autoregressive dynamics parameters for transition model */
#define PARAM_A1  2.0//2.0
#define PARAM_A2  -1.0//-1.0
#define PARAM_B0  1.0//1.0000

#include <iostream>
#include <math.h>
#include <stdlib.h>
#include <vector>

#include <boost/random.hpp>
#include <time.h>

#include <mcr_perception_msgs/LaserScanSegmentList.h>
#include <mcr_perception_msgs/LaserScanSegment.h>

using namespace std;

struct StrParticle
{
    StrParticle()
        : dX(0),
          dY(0),
          dPrevX(0),
          dPrevY(0),
          dOrgX(0),
          dOrgY(0),
          dWeight(0),
          iCorrespondToObj(-1)
    {
    }
    ;

    double dX;              // current x coordinate of the COG
    double dY;              // current x coordinate of the COG
    double dVx;             // current velocity in x direction of the COG
    double dVy;             // current velocity in y direction of the COG
    double dPrevX;          // previous x coordinate of the COG
    double dPrevY;          // previous y coordinate of the COG
    double dPrevVx;         // previous velocity in x direction of the COG
    double dPrevVy;         // previous velocity in y direction of the COG
    double dOrgX;           // original x coordinate of the COG
    double dOrgY;           // original x coordinate of the COG
    double dOrgVx;          // original velocity in x direction of the COG
    double dOrgVy;          // original velocity in y direction of the COG
    double dWeight;         // weight
    int iCorrespondToObj;   // determines to which object this particle belongs
    long double ldTimestep;  // time step between two state transitions

    static bool SortByWeights(const StrParticle* particleA, const StrParticle* particleB)
    {
        return particleA->dWeight > particleB->dWeight;
    }

};

struct StrPoint
{
    /** Constructor. Initializes all values to 0. */
    StrPoint()
        : dDistance(0),
          dRoll(0),
          dPitch(0),
          dYaw(0),
          dX(0),
          dY(0),
          dZ(0)
    {
    }
    ;
    /** Distance to point */
    double dDistance;
    /** Orientation of point -- Rotation around X-axis */
    double dRoll;
    /** Orientation of point -- Rotation around Y-axis */
    double dPitch;
    /** Orientation of point -- Rotation around Z-axis */
    double dYaw;
    /** Position of point -- Position on X-axis */
    double dX;
    /** Position of point -- Position on Y-axis */
    double dY;
    /** Position of point -- Position on Z-axis */
    double dZ;
};

class TrackingParticleFilter
{
public:
    /*
     * Constructor of TrackingParticleFilter
     *
     * @param unNumberOfParticles the number of particles which should be created/used
     *
     */
    TrackingParticleFilter(unsigned int unNumberOfParticles);

    /*
     * Destructor of TrackingParticleFilter
     *
     * cleans up eyerthing
     */
    virtual ~TrackingParticleFilter();

    /*
     * establish an initial distribution of particles based on the first measurements
     *
     * @param vecMeasurements a vector of measurements
     * @return returns 0 if the initialization was successful
     */
    int initialize(mcr_perception_msgs::LaserScanSegmentList vecMeasurements);

    /*
     * perform the state transition for each particle in the particle set
     *
     * @return returns 0 if the prediction was successful
     */
    int predict();

    /*
     * performs an update of all particles the current observation
     *
     * @param vecMeasurements a vector of measurements
     * @return returns 0 if the update was successful
     */
    int update(mcr_perception_msgs::LaserScanSegmentList vecMeasurements);

    /*
     * get the current particles
     *
     * @return returns the complete set of particles
     */
    vector<StrParticle*> getParticles()
    {
        return this->_vecParticleSet;
    }

    void getPersonEstimates();
    StrPoint* getMostLikelyParticle();
    StrPoint* getMostLikelyPosition();
    //strParticle* predictAndUpdate();
    //strParticle* update(){};

private:
    /*
     * calculates the state transition for a given particle
     *
     * @param strParticle a single particle
     *        ldTimestep time in milliseconds between two predictions
     * @return returns 0 if the state transition was successful
     */
    int stateTransition(StrParticle *strParticle);

    //returns summed particle weights (not normalized)
    double oberservationLikelihood(mcr_perception_msgs::LaserScanSegmentList vecMeasurements);
    int normalizeParticleWeights(double dSummedWeights);
    int resampleParticles();

    double getGaussian2D(double dXValue, double dYValue, double dXcenter, double dYCenter, double dXSigma, double dYSigma, double dTheta);
    double getRandomNoise(double dDeviation);

    //void normalize_weights( particle* particles, int n );
    //strParticles* resample( particle* particles, int n );
    //int particle_cmp( void* p1, void* p2 );
    //void display_particle( IplImage* img, particle p, CvScalar color );

    /*
     * holds the overall number of particles
     */
    unsigned int _unNumberOfParticles;

    /*
     * storage of all particles
     */
    vector<StrParticle*> _vecParticleSet;

    /*
     * random number generator
     */
    boost::mt19937 _pRandomNumberGenerator;
};

#endif /* TRACKINGPARTICLEFILTER_H_ */
