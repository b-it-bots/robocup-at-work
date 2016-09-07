/*
 * particle_filter.cpp
 *
 *  Created on: Mar 24, 2010
 *      Author: Frederik Hegger
 */

#include "mcr_people_tracking/particle_filter.h"

TrackingParticleFilter::TrackingParticleFilter(unsigned int unNumberOfParticles)
{
    this->_unNumberOfParticles = unNumberOfParticles;

    _pRandomNumberGenerator.seed(time(0));
}

TrackingParticleFilter::~TrackingParticleFilter()
{
    for (unsigned int i = 0; i < this->_vecParticleSet.size(); ++i)
        free(this->_vecParticleSet.at(i));
}

double TrackingParticleFilter::getRandomNoise(double dDeviation)
{
    dDeviation /= 2;

    boost::uniform_real<> gm(-dDeviation, dDeviation);
    boost::variate_generator<boost::mt19937&, boost::uniform_real<> > generate(_pRandomNumberGenerator, gm);

    return generate();
}

int TrackingParticleFilter::initialize(mcr_perception_msgs::LaserScanSegmentList vecMeasurements)
{
    unsigned int iPriorParticlesPerMeasurement = 0;
    unsigned int i = 0, k = 0;
    StrParticle *strParticle = NULL;

    //clear particle vector
    this->_vecParticleSet.clear();

    //initial number of particles per detected person
    iPriorParticlesPerMeasurement = this->_unNumberOfParticles / vecMeasurements.segments.size();

    //distribute particles uniformly over all initial observations
    for (i = 0, k = 0; i < vecMeasurements.segments.size(); ++i)
    {
        for (unsigned int j = 0; j < iPriorParticlesPerMeasurement; ++j, ++k)
        {
            strParticle = new StrParticle();
            strParticle->dOrgX = strParticle->dPrevX = strParticle->dX = vecMeasurements.segments[i].center.x + this->getRandomNoise(INIT_X_STD);
            strParticle->dOrgY = strParticle->dPrevY = strParticle->dY = vecMeasurements.segments[i].center.y + this->getRandomNoise(INIT_Y_STD);
            strParticle->dOrgVx = strParticle->dPrevVx = strParticle->dVx = 0;
            strParticle->dOrgVy = strParticle->dPrevVy = strParticle->dVx = 0;
            strParticle->dWeight = 0;
            strParticle->iCorrespondToObj = i;
            strParticle->ldTimestep = 0;
            this->_vecParticleSet.push_back(strParticle);
        }
    }

    //assign the remaining particles
    for (i = 0; k < this->_unNumberOfParticles; ++k)
    {
        strParticle = new StrParticle();
        strParticle->dOrgX = strParticle->dPrevX = strParticle->dX = vecMeasurements.segments[i].center.x + this->getRandomNoise(INIT_X_STD);
        strParticle->dOrgY = strParticle->dPrevY = strParticle->dY = vecMeasurements.segments[i].center.y + this->getRandomNoise(INIT_Y_STD);
        strParticle->dOrgVx = strParticle->dPrevVx = strParticle->dVx = 0;
        strParticle->dOrgVy = strParticle->dPrevVy = strParticle->dVx = 0;
        strParticle->dWeight = 0;
        strParticle->iCorrespondToObj = i;
        this->_vecParticleSet.push_back(strParticle);

        ++k;
        i = (i + 1) % this->_unNumberOfParticles;
    }

    return 0;
}

int TrackingParticleFilter::predict()
{
    // performs the prediction for the complete particle set
    for (unsigned int i = 0; i < this->_unNumberOfParticles; ++i)
        this->stateTransition(this->_vecParticleSet.at(i));

    return 0;
}

int TrackingParticleFilter::stateTransition(StrParticle *strParticle)
{
    double dNewX = 0, dNewY = 0;
//  double dVx = 0, dVy = 0;
//  const double dGamma = 0.6;

// sample new state using second-order autoregressive dynamics
    dNewX = PARAM_A1 * (strParticle->dX - strParticle->dOrgX) + PARAM_A2 * (strParticle->dPrevX - strParticle->dOrgX)
            + PARAM_B0 * this->getRandomNoise(TRANS_X_STD) + strParticle->dOrgX;
    dNewY = PARAM_A1 * (strParticle->dY - strParticle->dOrgY) + PARAM_A2 * (strParticle->dPrevY - strParticle->dOrgY)
            + PARAM_B0 * this->getRandomNoise(TRANS_Y_STD) + strParticle->dOrgY;

    // calculate velocities
    //dVx = (dGamma * (strParticle->dPrevVx * ldTimestep)) + ((1.0 - dGamma) * (dNewX - strParticle->dX));
    //dVy = (dGamma * (strParticle->dPrevVy * ldTimestep)) + ((1.0 - dGamma) * (dNewY - strParticle->dY));

    //dNewX += dVx;
    //dNewY += dVy;

    strParticle->dPrevX = strParticle->dX;
    strParticle->dPrevY = strParticle->dY;
    strParticle->dX = dNewX;
    strParticle->dY = dNewY;

    strParticle->dPrevVx = strParticle->dVx;
    strParticle->dPrevVy = strParticle->dVy;
    //strParticle->dVx = (dVx / ldTimestep);
    //strParticle->dVy = (dVy / ldTimestep);

    //cout << "vx: " << strParticle->dVx << " vy: " << strParticle->dVy << endl;

    strParticle->dWeight = 0;

    return 0;
}

int TrackingParticleFilter::update(mcr_perception_msgs::LaserScanSegmentList vecMeasurements)
{
    double dSummedWeights = 0;

    dSummedWeights = this->oberservationLikelihood(vecMeasurements);

    if (dSummedWeights > 0)
    {
        this->normalizeParticleWeights(dSummedWeights);
        //this->getPersonEstimates();
        this->resampleParticles();
    }

    return 0;
}

double TrackingParticleFilter::oberservationLikelihood(mcr_perception_msgs::LaserScanSegmentList vecMeasurements)
{
    //double* dObservLikelihood = new double[this->_unNumberOfParticles];
    double dSummedDistances = 0;
    double dMaxParticleLikelihood = 0;
    double dParticleLikelihood = 0;
//  double dEuclDist = 0;

    // calculate distances between particle locations and measurements
    for (unsigned int i = 0; i < this->_unNumberOfParticles; ++i)
    {
        for (unsigned int j = 0; j < vecMeasurements.segments.size(); ++j)
        {
            dParticleLikelihood = this->getGaussian2D(vecMeasurements.segments[j].center.x, vecMeasurements.segments[j].center.y,
                                  this->_vecParticleSet.at(i)->dX, this->_vecParticleSet.at(i)->dY, SYSTEM_X_STD, SYSTEM_Y_STD, 0.0);

            //dEuclDist = getEuclideanDistance2D(vecMeasurements.at(j)->strCenterPoint.dX, vecMeasurements.at(j)->strCenterPoint.dY,
            //                              this->_vecParticleSet.at(i)->dX, this->_vecParticleSet.at(i)->dY);

            //dParticleLikelihood = (1.0 / (OBSERV_STD * sqrt(2 * M_PI))) * exp( (-pow(dEuclDist, 2.0)) / (2.0 * pow(OBSERV_STD, 2.0)) );

            // only take the closest measurement
            if (dParticleLikelihood >= dMaxParticleLikelihood)
            {
                dMaxParticleLikelihood = dParticleLikelihood;
                //  dObservLikelihood[i] = dParticleLikelihood;
                this->_vecParticleSet.at(i)->iCorrespondToObj = j;

            }
        }

        dSummedDistances += dMaxParticleLikelihood;

        this->_vecParticleSet.at(i)->dWeight = dMaxParticleLikelihood;

        dMaxParticleLikelihood = 0;
    }

    return dSummedDistances;
}

void TrackingParticleFilter::getPersonEstimates()
{
    vector<double> vecProbabilityEstimate;

    vecProbabilityEstimate.push_back(0.0);
    vecProbabilityEstimate.push_back(0.0);
    vecProbabilityEstimate.push_back(0.0);

    for (unsigned int i = 0; i < this->_unNumberOfParticles; ++i)
    {
        vecProbabilityEstimate.at(this->_vecParticleSet.at(i)->iCorrespondToObj) += this->_vecParticleSet.at(i)->dWeight;
    }

    for (unsigned int j = 0; j < vecProbabilityEstimate.size(); ++j)
        cout << "id: " << j << " prob: " << vecProbabilityEstimate.at(j) << endl;
}

int TrackingParticleFilter::normalizeParticleWeights(double dSummedWeights)
{
    for (unsigned j = 0; j < this->_unNumberOfParticles; ++j)
    {
        this->_vecParticleSet.at(j)->dWeight /= dSummedWeights;
//      cout << "w: " << this->_vecParticleSet.at(j)->dWeight << endl;
    }

    return 0;
}

int TrackingParticleFilter::resampleParticles()
{
    vector<StrParticle*> vecNewParticles;
    StrParticle* tmpParticle = NULL;
    unsigned int k = 0;
    int iCountNewParticles = 0;

    sort(this->_vecParticleSet.begin(), this->_vecParticleSet.end(), StrParticle::SortByWeights);

    for (unsigned int i = 0; i < this->_unNumberOfParticles; ++i)
    {
        iCountNewParticles = (int) round(this->_vecParticleSet.at(i)->dWeight * this->_unNumberOfParticles);

        //cout << "weight: "  << this->_vecParticleSet.at(i)->dWeight << " newparticles: " << iCountNewParticles << endl;

        for (int j = 0; j < iCountNewParticles; ++j)
        {
            tmpParticle = new StrParticle();
            tmpParticle->dOrgX = this->_vecParticleSet.at(i)->dOrgX;
            tmpParticle->dOrgY = this->_vecParticleSet.at(i)->dOrgY;
            tmpParticle->dPrevX = this->_vecParticleSet.at(i)->dPrevX;
            tmpParticle->dPrevY = this->_vecParticleSet.at(i)->dPrevY;
            tmpParticle->dX = this->_vecParticleSet.at(i)->dX;
            tmpParticle->dY = this->_vecParticleSet.at(i)->dY;
            tmpParticle->dWeight = this->_vecParticleSet.at(i)->dWeight;
            tmpParticle->iCorrespondToObj = this->_vecParticleSet.at(i)->iCorrespondToObj;

            //cout << "id: " << tmpParticle->iCorrespondToObj << endl;

            vecNewParticles.push_back(tmpParticle);

            ++k;

            if (k == this->_unNumberOfParticles)
            {
                for (unsigned int l = 0; l < this->_vecParticleSet.size(); ++l)
                    free(this->_vecParticleSet.at(l));

                this->_vecParticleSet.clear();
                this->_vecParticleSet.assign(vecNewParticles.begin(), vecNewParticles.end());

                return 0;
            }
        }
    }
    while (k < this->_unNumberOfParticles)
    {
        tmpParticle = new StrParticle();

        tmpParticle->dOrgX = this->_vecParticleSet.at(0)->dOrgX;
        tmpParticle->dOrgY = this->_vecParticleSet.at(0)->dOrgY;
        tmpParticle->dPrevX = this->_vecParticleSet.at(0)->dPrevX;
        tmpParticle->dPrevY = this->_vecParticleSet.at(0)->dPrevY;
        tmpParticle->dX = this->_vecParticleSet.at(0)->dX;
        tmpParticle->dY = this->_vecParticleSet.at(0)->dY;
        tmpParticle->dWeight = this->_vecParticleSet.at(0)->dWeight;
        tmpParticle->iCorrespondToObj = this->_vecParticleSet.at(0)->iCorrespondToObj;

        //  cout << "id: " << tmpParticle->iCorrespondToObj << endl;

        vecNewParticles.push_back(tmpParticle);

        ++k;
    }

    for (unsigned int m = 0; m < this->_vecParticleSet.size(); ++m)
        free(this->_vecParticleSet.at(m));

    this->_vecParticleSet.clear();
    this->_vecParticleSet.assign(vecNewParticles.begin(), vecNewParticles.end());

    return 0;
}

StrPoint* TrackingParticleFilter::getMostLikelyParticle()
{
    double dMaxWeight = 0;
    int unMaxIndex = 0;
    StrPoint *strTmpPoint = new StrPoint();

    for (unsigned int i = 0; i < this->_unNumberOfParticles; ++i)
    {
        if (this->_vecParticleSet.at(i)->dWeight >= dMaxWeight)
        {
            dMaxWeight = this->_vecParticleSet.at(i)->dWeight;
            unMaxIndex = i;
        }

    }

    strTmpPoint->dDistance = sqrt(pow(this->_vecParticleSet.at(unMaxIndex)->dX, 2) + pow(this->_vecParticleSet.at(unMaxIndex)->dY, 2));
    strTmpPoint->dRoll = 0;
    strTmpPoint->dPitch = 0;
    strTmpPoint->dYaw = atan(this->_vecParticleSet.at(unMaxIndex)->dY / this->_vecParticleSet.at(unMaxIndex)->dX);
    strTmpPoint->dX = this->_vecParticleSet.at(unMaxIndex)->dX;
    strTmpPoint->dY = this->_vecParticleSet.at(unMaxIndex)->dY;
    strTmpPoint->dZ = 0;

    return strTmpPoint;
}

StrPoint* TrackingParticleFilter::getMostLikelyPosition()
{
    StrPoint *strTmpPoint = new StrPoint();
    //int iPercentage = floor(((double)this->_unNumberOfParticles / 100.0 * 10.0));

    //cout << "perc: " << iPercentage << endl;

    //sort(this->_vecParticleSet.begin(), this->_vecParticleSet.end(), StrParticle::SortByWeights);

    for (unsigned int i = 0; i < this->_vecParticleSet.size(); ++i)
    {
        strTmpPoint->dX += this->_vecParticleSet.at(i)->dX;
        strTmpPoint->dY += this->_vecParticleSet.at(i)->dY;
    }

    strTmpPoint->dX /= this->_vecParticleSet.size();
    strTmpPoint->dY /= this->_vecParticleSet.size();
    strTmpPoint->dZ = 0;
    strTmpPoint->dDistance = sqrt(pow(strTmpPoint->dX, 2) + pow(strTmpPoint->dY, 2));
    strTmpPoint->dRoll = 0;
    strTmpPoint->dPitch = 0;
    strTmpPoint->dYaw = atan(strTmpPoint->dY / strTmpPoint->dX);

    return strTmpPoint;
}

double TrackingParticleFilter::getGaussian2D(double dXValue, double dYValue, double dXcenter, double dYCenter, double dXSigma, double dYSigma, double dTheta)
{
    double dA = 1.0;
    double da = 0.0, db = 0.0, dc = 0.0;
    double dZ = 0.0;

    da = (pow(cos(dTheta), 2) / (2 * pow(dXSigma, 2))) + (pow(sin(dTheta), 2) / (2 * pow(dYSigma, 2)));
    db = (-sin((2 * dTheta)) / (4 * pow(dXSigma, 2))) + (sin((2 * dTheta)) / (4 * pow(dYSigma, 2)));
    dc = (pow(sin(dTheta), 2) / (2 * pow(dXSigma, 2))) + (pow(cos(dTheta), 2) / (2 * pow(dYSigma, 2)));

    //cout << "da: " << da << " db: " << db << " dc: " << dc << endl;

    dZ = dA * exp(-((da * pow((dXValue - dXcenter), 2)) + (2 * db * (dXValue - dXcenter) * (dYValue - dYCenter)) + (dc * pow((dYValue - dYCenter), 2))));

    //cout << "dZ: " << dZ << endl;

    return dZ;
}

