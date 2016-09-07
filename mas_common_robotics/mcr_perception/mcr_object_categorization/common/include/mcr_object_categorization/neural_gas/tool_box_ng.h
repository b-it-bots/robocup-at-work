/*
 * Created on: 19.04.2011
 * Author: Christian Mueller
 */

#ifndef CTOOLBOX_NG_H
#define CTOOLBOX_NG_H

#include<iostream>
#include<vector>
#include<cmath>
#define PI 3.14159265

class CToolBoxNG
{
private:
    int id;
public:
    CToolBoxNG()
    {
        ;
    }
    std::vector<double> normalizePoint3D(std::vector<double> point)
    {
        std::vector<double> normalizedpoint;

        normalizedpoint.resize(point.size());
        double a;
        double sum = 0;

        for (unsigned int i = 0; i < point.size(); i++)
        {
            sum += (point[i] * point[i]);
        }

        a = sqrt(sum);

        for (unsigned int i = 0; i < point.size(); i++)
        {
            normalizedpoint[i] = (point[i] / a);
        }

        return normalizedpoint;
    }

    float angleBetweenPoints(std::vector<double> point1, std::vector<double> point2)
    {
        float angle = 0;
        if (point1.size() != point2.size())
            return angle;

        std::vector<double> normalizedpoint1;
        std::vector<double> normalizedpoint2;

        normalizedpoint1 = this->normalizePoint3D(point1);
        normalizedpoint2 = this->normalizePoint3D(point2);

        double sum = 0;
        for (unsigned int i = 0; i < normalizedpoint1.size(); i++)
        {
            sum += normalizedpoint1[i] * normalizedpoint2[i];
            //  std::cout<<"norm1 "<<normalizedpoint1[i]<<"\n";
            //std::cout<<"norm2 "<<normalizedpoint2[i]<<"\n";
        }


        angle = (acos(sum) *  180.0 / PI);

        //std::cout<<"norm1 "<<normalizedpoint1<<"\n";
        //std::cout<<"norm2 "<<normalizedpoint2<<"\n";
        //  std::cout<<"sum "<<sum<<"\n";
        //std::cout<<"angle "<<angle<<"\n";

        return angle;
    }
};

#endif
