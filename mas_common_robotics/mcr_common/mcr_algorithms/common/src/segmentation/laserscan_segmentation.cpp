#include "mcr_algorithms/segmentation/laserscan_segmentation.h"

LaserScanSegmentation::LaserScanSegmentation(double dThresholdDistanceBetweenAdajecentPoints, unsigned int unMinimumPointsPerSegment)
{
	this->_dThresholdDistanceBetweenAdajecentPoints = dThresholdDistanceBetweenAdajecentPoints;
	this->_unMinimumPointsPerSegment = unMinimumPointsPerSegment;
}

LaserScanSegmentation::~LaserScanSegmentation()
{
}

mcr_perception_msgs::LaserScanSegmentList LaserScanSegmentation::getSegments(const sensor_msgs::LaserScan::ConstPtr &inputScan, bool store_data_points)
{
	mcr_perception_msgs::LaserScanSegmentList segments;
	vector<geometry_msgs::Point> data_points;

	double dNumberofPointsBetweenStartAndEnd = 0;
	unsigned int unSegmentStartPoint = 0;
	unsigned int unSegmentEndPoint = 0;

	uint32_t scan_size = ceil((inputScan->angle_max - inputScan->angle_min) / inputScan->angle_increment);

	if(scan_size == 0)
		return segments;

	//run over laser scan data

	for(unsigned int i = 0; i < (scan_size - 1); ++i)
	{
		++dNumberofPointsBetweenStartAndEnd;

		// first point in laser scan is start point of the first segment
		if(i == 0)
			unSegmentStartPoint = i;

		// the distance between two adjacent points is above a given threshold -> remember end point and start with a new segment
		//cout << "angle: " << ppdLaserScan[i][BUFFER_COLUMN_YAW] << " " << (ppdLaserScan[i][BUFFER_COLUMN_YAW] * 180 / M_PI) << std::endl;

		double dAngleCur = inputScan->angle_min + (i * inputScan->angle_increment);
		double dDistanceCur = inputScan->ranges[i];
		double dAngleNext = inputScan->angle_min + ((i+1) * inputScan->angle_increment);
		double dDistanceNext = inputScan->ranges[i+1];

		if(store_data_points)
		{
			geometry_msgs::Point cur_point;
			cur_point.x = dDistanceCur * cos(dAngleCur);
			cur_point.y = dDistanceCur * sin(dAngleCur);
			data_points.push_back(cur_point);
		}

//		cout << "a: " << dAngleCur << " d: " << dDistanceCur << " a: " << dAngleNext<< " d: " << dAngleNext << endl;

		if((getEuclideanDistance(dDistanceCur, dAngleCur, dDistanceNext, dAngleNext) > this->_dThresholdDistanceBetweenAdajecentPoints) || (i == (scan_size - 2)))
		{
			if( i < (scan_size - 2))
				unSegmentEndPoint = i;
			else
				unSegmentEndPoint = i+1;

			// if number of points between start and end point is lesser then 3 , it is not a segment
			if(dNumberofPointsBetweenStartAndEnd >= this->_unMinimumPointsPerSegment )
			{
				geometry_msgs::Point centerPoint;
				centerPoint = getCenterOfGravity(unSegmentStartPoint, unSegmentEndPoint, inputScan);
				double dDistanceToSegment = sqrt( pow(centerPoint.x, 2.0) + pow(centerPoint.y, 2.0));

				if(dDistanceToSegment < 5.0)
				{
					mcr_perception_msgs::LaserScanSegment seg;

					seg.header = inputScan->header;
					seg.header.stamp = ros::Time::now();
					seg.center.x = centerPoint.x;
					seg.center.y = centerPoint.y;

					if(store_data_points)
						seg.data_points = data_points;

					/*
					cout << "eucl: " << getEuclideanDistance(dDistanceCur, dAngleCur, dDistanceNext, dAngleNext) << " thr: " << this->_dThresholdDistanceBetweenAdajecentPoints << 	endl;
					cout << "new segmented: " << endl;
					cout << "dist: : " << dDistanceToSegment << endl;
					*/

					segments.segments.push_back(seg);
				}
			}

			if( i < (scan_size - 2))
			{
				unSegmentStartPoint = i+1;
				dNumberofPointsBetweenStartAndEnd = 0;

				if(store_data_points)
					data_points.clear();
			}
		}
	}

	segments.header = inputScan->header;
	segments.header.stamp = ros::Time::now();
	segments.num_segments = segments.segments.size();

	return segments;
}

double LaserScanSegmentation::getEuclideanDistance(double dDistanceA, double dAngleA, double dDistanceB, double dAngleB)
{
	return sqrt((dDistanceA * dDistanceA) + (dDistanceB * dDistanceB) -
			(2 * dDistanceA * dDistanceB) * cos(fabs(dAngleA - dAngleB)));
}

geometry_msgs::Point LaserScanSegmentation::getCenterOfGravity(unsigned int indexStart, unsigned int indexEnd, const sensor_msgs::LaserScan::ConstPtr &inputScan)
{
	geometry_msgs::Point centerPoint;

	centerPoint.x = 0;
	centerPoint.y = 0;
	centerPoint.z = 0;

	unsigned int i = 0, j = 0;
	for(i = indexStart, j = 0; i <= indexEnd; ++i, ++j)
	{
		double dAngle = inputScan->angle_min + (i * inputScan->angle_increment);
		double dX = inputScan->ranges[i] * cos(dAngle);
		double dY = inputScan->ranges[i] * sin(dAngle);

		centerPoint.x += dX;
		centerPoint.y += dY;
	}

	centerPoint.x /= j;
	centerPoint.y /= j;

	return centerPoint;
}
