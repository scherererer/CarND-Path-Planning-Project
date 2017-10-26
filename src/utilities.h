
#pragma once

#include "constants.h"
#include "map.h"

#include "tk/spline.h"

#include <cmath>
#include <vector>

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
inline double deg2rad(double const x) { return x * pi() / 180; }
inline double rad2deg(double const x) { return x * 180 / pi(); }

inline double mph2m_s(double const x) { return x * 0.44704; }
inline double m_s2mph(double const x) { return x / 0.44704; }

inline double distance(double x1, double y1, double x2, double y2)
{
	return sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1));
}

inline double angleDiff (double a, double b)
{
	double diff = fmod(b - a + M_PI, M_PI * 2.0);
	if (diff < 0)
		diff += M_PI * 2.0;
	return diff - M_PI;
}

inline int closestWaypoint(double x, double y, Map const &map)
{
	double closestLen = 100000; //large number
	int closestWaypoint = 0;

	for(int i = 0; i < map.waypoints_x.size(); i++)
	{
		double map_x = map.waypoints_x[i];
		double map_y = map.waypoints_y[i];
		double dist = distance(x,y,map_x,map_y);
		if(dist < closestLen)
		{
			closestLen = dist;
			closestWaypoint = i;
		}

	}

	return closestWaypoint;
}

inline int nextWaypoint(double x, double y, double theta, Map const &map)
{
	int closest = closestWaypoint(x, y, map);

	double map_x = map.waypoints_x[closest];
	double map_y = map.waypoints_y[closest];

	double heading = atan2( (map_y-y),(map_x-x) );

	double angle = abs(theta-heading);

	if(angle > pi()/4)
		++closest;

	return closest;

}

// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
inline std::vector<double> getFrenet(double x, double y, double theta, Map const &map)
{
	int next_wp = nextWaypoint(x, y, theta, map);

	int prev_wp;
	prev_wp = next_wp-1;
	if(next_wp == 0)
	{
		prev_wp  = map.waypoints_x.size()-1;
	}

	double n_x = map.waypoints_x[next_wp]-map.waypoints_x[prev_wp];
	double n_y = map.waypoints_y[next_wp]-map.waypoints_y[prev_wp];
	double x_x = x - map.waypoints_x[prev_wp];
	double x_y = y - map.waypoints_y[prev_wp];

	// find the projection of x onto n
	double proj_norm = (x_x*n_x+x_y*n_y)/(n_x*n_x+n_y*n_y);
	double proj_x = proj_norm*n_x;
	double proj_y = proj_norm*n_y;

	double frenet_d = distance(x_x,x_y,proj_x,proj_y);

	//see if d value is positive or negative by comparing it to a center point

	double center_x = 1000-map.waypoints_x[prev_wp];
	double center_y = 2000-map.waypoints_y[prev_wp];
	double centerToPos = distance(center_x,center_y,x_x,x_y);
	double centerToRef = distance(center_x,center_y,proj_x,proj_y);

	if(centerToPos <= centerToRef)
	{
		frenet_d *= -1;
	}

	// calculate s value
	double frenet_s = 0;
	for(int i = 0; i < prev_wp; i++)
	{
		frenet_s += distance(map.waypoints_x[i],map.waypoints_y[i],
		                     map.waypoints_x[i+1],map.waypoints_y[i+1]);
	}

	frenet_s += distance(0,0,proj_x,proj_y);

	return {frenet_s,frenet_d};

}

// Transform from Frenet s,d coordinates to Cartesian x,y
inline std::vector<double> getXY(double s, double d, Map const &map)
{
	int prev_wp = -1;

	while(s > map.waypoints_s[prev_wp+1] && (prev_wp < (int)(map.waypoints_s.size()-1) ))
	{
		prev_wp++;
	}

	int wp2 = (prev_wp+1)%map.waypoints_x.size();

	double heading = atan2((map.waypoints_y[wp2]-map.waypoints_y[prev_wp]),
	                       (map.waypoints_x[wp2]-map.waypoints_x[prev_wp]));
	// the x,y,s along the segment
	double seg_s = (s-map.waypoints_s[prev_wp]);

	double seg_x = map.waypoints_x[prev_wp]+seg_s*cos(heading);
	double seg_y = map.waypoints_y[prev_wp]+seg_s*sin(heading);

	double perp_heading = heading-pi()/2;

	double x = seg_x + d*cos(perp_heading);
	double y = seg_y + d*sin(perp_heading);

	return {x,y,heading};
}

inline constexpr int getLane (double d)
{
	return static_cast<int> (d / LANE_WIDTH);
}

inline constexpr double laneToD (int lane)
{
	// Get the center of the lane
	return (lane * LANE_WIDTH) + (LANE_WIDTH/2.0);
}

inline double clip (double x, double min, double max)
{
	return std::min (max, std::max (x, min));
}

/// \brief Ramp x0 towards x1 at a rate no greater than step
inline double ramp (double const x0, double const x1, double const step)
{
	double const dx = x1 - x0;
	if (fabs (dx) > step)
		return x0 + std::copysign(step, dx);
	return x0 + dx;
}
