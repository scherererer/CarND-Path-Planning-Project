
#pragma once

#include <vector>

struct Map
{
	std::vector<double> waypoints_x;
	std::vector<double> waypoints_y;

	double d_resolution;

	std::vector<double> waypoints_s;
	std::vector<double> waypoints_dx;
	std::vector<double> waypoints_dy;
};

/*void dialateMap (double distance, double resolution, Map &map)
{
	std::vector<double> const &wxo = map.waypoints_x[0];
	std::vector<double> const &wyo = map.waypoints_y[0];

	for (double d = resolution; d < distance; d += resolution)
	{
		map.waypoints_x.push_back ({});
		map.waypoints_y.push_back ({});

		std::vector<double> &wxn = map.waypoints_x.back ();
		std::vector<double> &wyn = map.waypoints_y.back ();

		for (int i1 = 0; i1 < wxo.size (); ++i1)
		{
			// Index before
			int const i0 = i1 > 0 ? i1 - 1 : wxo.size () - 1;
			// Index after
			int const i2 = (i1 + 1) % wxo.size ();

			double const angle = atan2 (wyo[i2] - wyo[i0],
										wxo[i2] - wxo[i0]);
			double const perp_angle = angle - M_PI / 2.0;

			wxn.push_back(wxo[i1] + d*cos(perp_angle));
			wyn.push_back(wyo[i1] + d*sin(perp_angle));
		}
	}

	d_resolution = resolution;
}*/
