
#pragma once

class CoordXY
{
public:
	CoordXY ();
	CoordXY (double x, double y)
		: x_ (x)
		, y_ (y)
	{
	}

	double x () const
		{ return x_; }
	double y () const
		{ return y_; }

private:
	double x_;
	double y_;
};
