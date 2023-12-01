//
// Created by aku on 11/30/23.
//

#pragma once

#include <cmath>
#include <algorithm>

#define EPSILON 1e-6

namespace rbplib
{
	/**
	 * Applies a function comprised of two exponential functions. Inspired by 5225, the
	 * Pilons. Visualization of curve is here (red graph):
	 * https://www.desmos.com/calculator/rrbvbmxvqo
	 *
	 *
	 * @param ivalue Input from joystick
	 * @param igain Scaling factor for curve (effect of curve can be seen through Desmos)
	 * @return
	 */
	inline double exponentialSumCurve(const double ivalue,
									  const double igain)
	{
		if (fabs(igain) < EPSILON)
			return ivalue;
		return std::clamp((powf(2.718, -igain)
			 + powf(2.718, (fabs(ivalue) - 1))
			 * (1 - powf(2.718, -igain)))
			 * ivalue, -1.0, 1.0);
	}
	/**
	 * Applies a function comprised of a single exponential functions. Inspired by
	 * 5225, the Pilons. Visualization of curve is here (blue graph):
	 * https://www.desmos.com/calculator/n3gwt6w4t5 (ctrl + click to open link)
	 *
	 *
	 * @param ivalue Input from joystick
	 * @param igain Scaling factor for curve (effect of curve can be seen through Desmos)
	 * @return
	 */
	inline double scaledExponentialCurve(const double ivalue,
								  		 const double igain)
	{
		if (fabs(igain) < EPSILON)
			return ivalue;
		return std::clamp((powf(2.718, igain * (fabs(ivalue) - 1) / 4)) * ivalue, -1.0,
						  1.0);
	}
} // namespace rbplib