/*
 *    Copyright (C) 2023 by YOUR NAME HERE
 *
 *    This file is part of RoboComp
 *
 *    RoboComp is free software: you can redistribute it and/or modify
 *    it under the terms of the GNU General Public License as published by
 *    the Free Software Foundation, either version 3 of the License, or
 *    (at your option) any later version.
 *
 *    RoboComp is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *    GNU General Public License for more details.
 *
 *    You should have received a copy of the GNU General Public License
 *    along with RoboComp.  If not, see <http://www.gnu.org/licenses/>.
 */

/**
	\brief
	@author authorname
*/

#ifndef SPECIFICWORKER_H
#define SPECIFICWORKER_H

#include <genericworker.h>
#include <abstract_graphic_viewer/abstract_graphic_viewer.h>
#include <ranges>

class SpecificWorker : public GenericWorker
{

Q_OBJECT
public:
	SpecificWorker(TuplePrx tprx, bool startup_check);
	~SpecificWorker();
	bool setParams(RoboCompCommonBehavior::ParameterList params);

public slots:
	void compute();
	int startup_check();
	void initialize(int period);

private:
    const float LOW_LOW = 300;
    const float LOW_HIGH = 500;
    const float MIDDLE_LOW = 600;
    const float MIDDLE_HIGH = 800;
    const float HIGH_LOW = 1800;
    const float HIGH_HIGH = 2000;

    bool startup_check_flag;
    AbstractGraphicViewer *viewer;

    struct Lines
    {
        RoboCompLidar3D::TPoints low, middle, high;
    };
    struct Door
    {
        const float THRESHOLD = 500;
        RoboCompLidar3D::TPoint left, right, middle;
    public:
        Door(RoboCompLidar3D::TPoint left_, RoboCompLidar3D::TPoint right_): left(left_), right(right_)
        {
            middle.x = (left.x + right.x)/2;
            middle.y = (left.y + right.y)/2;
        };
        bool operator == (const Door &d) const
        {
            return std::hypot(d.middle.x - middle.x, d.middle.y - middle.y) < THRESHOLD;

        };
    };
    using Doors = std::vector<Door>;

    void draw_lidar(const RoboCompLidar3D::TPoints &points, AbstractGraphicViewer *viewer);
    void draw_doors(const Doors &doors, AbstractGraphicViewer *viewer);

    Lines extract_lines(const RoboCompLidar3D::TPoints &points);
    SpecificWorker::Lines extract_peaks(const Lines &peaks);

    std::tuple<SpecificWorker::Doors, SpecificWorker::Doors, SpecificWorker::Doors> get_doors(const Lines &lines);

    Doors filter_doors(const std::tuple<SpecificWorker::Doors, SpecificWorker::Doors, SpecificWorker::Doors> &doors);

};

#endif
