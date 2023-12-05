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
#include <tuple>

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
    const float LOW_LOW = 0;
    const float LOW_HIGH = 400;
    const float MIDDLE_LOW = 800;
    const float MIDDLE_HIGH = 1200;
    const float HIGH_LOW = 1600;
    const float HIGH_HIGH = 2000;

    bool startup_check_flag;
    AbstractGraphicViewer *viewer;

    const float MAX_ADV_SPEED = 700;
    const float DOOR_PROXIMITY_THRESHOLD = 1200;

    struct Lines
    {
        RoboCompLidar3D::TPoints low, middle, high;
    };
    struct Door
    {
        struct Point{
            double x;
            double y;
        };
        RoboCompLidar3D::TPoint left, right, middle;
        const float THRESHOLD = 500; //door equality
        Door(){ left = right = middle = RoboCompLidar3D::TPoint(0,0,0);};
        Door(const RoboCompLidar3D::TPoint &left_,
             const RoboCompLidar3D::TPoint &right_) : left(left_), right(right_)
        {
            middle.x = (left.x + right.x)/2;
            middle.y = (left.y + right.y)/2;
        };
        bool operator==(const Door &d) const
        {
            return std::hypot(d.middle.x - middle.x, d.middle.y - middle.y) < THRESHOLD;
        };
        Door& operator=(const Door &d)
        {
            left = d.left;
            right = d.right;
            middle = d.middle;
            return *this;
        };
        void print()
        {
            qInfo() << "Door:";
            qInfo() << "    left:" << left.x << left.y;
            qInfo() << "    right:" << right.x << right.y;
        };
        float dist_to_robot() const
        { return std::hypot(middle.x, middle.y);}
        float angle_to_robot() const
        { return atan2(middle.x, middle.y);}
        Point perpendicular_point()const
        {
            // Calculate the direction vector from p1 to p2 and rotate it by 90 degrees
            Point d_perp;
            d_perp.x = -(left.y - right.y);
            d_perp.y = left.x - right.x;

            // Normalize the perpendicular vector to get the unit vector
            double magnitude = std::sqrt(std::pow(d_perp.x, 2) + std::pow(d_perp.y, 2));
            Point u_perp;
            u_perp.x = d_perp.x / magnitude;
            u_perp.y = d_perp.y / magnitude;

            // Calculate the points P1 and P2 at a distance of 1 meter from M along the perpendicular
            Point a, b;
            Point M{middle.x, middle.y};
            a.x = M.x + u_perp.x * 1000; // 1 meter in the direction of u_perp
            a.y = M.y + u_perp.y * 1000;
            b.x = M.x - u_perp.x * 1000; // 1 meter in the opposite direction of u_perp
            b.y = M.y - u_perp.y * 1000;
            float len_a = std::hypot(a.x, a.y);
            float len_b = std::hypot(b.x, b.y);
            return len_a < len_b ? a : b;
        }
        float perp_dist_to_robot() const
        {
            auto p = perpendicular_point();
            return std::hypot(p.x, p.y);
        }
        float perp_angle_to_robot() const
        {
            auto p = perpendicular_point();
            return atan2(p.x, p.y);
        }
    };

    using Doors = std::vector<Door>;

    void draw_lidar(const RoboCompLidar3D::TPoints &points, AbstractGraphicViewer *viewer);
    void draw_doors(const Doors &doors, AbstractGraphicViewer *viewer, QColor = QColor("green"));

    Lines extract_lines(const RoboCompLidar3D::TPoints &points);
    SpecificWorker::Lines extract_peaks(const Lines &peaks);

    std::tuple<Doors, Doors, Doors>
    get_doors(const Lines &lines);

    Doors filter_doors(const std::tuple<Doors, Doors, Doors> &doors);
    Doors doors_extractor(const RoboCompLidar3D::TPoints &filtered_points);

    // Estados
    Door door_target;
    enum class States{ IDLE, SEARCH_DOOR, GOTO_DOOR, GO_THROUGH};
    States state = States::SEARCH_DOOR;

    void move_robot(float side, float adv, float rot);
    float break_adv(float dist_to_target);
    float break_rot(float rot);

};

#endif
