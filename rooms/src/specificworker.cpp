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
#include "specificworker.h"
#include <cppitertools/sliding_window.hpp>
#include <cppitertools/combinations.hpp>


/**
* \brief Default constructor
*/
SpecificWorker::SpecificWorker(TuplePrx tprx, bool startup_check) : GenericWorker(tprx)
{
	this->startup_check_flag = startup_check;
	// Uncomment if there's too many debug messages
	// but it removes the possibility to see the messages
	// shown in the console with qDebug()
//	QLoggingCategory::setFilterRules("*.debug=false\n");
}

/**
* \brief Default destructor
*/
SpecificWorker::~SpecificWorker()
{
	std::cout << "Destroying SpecificWorker" << std::endl;
}

bool SpecificWorker::setParams(RoboCompCommonBehavior::ParameterList params)
{
//	THE FOLLOWING IS JUST AN EXAMPLE
//	To use innerModelPath parameter you should uncomment specificmonitor.cpp readConfig method content
//	try
//	{
//		RoboCompCommonBehavior::Parameter par = params.at("InnerModelPath");
//		std::string innermodel_path = par.value;
//		innerModel = std::make_shared(innermodel_path);
//	}
//	catch(const std::exception &e) { qFatal("Error reading config params"); }

	return true;
}

void SpecificWorker::initialize(int period)
{
	std::cout << "Initialize worker" << std::endl;
	this->Period = period;
	if(this->startup_check_flag)
	{
		this->startup_check();
	}
	else
	{
        // Inicializaciones personales
        viewer = new AbstractGraphicViewer(this, QRectF(-5000, -5000, 10000, 10000));
        viewer->add_robot(460, 480, 0, 100, QColor("blue"));
        viewer->show();
        viewer->activateWindow();
		timer.start(Period);
	}
}

void SpecificWorker::compute()
{
    try {
        auto ldata = lidar3d_proxy->getLidarData("helios", 0, 360, 1);
        const auto &points = ldata.points;
        if (points.empty()) return;

        /// Filter points above 2000
        RoboCompLidar3D::TPoints filtered_points;
        std::ranges::remove_copy_if(ldata.points, std::back_inserter(filtered_points),[](auto &p) { return p.z > 2000; });

        auto lines = extract_lines(filtered_points);
        auto peaks = extract_peaks(lines);
        auto doors = get_doors(peaks);
        auto final_doors = filter_doors(doors);

        draw_lidar(lines.middle, viewer);
        draw_doors(std::get<0>(doors), viewer);


    }
    catch(const Ice::Exception &e)
    {
        std::cout << "Error reading from Camera" << e << std::endl;
    }
}


SpecificWorker::Lines SpecificWorker::extract_lines(const RoboCompLidar3D::TPoints &points)
{
    Lines lines;
    for(const auto &p: points)
    {
        qInfo() << p.x << p.y << p.z;
        if(p.z > LOW_LOW and p.z < LOW_HIGH)
            lines.low.push_back(p);
        if(p.z > MIDDLE_LOW and p.z < MIDDLE_HIGH)
            lines.middle.push_back(p);
        if(p.z > HIGH_LOW and p.z < HIGH_HIGH)
            lines.high.push_back(p);
    }
    return lines;
}

SpecificWorker::Lines SpecificWorker::extract_peaks(const SpecificWorker::Lines &lines)
{
    Lines peaks;
    const float THRES_PEAK = 1000;

    for(const auto &both: iter::sliding_window(lines.low, 2))
    {
        if(fabs(both[1].r - both[0].r) > THRES_PEAK)
        {
            if(both[0].r < both[1].r){
                peaks.low.push_back(both[0]);
            }
            else
            {
                peaks.low.push_back(both[1]);
            }
        }
    }

    for(const auto &both: iter::sliding_window(lines.middle, 2))
    {
        if(fabs(both[1].r - both[0].r) > THRES_PEAK)
        {
            if(both[0].r < both[1].r)
            {
                peaks.middle.push_back(both[0]);
            }
            else
            {
                peaks.middle.push_back(both[1]);
            }
        }
    }

    for(const auto &both: iter::sliding_window(lines.high, 2))
    {
        if(fabs(both[1].r - both[0].r) > THRES_PEAK)
        {
            if(both[0].r < both[1].r)
            {
                peaks.high.push_back(both[0]);
            }
            else
            {
                peaks.high.push_back(both[1]);
            }
        }
    }
    return peaks;
}

std::tuple<SpecificWorker::Doors, SpecificWorker::Doors, SpecificWorker::Doors>
SpecificWorker::get_doors(const SpecificWorker::Lines &peaks) {

    Doors doors_low, doors_middle, doors_high;

    auto dist = [](auto a, auto b){
        qInfo() << std::hypot(a.x-b.x, a.y-b.y);
        return std::hypot(a.x-b.x, a.y-b.y);
    };

    const float THRES_DOOR = 500;

    auto near_door = [dist, THRES_DOOR](auto &doors, auto d){
        for(auto &&old: doors)
        {
            qInfo() << dist(old.left, d.left) << dist(old.right, d.right) << dist(old.right, d.left) << dist(old.left, d.right);
            if( dist(old.left, d.left) < THRES_DOOR or
                dist(old.right, d.right) < THRES_DOOR or
                dist(old.right, d.left) < THRES_DOOR or
                dist(old.left, d.right) < THRES_DOOR)
                return true;
        }
        return false;
    };

    for(auto &par : peaks.low | iter::combinations(2)){
        if(dist(par[0], par[1]) < 1400 && dist(par[0], par[1]) > 500){
            auto door = Door(par[0], par[1]);
            if(!near_door(doors_low, door)) {
                doors_low.emplace_back(Door{par[0], par[1]});
            }
        }
    }
    for(auto &par : peaks.middle | iter::combinations(2)){
        if(dist(par[0], par[1]) < 1400 && dist(par[0], par[1]) > 500){
            auto door = Door(par[0], par[1]);
            if(!near_door(doors_middle, door)) {
                doors_middle.emplace_back(Door{par[0], par[1]});
            }
        }
    }
    for(auto &par : peaks.high | iter::combinations(2)){
        if(dist(par[0], par[1]) < 1400 && dist(par[0], par[1]) > 500){
            auto door = Door(par[0], par[1]);
            if(!near_door(doors_high, door)) {
                doors_high.emplace_back(Door{par[0], par[1]});
            }
        }
    }

    return std::make_tuple(doors_low, doors_middle, doors_high);
}

SpecificWorker::Doors
SpecificWorker::filter_doors(const tuple<SpecificWorker::Doors, SpecificWorker::Doors, SpecificWorker::Doors> &doors)
{
    Doors final_doors;

    auto &[dlow, dmiddle, dhigh] = doors;
    for(const auto &dl: dlow)
    {
        bool equal_middle = std::ranges::find( dmiddle, dl ) != dmiddle.end();
        bool equal_high = std::ranges::find( dhigh, dl ) != dhigh.end();

        if (equal_middle and equal_high)
        {
            final_doors.emplace_back(dl);
       }
    }

    return final_doors;
}

////////////////////////////////////////////////////////////////////////////////////////////
int SpecificWorker::startup_check()
{
    std::cout << "Startup check" << std::endl;
    QTimer::singleShot(200, qApp, SLOT(quit()));
    return 0;
}

void SpecificWorker::draw_lidar(const RoboCompLidar3D::TPoints &points, AbstractGraphicViewer *viewer)
{
    static std::vector<QGraphicsItem*> borrar;
    for(auto &b: borrar)
    {
        viewer->scene.removeItem(b);
        delete b;
    }

    borrar.clear();

    for(const auto &p: points)
    {
        auto point = viewer->scene.addRect(-50, -50, 100, 100, QPen(QColor("blue")), QBrush(QColor("blue")));
        point->setPos(p.x, p.y);
        borrar.push_back(point);
    }
}

void SpecificWorker::draw_doors(const Doors &doors, AbstractGraphicViewer *viewer)
{
    static std::vector<QGraphicsItem *> borrar;
    for (auto &b: borrar)
    {
        viewer->scene.removeItem(b);
        delete b;
    }
    borrar.clear();

    for (const auto &d: doors)
    {
        auto point = viewer->scene.addRect(-50, -50, 100, 100, QPen(QColor("green")), QBrush(QColor("green")));
        point->setPos(d.left.x, d.left.y);
        borrar.push_back(point);

        point = viewer->scene.addRect(-50, -50, 100, 100, QPen(QColor("green")), QBrush(QColor("green")));
        point->setPos(d.right.x, d.right.y);
        borrar.push_back(point);

        auto line = viewer->scene.addLine(d.left.x, d.left.y, d.right.x, d.right.y, QPen(QColor("green"), 50));
        borrar.push_back(line);
    }
}

/**************************************/
// From the RoboCompLidar3D you can call this methods:
// this->lidar3d_proxy->getLidarData(...)

/**************************************/
// From the RoboCompLidar3D you can use this types:
// RoboCompLidar3D::TPoint
// RoboCompLidar3D::TData

