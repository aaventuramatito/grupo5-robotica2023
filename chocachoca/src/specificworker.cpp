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
        viewer->add_robot(460, 480, 0, 100, QColor("Blue"));
        viewer->show();
        viewer->activateWindow();

		timer.start(Period);
	}

}

void SpecificWorker::compute()
{
    auto ldata = lidar3d_proxy->getLidarData("bpearl", 0, 2*M_PI, 1);
    //qInfo() << ldata.points.size();
    const auto &points = ldata.points;
    if(points.empty()) return;

    /// Filter points above 2000
    std::ranges::remove_copy_if(ldata.points, std::back_inserter(filtered_points), [](auto &p){ return p.z > 2000;});
    draw_lidar(filtered_points, viewer);
    std::tuple<Estado, RobotSpeed> res;

    /// State machine
    switch(estado)
    {
        case Estado::IDLE:
            break;
        case Estado::FOLLOW_WALL:
            break;
        case Estado::STRAIGHT_LINE:
            res = chocachoca();
            break;
        case Estado::SPIRAL:
            break;
    };

    try
    {
        const auto &[adv, side, rot] = std::get<RobotSpeed>(res);
        omnirobot_proxy->setSpeedBase(adv, side, rot);
    }
    catch(const Ice::Exception &e)
    {  std::cout << "Error reading from Camera" << e << std::endl; 	}
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
///////////////////////////////////////////////////////////
/// Estados
//////////////////////////////////////////////////////////
std::tuple<SpecificWorker::Estado, SpecificWorker::RobotSpeed> SpecificWorker::chocachoca()
{
    int offset = filtered_points.size()/2-filtered_points.size()/3;
    auto min_elem = std::min_element(filtered_points.begin()+offset, filtered_points.end()-offset,
                                     [](auto a, auto b) { return std::hypot(a.x, a.y) < std::hypot(b.x, b.y);});

    RobotSpeed robot_speed;
    Estado estado;
    const float MIN_DISTANCE = 1000;
    if(std::hypot(min_elem->x, min_elem->y) < MIN_DISTANCE)
        robot_speed = RobotSpeed{.adv=0, .side=0, .rot=0.5};
    else
        robot_speed = RobotSpeed{.adv=1, .side=0, .rot=0};

    return std::make_tuple(Estado::STRAIGHT_LINE, robot_speed);
}
/**************************************/
// From the RoboCompLidar3D you can call this methods:
// this->lidar3d_proxy->getLidarData(...)

/**************************************/
// From the RoboCompLidar3D you can use this types:
// RoboCompLidar3D::TPoint
// RoboCompLidar3D::TData

