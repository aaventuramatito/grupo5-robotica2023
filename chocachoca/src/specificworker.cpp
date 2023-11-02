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
        viewer->add_robot(460, 480, 0, 100, QColor("blue"));
        viewer->show();
        viewer->activateWindow();
		timer.start(Period);
	}
}

void SpecificWorker::compute()
{
    try {
        auto ldata = lidar3d_proxy->getLidarData("bpearl", 0, 2 * M_PI, 1);
        const auto &points = ldata.points;
        if (points.empty()) return;

        /// Filter points above 2000
        RoboCompLidar3D::TPoints filtered_points;
        std::ranges::remove_copy_if(ldata.points, std::back_inserter(filtered_points),
                                    [](auto &p) { return p.z > 2000; });
        draw_lidar(filtered_points, viewer);

        /// State machine
        switch (estado) {
            case Estado::STRAIGHT_LINE:
                qInfo() << "STRAIGHT_LINE";
                straight_line(filtered_points);
                break;
            case Estado::FOLLOW_WALL:
                qInfo() << "FOLLOW_WALL";
                follow_wall(filtered_points);
                break;
            case Estado::SPIRAL:
                qInfo() << "SPIRAL";
                spiral(filtered_points);
                break;
            case Estado::TURN:
                qInfo() << "TURN";
                turn(filtered_points);
                break;
            case Estado::MIDLE:
                qInfo() << "MIDLE";
                midle(filtered_points);
                break;
        }
    }
    catch(const Ice::Exception &e)
    {
        std::cout << "Error reading from Camera" << e << std::endl;
    }
}
///////////////////////////////////////////////////////////
/// Estados
//////////////////////////////////////////////////////////
void SpecificWorker::straight_line(RoboCompLidar3D::TPoints &filtered_points)
{
    int offset = filtered_points.size() / 2 - filtered_points.size() / 5;
    auto min_elem = std::min_element(filtered_points.begin() + offset, filtered_points.end() - offset,
                                     [](auto a, auto b) { return std::hypot(a.x, a.y) < std::hypot(b.x, b.y);});

    if(std::hypot(min_elem->x, min_elem->y) < MIN_DISTANCE)
    {
        estado = Estado::TURN;
    }
    else
    {
        omnirobot_proxy->setSpeedBase(2, 0, 0);

    }
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_int_distribution<int> distribution(1, 250);
    int i = distribution(gen);
    if(i == 13 || i == 150 || i == 225)
    {
        estado = Estado::SPIRAL;
    }
    if(i == 113)
    {
        estado = Estado::FOLLOW_WALL;
    }
}

void SpecificWorker::follow_wall(RoboCompLidar3D::TPoints &filtered_points)
{
    static int pared = 0;
    float K = 7.5;
    int offset = filtered_points.size() / 2 - filtered_points.size() / 4;
    auto min_elem = std::min_element(filtered_points.begin() + offset, filtered_points.end() - offset,
                                     [](auto a, auto b) { return std::hypot(a.x, a.y) < std::hypot(b.x, b.y); });

    float angulo_actual = std::atan2(min_elem->y, min_elem->x);
    float velocidad_angular = K * angulo_actual;

    if(std::hypot(min_elem->x, min_elem->y) < MIN_FOLLOW_WALL)
    {
        omnirobot_proxy->setSpeedBase(1.5, 0, velocidad_angular);
        if(velocidad_angular > 1.0)
        {
            pared++;
        }
        if(pared >= 24)
        {
            MIN_FOLLOW_WALL -= 210;
            pared = 0;
            if(MIN_FOLLOW_WALL <= 800)
            {
                estado = Estado::STRAIGHT_LINE;
            }
        }
    }
    else
    {
        omnirobot_proxy->setSpeedBase(1.5, 0, 0);
    }
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_int_distribution<int> distribution(1, 500);
    int i = distribution(gen);
    if (i == 54)
    {
        estado = Estado::STRAIGHT_LINE;
    }
}

void SpecificWorker::turn(RoboCompLidar3D::TPoints &filtered_points)
{
    static int i = 2;
    int offset = filtered_points.size() / 2 - filtered_points.size() / 5;
    auto min_elem = std::min_element(filtered_points.begin() + offset, filtered_points.end() - offset,
                                     [](auto a, auto b) { return std::hypot(a.x, a.y) < std::hypot(b.x, b.y);});

    if(std::hypot(min_elem->x, min_elem->y) < MIN_DISTANCE)
    {
        if(i % 2 == 0)
        {
            omnirobot_proxy->setSpeedBase(0, 0, 1);

        }
        else
        {
            omnirobot_proxy->setSpeedBase(0, 0, -1);
        }
    }
    else{
        estado = Estado::STRAIGHT_LINE;
        std::random_device rd;
        std::mt19937 gen(rd());
        std::uniform_int_distribution<int> distribution(2, 3);
        i = distribution(gen);
    }
}

void SpecificWorker::spiral(RoboCompLidar3D::TPoints &filtered_points)
{
    int offset = filtered_points.size() / 2 - filtered_points.size() / 5;
    auto min_elem = std::min_element(filtered_points.begin() + offset, filtered_points.end() - offset,
                                     [](auto a, auto b) { return std::hypot(a.x, a.y) < std::hypot(b.x, b.y);});

    omnirobot_proxy->setSpeedBase(forwardSpeed, 0, angularSpeed);
    if(forwardSpeed >= 2.0)
    {
        angularSpeed -= 0.01;
    }
    else
    {
        forwardSpeed += 0.02;
        angularSpeed += 0.02;
    }

    if(std::hypot(min_elem->x, min_elem->y) < MIN_DISTANCE)
    {
        estado = Estado::MIDLE;
        forwardSpeed = 0.1;
        angularSpeed = 1.1;
    }
}

void SpecificWorker::midle(RoboCompLidar3D::TPoints &filtered_points)
{
    int offset = filtered_points.size() / 2 - filtered_points.size() / 5;
    auto min_elem = std::min_element(filtered_points.begin() + offset, filtered_points.end() - offset,
                                     [](auto a, auto b) { return std::hypot(a.x, a.y) < std::hypot(b.x, b.y);});

    if(std::hypot(min_elem->x, min_elem->y) < MIN_DISTANCE)
    {
        omnirobot_proxy->setSpeedBase(0, 0, 3);
    }
    else
    {
        omnirobot_proxy->setSpeedBase(2, 0, 0);
        if (std::hypot(min_elem->x, min_elem->y) > 2300)
        {
            std::random_device rd;
            std::mt19937 gen(rd());
            std::uniform_int_distribution<int> distribution(1, 10);
            int i = distribution(gen);
            if(i == 4)
            {
                estado = Estado::SPIRAL;
            }
            else
            {
                estado = Estado::FOLLOW_WALL;
            }
        }
    }
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

/**************************************/
// From the RoboCompLidar3D you can call this methods:
// this->lidar3d_proxy->getLidarData(...)

/**************************************/
// From the RoboCompLidar3D you can use this types:
// RoboCompLidar3D::TPoint
// RoboCompLidar3D::TData

