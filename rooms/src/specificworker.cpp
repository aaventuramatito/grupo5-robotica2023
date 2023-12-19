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
#include <cppitertools/enumerate.hpp>

/**
* \brief Default constructor
*/
SpecificWorker::SpecificWorker(TuplePrx tprx, bool startup_check) : GenericWorker(tprx)
{
    this->startup_check_flag = startup_check;
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
        viewer = new AbstractGraphicViewer(this, QRectF(-5000,-5000,10000,10000));
        viewer->add_robot(460,480,0,100,QColor("Blue"));
        viewer->show();
        viewer->activateWindow();

        timer.start(Period);
    }
}

void SpecificWorker::compute()
{
    RoboCompLidar3D::TData ldata;
    ldata = lidar3d_proxy->getLidarData(consts.lidar_name, 0, 360, 1);
    const auto &points = ldata.points;
    if (points.empty()) return;

    // Doors
    auto lines = extract_lines(points, consts.ranges_list);
    auto doors = door_detector.detect(lines, &viewer->scene);

    match_door_target(doors, door_target);

    // Estados
    state_machine(doors);

    draw_lines(lines, viewer);
    draw_target_door(door_target, viewer);

}

///////////////////////////////////////////////////////////////////////////////
void SpecificWorker::state_machine(const Doors &doors)
{
    switch (estado)
    {
        case Estados::IDLE:
        {
            move_robot(0,0,0);
            break;
        }
        case Estados::SEARCH_DOOR:
        {
            if(!doors.empty())
            {
                Door closest_door = doors[0];
                for (const auto& door : doors)
                {
                    if (fabs(door.angle_to_robot()) < fabs(closest_door.angle_to_robot()))
                    {
                        closest_door = door;
                    }
                }
                door_target = closest_door;
                move_robot(0,0,0);
                estado = Estados::GOTO_DOOR;
                qInfo() << "Puerta con el ángulo más pequeño encontrado";
                door_target.print();
            }
            else
                move_robot(0,0,0.3);
            break;
        }
        case Estados::GOTO_DOOR:
        {
            if(door_target.perp_dist_to_robot() < consts.DOOR_PROXIMITY_THRESHOLD)
            {
                move_robot(1,0, 0);
                qInfo() << "GOTO_DOOR Objetivo alcanzado";
                estado = Estados::ALIGN;
            }
            else
            {
                float rot = -0.5 * door_target.perp_angle_to_robot();
                float adv = consts.MAX_ADV_SPEED * break_adv(door_target.perp_dist_to_robot()) *
                            break_rot(door_target.perp_angle_to_robot()) / 1000.f;
                move_robot(0, adv, rot);
            }
            break;
        }
        case Estados::ALIGN:
        {
            if( fabs(door_target.angle_to_robot()) < 0.02)
            {
                move_robot(0,0,0);
                estado = Estados::GO_THROUGH;
                return;
            }
            float rot = -0.5 * door_target.angle_to_robot();
            move_robot(0,0,rot);
            break;
        }
        case Estados::GO_THROUGH:
        {
            auto now = std::chrono::steady_clock::now();

            if (!InicioTimer.time_since_epoch().count())
                InicioTimer = now;

            if (std::chrono::duration_cast<std::chrono::milliseconds>(now - InicioTimer) < std::chrono::milliseconds(10000))
            {
                move_robot(0, 1.0, 0);
            }
            else
            {
                InicioTimer = std::chrono::steady_clock::time_point();
                estado = Estados::SEARCH_DOOR;
            }
            break;
        }
    }
}

void SpecificWorker::match_door_target(const Doors &doors, const Door &target)
{
    if(doors.empty())
        return;

    if(auto res = std::ranges::find(doors, target); res != doors.end())
        door_target = *res;
    else
    {
        move_robot(0,0,0);
        estado = Estados::SEARCH_DOOR;
        qInfo() << "GOTO_DOOR Puerta perdida, buscando";
    }
}

SpecificWorker::Lines SpecificWorker::extract_lines(const RoboCompLidar3D::TPoints &points, const std::vector<std::pair<float, float>> &ranges)
{
    Lines lines(ranges.size());
    for(const auto &p: points)
        for(const auto &[i, r] : ranges | iter::enumerate)
            if(p.z > r.first and p.z < r.second)
                lines[i].emplace_back(p.x, p.y);
    return lines;
}

float SpecificWorker::break_adv(float dist_to_target)
{
    return std::clamp(dist_to_target / consts.DOOR_PROXIMITY_THRESHOLD, 0.f, 1.f );
}

float SpecificWorker::break_rot(float rot)
{
    return rot>=0 ? std::clamp(1-rot, 0.f, 1.f) : std::clamp(rot+1, 0.f, 1.f);
}

void SpecificWorker::move_robot(float side, float adv, float rot)
{
    try
    {
        omnirobot_proxy->setSpeedBase(adv, 0, rot);
    }
    catch(const Ice::Exception &e){ std::cout << e << std::endl;}
}

int SpecificWorker::asignarIDHabitacion()
{
    int roomID = nextRoomID;
    nextRoomID++;
    return roomID;
}

void SpecificWorker::inicializarGrafo()
{
    for (int i = 0; i < NUM_HABITACIONES; ++i)
    {
        int currentRoomID = asignarIDHabitacion();
        graph.add_node(currentRoomID);
    }

    graph.add_edge(1, 2);
    graph.add_edge(2, 3);
    graph.add_edge(3, 4);

    graph.print();
}

////////////////////////////////////////////////////////////////////////////
int SpecificWorker::startup_check()
{
    std::cout << "Startup check" << std::endl;
    QTimer::singleShot(200, qApp, SLOT(quit()));
    inicializarGrafo();
    return 0;
}

void SpecificWorker::draw_lidar(const RoboCompLidar3D::TPoints &points, AbstractGraphicViewer *viewer)
{
    static std::vector<QGraphicsItem*> borrar;
    for(auto &b : borrar) {
        viewer->scene.removeItem(b);
        delete b;
    }
    borrar.clear();

    for(const auto &p : points)
    {
        auto point = viewer->scene.addRect(-50,-50,100, 100,
                                           QPen(QColor("red")), QBrush(QColor("red")));
        point->setPos(p.x, p.y);
        borrar.push_back(point);
    }
}

void SpecificWorker::draw_target_door(const Door &target, AbstractGraphicViewer *viewer, QColor color, QColor color_far)
{
    static std::vector<QGraphicsItem *> borrar;
    for (auto &b: borrar) {
        viewer->scene.removeItem(b);
        delete b;
    }
    borrar.clear();

    auto perp = door_target.point_perpendicular_to_door_at();
    auto middle = viewer->scene.addRect(-100, -100, 200, 200, color, QBrush(color));
    middle->setPos(perp.first.x(), perp.first.y());
    auto middle_far= viewer->scene.addRect(-100, -100, 200, 200, color_far, QBrush(color_far));
    middle_far->setPos(perp.second.x(), perp.second.y());
    borrar.push_back(middle);
    borrar.push_back(middle_far);
}

void SpecificWorker::draw_lines(const Lines &lines, AbstractGraphicViewer *pViewer)
{
    static std::vector<QGraphicsItem*> borrar;
    for(auto &b : borrar) {
        pViewer->scene.removeItem(b);
        delete b;
    }
    borrar.clear();

    for(const auto &line : lines)
        for(const auto &p : line)
        {
            auto point = pViewer->scene.addRect(-50,-50,100, 100,
                                                QPen(QColor("red")), QBrush(QColor("red")));
            point->setPos(p.x(), p.y());
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