/*
 *    Copyright (C) 2020 by YOUR NAME HERE
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
}

/**
* \brief Default destructor
*/
SpecificWorker::~SpecificWorker()
{
    delete env;
	std::cout << "Destroying SpecificWorker" << std::endl;
}

bool SpecificWorker::setParams(RoboCompCommonBehavior::ParameterList params)
{
	try
	{
		RoboCompCommonBehavior::Parameter par = params.at("InnerModelPath");
		std::string innermodel_path = par.value;
		innerModel = std::make_shared<InnerModel>(innermodel_path);

	}
	catch(const std::exception &e) { qFatal("Error reading config params"); }
	return true;
}

void SpecificWorker::initialize(int period)
{
	std::cout << "Initialize worker" << std::endl;

	//grid
	Grid<>::Dimensions dim;  //default values
    grid.initialize(&scene, dim);

    //view
    init_drawing(dim);

    // target
    target = QPointF(500, 2200);
    newTarget = true;

    // path
    for(auto i : iter::range(0, 2300, 100))
        path.emplace_back(QPointF(100 - qrand()%200, i));
    draw_path();

    initialize_model();

	this->Period = period;
	if(this->startup_check_flag)
		this->startup_check();
	else
		timer.start(Period);
}

void SpecificWorker::init_drawing( Grid<>::Dimensions dim)
{
    graphicsView->setScene(&scene);
    graphicsView->setMinimumSize(400,400);
    scene.setItemIndexMethod(QGraphicsScene::NoIndex);
    scene.setSceneRect(dim.HMIN, dim.VMIN, dim.WIDTH, dim.HEIGHT);
    graphicsView->scale(1, -1);
    graphicsView->fitInView(scene.sceneRect(), Qt::KeepAspectRatio );
    graphicsView->show();
    connect(&scene, &MyScene::new_target, this, [this](QGraphicsSceneMouseEvent *e)
    {
        qDebug() << "Lambda SLOT: " << e->scenePos();
        target = QPointF(e->scenePos().x() , e->scenePos().y());
        newTarget = true;
        // target_buffer.put(Eigen::Vector2f ( e->scenePos().x() , e->scenePos().y()));
        // atTarget = false;
    });

    //robot
    QPolygonF poly2;
    float size = ROBOT_LENGTH / 2.f;
    poly2 << QPoint(-size, -size)
          << QPoint(-size, size)
          << QPoint(-size / 3, size * 1.6)
          << QPoint(size / 3, size * 1.6)
          << QPoint(size, size)
          << QPoint(size, -size);
    QBrush brush;
    brush.setColor(QColor("DarkRed"));
    brush.setStyle(Qt::SolidPattern);
    robot_polygon = scene.addPolygon(poly2, QPen(QColor("DarkRed")), brush);
    robot_polygon->setZValue(5);
    robot_polygon->setPos(0,0);

}

void SpecificWorker::compute()
{
    auto bState = read_base();
    // auto laser_poly = read_laser();
    // fill_grid(laser_poly);
    // auto [x,z,alpha] = state_change(bState, 0.1);  //secs
    if(newTarget)
    {
        QPointF x0(bState.x, bState.z);
        double pos_error = sqrt(pow(x0.x() - target.x(),2) + pow(x0.y() - target.y(),2));
        //double rot_error = sqrt(pow(x0.z() - xRef.z(),2));
        if (pos_error < 40)
        {
            omnirobot_proxy->setSpeedBase(0, 0, 0);
            std::cout << "FINISH" << std::endl;
            newTarget = false;
            return;
        }
        else
        {
            optimize(bState);
            float x = vel_vars[0].get(GRB_DoubleAttr_X);
            float y = vel_vars[1].get(GRB_DoubleAttr_X);
            omnirobot_proxy->setSpeedBase(x, y, 0);
        }
    }
    draw_path();
    //qInfo() << bState.x << bState.z << bState.alpha;
}

void SpecificWorker::initialize_model()
{
    // Create environment
    env = new GRBEnv("path_optimization.log");

    // Create initial model
    model = new GRBModel(*env);
    model->set(GRB_StringAttr_ModelName, "path_optimization");

    uint np = path.size();
    model_vars = model->addVars(4*np, GRB_CONTINUOUS);
    pose_vars = &(model_vars[0]);
    vel_vars = &(model_vars[2*np]);
    //sin_cos_vars = &(model_vars[4*np]);
    
    for (uint e = 0; e < np; e++)
    {
        ostringstream vnamex, vnamey;
        vnamex << "x" << e;
        pose_vars[e*2].set(GRB_StringAttr_VarName, vnamex.str());
        pose_vars[e*2].set(GRB_DoubleAttr_LB, -10000);
        pose_vars[e*2].set(GRB_DoubleAttr_UB, +10000);
        //pose_vars[e*2].set(GRB_DoubleAttr_Start, path[e].x());
        vnamey << "y" << e;
        pose_vars[e*2+1].set(GRB_StringAttr_VarName, vnamey.str());
        pose_vars[e*2+1].set(GRB_DoubleAttr_LB, -10000);
        pose_vars[e*2+1].set(GRB_DoubleAttr_UB, +10000);

        //pose_vars[e*2+1].set(GRB_DoubleAttr_Start, path[e].y());
    }
    for (uint e = 0; e < np; e++)
    {
        ostringstream vnameu, vnamev, vnamesin, vnamecos;
        vnameu << "u" << e;
        vel_vars[e*2].set(GRB_StringAttr_VarName, vnameu.str());
        vel_vars[e*2].set(GRB_DoubleAttr_LB, -10000);
        vel_vars[e*2].set(GRB_DoubleAttr_UB, +10000);

        vnamev << "v" << e;
        vel_vars[e*2+1].set(GRB_StringAttr_VarName, vnamev.str());
        vel_vars[e*2+1].set(GRB_DoubleAttr_LB, -10000);
        vel_vars[e*2+1].set(GRB_DoubleAttr_UB, +10000);

        // vnamesin << "sin_v" << e;
        // sin_cos_vars[e*2].set(GRB_StringAttr_VarName, vnamesin.str());
        // vnamecos << "cos_v" << e;
        // sin_cos_vars[e*2+1].set(GRB_StringAttr_VarName, vnamecos.str());

    }

    model->addConstr(pose_vars[0] == 0, "c0x");
    model->addConstr(pose_vars[1] == 0, "c0y");
    model->addConstr(pose_vars[(np-1)*2] == target.x(), "c1x");
    model->addConstr(pose_vars[(np-1)*2+1] == target.y(), "c1y");


    for (uint e = 0; e < np-1; e++)
    {
        ostringstream vnamecx, vnamecy;
        GRBLinExpr le, re;

        vnamecx << "cx" << e+2;
        le = pose_vars[e*2] + vel_vars[e*2];
        re = pose_vars[(e+1)*2];
        model->addConstr( le == re, vnamecx.str());
        
        vnamecy << "cy" << e+2;
        le = pose_vars[e*2+1] + vel_vars[e*2+1];        
        re = pose_vars[(e+1)*2+1];
        model->addConstr(le == re, vnamecy.str());
    }

    // for (uint e = 0; e < np-1; e++)
    // {
    //     ostringstream vnamecx, vnamecy, vnamecsin, vnameccos;
    //     GRBQuadExpr le, re;

    //     vnamecx << "cx" << e+2;
    //     le = pose_vars[e*2] + vel_vars[e*2]*sin_cos_vars[e*2+1];
    //     re = pose_vars[(e+1)*2];
    //     model->addQConstr( le == re, vnamecx.str());
        
    //     vnamecy << "cy" << e+2;
    //     le = pose_vars[e*2+1] + vel_vars[e*2]*sin_cos_vars[e*2];        
    //     re = pose_vars[(e+1)*2+1];
    //     model->addQConstr(le == re, vnamecy.str());

    //     // vnamecsin << "csin" << e+2;
    //     // model->addGenConstrSin(vel_vars[e*2+1], sin_cos_vars[e*2], vnamecsin.str());
    //     // vnameccos << "ccos" << e+2;
    //     // model->addGenConstrCos(vel_vars[e*2+1], sin_cos_vars[e*2+1], vnameccos.str());


    // }

    obj = 0;
    for (uint e = 0; e < np-1; e++)
    {
        obj += vel_vars[e*2]*vel_vars[e*2];
        obj += vel_vars[e*2+1]*vel_vars[e*2+1];

        //obj += (pose_vars[e*2]-pose_vars[(e+1)*2])*(pose_vars[e*2]-pose_vars[(e+1)*2]);
        //obj += (pose_vars[e*2+1]-pose_vars[(e+1)*2+1])*(pose_vars[e*2+1]-pose_vars[(e+1)*2+1]);
    }
    model->update();

}
void SpecificWorker::optimize(const RoboCompGenericBase::TBaseState &bState)
{
    uint np = path.size();

    auto c0x = model->getConstrByName("c0x");
    model->remove(c0x);
    auto c0y = model->getConstrByName("c0y");
    model->remove(c0y);
    auto c1x = model->getConstrByName("c1x");
    model->remove(c1x);
    auto c1y = model->getConstrByName("c1y");
    model->remove(c1y);
    model->update();
    model->addConstr(pose_vars[0] == bState.x, "c0x");
    model->addConstr(pose_vars[1] == bState.z, "c0y");
    model->addConstr(pose_vars[(np-1)*2] == target.x(), "c1x");
    model->addConstr(pose_vars[(np-1)*2+1] == target.y(), "c1y");
    model->update();
    model->setObjective(obj, GRB_MINIMIZE);
    model->optimize();


    cout << "before optimizing" << endl;
    for(uint e = 0; e < np; e++)
    {
        float x = pose_vars[e*2].get(GRB_DoubleAttr_Start);
        cout << pose_vars[e*2].get(GRB_StringAttr_VarName) << " " << x << endl;
        float y = pose_vars[e*2+1].get(GRB_DoubleAttr_Start);
        cout << pose_vars[e*2+1].get(GRB_StringAttr_VarName) << " " << y << endl;
    
    }

    cout << "after optimizing" << endl;
    path.clear();
    for(uint e = 0; e < np; e++)
    {
        float x = pose_vars[e*2].get(GRB_DoubleAttr_X);
        cout << pose_vars[e*2].get(GRB_StringAttr_VarName) << " "
         << x << endl;
        float y = pose_vars[e*2+1].get(GRB_DoubleAttr_X);
        cout << pose_vars[e*2+1].get(GRB_StringAttr_VarName) << " "
         << y << endl;
        path.emplace_back(QPointF(x, y));
    }
    for(uint e = 0; e < np; e++)
    {
        float x = vel_vars[e*2].get(GRB_DoubleAttr_X);
        cout << vel_vars[e*2].get(GRB_StringAttr_VarName) << " "
         << x << endl;
        float y = vel_vars[e*2+1].get(GRB_DoubleAttr_X);
        cout << vel_vars[e*2+1].get(GRB_StringAttr_VarName) << " "
         << y << endl;
    }

}

//////////////////////////////////////////////////////////////////////////////////////////////////////
RoboCompGenericBase::TBaseState SpecificWorker::read_base()
{
    RoboCompGenericBase::TBaseState bState;
    try
    {
        omnirobot_proxy->getBaseState(bState);
        innerModel->updateTransformValues("base", bState.x, 0, bState.z, 0, bState.alpha, 0);
        robot_polygon->setRotation(qRadiansToDegrees(-bState.alpha));
        robot_polygon->setPos(bState.x, bState.z);
    }
    catch(const Ice::Exception &e)
    { 
    //    std::cout << "Error reading from Camera" << e << std::endl;
    }
    return bState;
}

QPolygonF SpecificWorker::read_laser()
{
    QPolygonF laser_poly;
    try
    {
        auto ldata = laser_proxy->getLaserData();
        laser_poly = draw_laser(ldata);
    }
    catch(const Ice::Exception &e)
    { std::cout << "Error reading from Camera" << e << std::endl;}
    return laser_poly;
}

std::tuple<float,float,float> SpecificWorker::state_change( const RoboCompGenericBase::TBaseState &bState, float delta_t)
{
    float rot = bState.rotV;
    float adv = bState.advVz;

    if (fabs(rot) > 0.05)
    {
        float r = adv / rot; //radio
        float x = r - r * cos(rot * delta_t);
        float z = r * sin(rot * delta_t);
        QVec res =  innerModel->transform6D("world", QVec::vec6(x, 0, z, 0, rot * delta_t, 0), "base");
        return std::make_tuple(res.x(), res.z(), res.ry());
    }
    else
    {
        QVec res = innerModel->transform6D("world", QVec::vec6(0, 0,adv * delta_t, 0, 0, 0), "base");
        return std::make_tuple(res.x(), res.z(), res.ry());
    }
}

void SpecificWorker::fill_grid(const QPolygonF &laser_poly)
{
    for(auto &[k, v] : grid)
        if(laser_poly.containsPoint(QPointF(k.x, k.z), Qt::OddEvenFill))
            v.free = true;
        else
            v.free = false;
    grid.draw(&scene);
}

QPolygonF SpecificWorker::draw_laser(const RoboCompLaser::TLaserData &ldata)
{
    if (laser_polygon != nullptr)
        scene.removeItem(laser_polygon);

    QPolygonF poly;
    for( auto &l : ldata)
        poly << robot_polygon->mapToScene(QPointF(l.dist * sin(l.angle), l.dist * cos(l.angle)));

    QColor color("LightGreen");
    color.setAlpha(40);
    laser_polygon = scene.addPolygon(poly, QPen(color), QBrush(color));
    laser_polygon->setZValue(3);
    return poly;
}

void SpecificWorker::draw_path()
{
    for(auto p : path_paint)
        scene.removeItem(p);
    path_paint.clear();
    for(auto &p : path)
        path_paint.push_back(scene.addEllipse(p.x()-25, p.y()-25, 50 , 50, QPen(path_color), QBrush(QColor(path_color))));
}

///////////////////////////////////////////////////////////////////////////////////////////////
int SpecificWorker::startup_check()
{
	std::cout << "Startup check" << std::endl;
	QTimer::singleShot(200, qApp, SLOT(quit()));
	return 0;
}

