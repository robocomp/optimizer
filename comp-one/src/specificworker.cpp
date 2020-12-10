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
    readSettings();
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
    target = QPointF(0, 2000);
    target_ang = 0.;
    newTarget = false;

    // path
    for(auto i : iter::range(0, 2100, 250))
        path.emplace_back(QPointF(100 - qrand()%200, i));
    draw_path();

    initialize_model();

	this->Period = period;
	if(this->startup_check_flag)
		this->startup_check();
	else
		timer.start(Period);
}


void SpecificWorker::compute()
{
    static int cont=0;
    auto bState = read_base();
    // auto laser_poly = read_laser();
    // fill_grid(laser_poly);
    // auto [x,z,alpha] = state_change(bState, 0.1);  //secs

    // check for new target
    if(auto t = target_buffer.try_get(); t.has_value())
    {
        // angular reference along line connecting robot and target when clicked
        float tr_x = t.value().x()-bState.x; float tr_y = t.value().y() - bState.z;
        target.setX(t.value().x()); target.setY(t.value().y());
        float ref_ang = -atan2(tr_x, tr_y);   // signo menos para tener ángulos respecto a Y CCW

        // draw
        if(target_draw) scene.removeItem(target_draw);
        target_draw = scene.addEllipse(t.value().x()-50, t.value().y()-50, 100, 100, QPen(QColor("green")), QBrush(QColor("green")));
        auto ex = t.value().x() + 350*sin(-ref_ang);
        auto ey  =  t.value().y() + 350*cos(-ref_ang);  //OJO signos porque el ang está respecto a Y CCW
        auto line = scene.addLine(t.value().x(), t.value().y(), ex, ey, QPen(QBrush(QColor("green")), 20));
        line->setParentItem(target_draw);
        auto ball = scene.addEllipse(ex-25, ey-25, 50, 50, QPen(QColor("green")), QBrush(QColor("green")));
        ball->setParentItem(target_draw);
        cont = 0;
        xGraph->data()->clear();wGraph->data()->clear();yGraph->data()->clear();exGraph->data()->clear();ewGraph->data()->clear();
    }
    if(not atTarget)
    {
        QPointF x0(bState.x, bState.z);
        double pos_error = sqrt(pow(x0.x() - target.x(),2) + pow(x0.y() - target.y(),2));
        //double rot_error = bState.alpha - );
        rtarget =  innerModel->transform("base", QVec::vec3(target.x(), 0., target.y()), "world");
        target_ang = atan2(rtarget[0], rtarget[2]);
        qDebug()<<"target ang"<<target_ang;
        //double rot_error = sqrt(pow(x0.z() - xRef.z(),2));
        if (pos_error < 40)
        {
            omnirobot_proxy->setSpeedBase(0, 0, 0);
            std::cout << "FINISH" << std::endl;
            newTarget = false;
            atTarget = true;
        }
        else
        {
            optimize();
            float x = vel_vars[0].get(GRB_DoubleAttr_X);
            float y = vel_vars[1].get(GRB_DoubleAttr_X);
            float a = vel_vars[2].get(GRB_DoubleAttr_X);
            omnirobot_proxy->setSpeedBase(x, y, a);

            // draw
            xGraph->addData(cont, x);
            yGraph->addData(cont, x);
            wGraph->addData(cont, x*300);  // visual scale
            exGraph->addData(cont, pos_error);
            //ewGraph->addData(cont, rot_error*300);  // visual scale
            cont++;
            custom_plot.replot();
        }
        
        qDebug() << "target from robot" <<rtarget;

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
    model_vars = model->addVars((NP+NV)*np, GRB_CONTINUOUS);
    pose_vars = &(model_vars[0]);
    vel_vars = &(model_vars[NP*np]);
    //sin_cos_vars = &(model_vars[4*np]);
    
    for (uint e = 0; e < np; e++)
    {
        ostringstream vnamex, vnamey, vnamea;
        vnamex << "x" << e;
        pose_vars[e*NP].set(GRB_StringAttr_VarName, vnamex.str());
        pose_vars[e*NP].set(GRB_DoubleAttr_LB, -10000);
        pose_vars[e*NP].set(GRB_DoubleAttr_UB, +10000);
        //pose_vars[e*2].set(GRB_DoubleAttr_Start, path[e].x());
        vnamey << "y" << e;
        pose_vars[e*NP+1].set(GRB_StringAttr_VarName, vnamey.str());
        pose_vars[e*NP+1].set(GRB_DoubleAttr_LB, -10000);
        pose_vars[e*NP+1].set(GRB_DoubleAttr_UB, +10000);
        vnamea << "a" << e;
        pose_vars[e*NP+2].set(GRB_StringAttr_VarName, vnamea.str());
        pose_vars[e*NP+2].set(GRB_DoubleAttr_LB, -M_PI);
        pose_vars[e*NP+2].set(GRB_DoubleAttr_UB, M_PI);


        //pose_vars[e*2+1].set(GRB_DoubleAttr_Start, path[e].y());
    }
    for (uint e = 0; e < np; e++)
    {
        ostringstream vnameu, vnamev, vnamew, vnamesin, vnamecos;
        vnameu << "u" << e;
        vel_vars[e*NV].set(GRB_StringAttr_VarName, vnameu.str());
        vel_vars[e*NV].set(GRB_DoubleAttr_LB, -10000);
        vel_vars[e*NV].set(GRB_DoubleAttr_UB, +10000);

        vnamev << "v" << e;
        vel_vars[e*NV+1].set(GRB_StringAttr_VarName, vnamev.str());
        vel_vars[e*NV+1].set(GRB_DoubleAttr_LB, -10000);
        vel_vars[e*NV+1].set(GRB_DoubleAttr_UB, +10000);

        vnamew << "w" << e;
        vel_vars[e*NV+2].set(GRB_StringAttr_VarName, vnamew.str());
        vel_vars[e*NV+2].set(GRB_DoubleAttr_LB, -M_PI);
        vel_vars[e*NV+2].set(GRB_DoubleAttr_UB, M_PI);


        // vnamesin << "sin_v" << e;
        // sin_cos_vars[e*2].set(GRB_StringAttr_VarName, vnamesin.str());
        // vnamecos << "cos_v" << e;
        // sin_cos_vars[e*2+1].set(GRB_StringAttr_VarName, vnamecos.str());

    }

    model->addConstr(pose_vars[0] == 0, "c0x");
    model->addConstr(pose_vars[1] == 0, "c0y");
    model->addConstr(pose_vars[2] == 0, "c0a");
    model->addConstr(pose_vars[(np-1)*NP] == target.x(), "c1x");
    model->addConstr(pose_vars[(np-1)*NP+1] == target.y(), "c1y");
    model->addConstr(pose_vars[(np-1)*NP+2] == target_ang, "c1a");


    for (uint e = 0; e < np-1; e++)
    {
        ostringstream vnamecx, vnamecy, vnameca;
        GRBLinExpr le, re;

        vnamecx << "cx" << e+2;
        le = pose_vars[e*NP] + vel_vars[e*NV];
        re = pose_vars[(e+1)*NP];
        model->addConstr( le == re, vnamecx.str());
        
        vnamecy << "cy" << e+2;
        le = pose_vars[e*NP+1] + vel_vars[e*NV+1];        
        re = pose_vars[(e+1)*NP+1];
        model->addConstr(le == re, vnamecy.str());

        vnameca << "ca" << e+2;
        le = pose_vars[e*NP+2] + vel_vars[e*NV+2];        
        re = pose_vars[(e+1)*NP+2];
        model->addConstr(le == re, vnameca.str());

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
        obj += vel_vars[e*NV]*vel_vars[e*NV];
        obj += vel_vars[e*NV+1]*vel_vars[e*NV+1];
        obj += vel_vars[e*NV+2]*vel_vars[e*NV+2];        

        //obj += (pose_vars[e*2]-pose_vars[(e+1)*2])*(pose_vars[e*2]-pose_vars[(e+1)*2]);
        //obj += (pose_vars[e*2+1]-pose_vars[(e+1)*2+1])*(pose_vars[e*2+1]-pose_vars[(e+1)*2+1]);
    }
    model->update();

}
void SpecificWorker::optimize()
{
    uint np = path.size();

    auto c1x = model->getConstrByName("c1x");
    model->remove(c1x);
    auto c1y = model->getConstrByName("c1y");
    model->remove(c1y);
    auto c1a = model->getConstrByName("c1a");
    model->remove(c1a);

    model->update();
    model->addConstr(pose_vars[(np-1)*NP] == rtarget[0], "c1x");
    model->addConstr(pose_vars[(np-1)*NP+1] == rtarget[2], "c1y");
    model->addConstr(pose_vars[(np-1)*NP+2] == target_ang, "c1a");
    model->update();
    model->setObjective(obj, GRB_MINIMIZE);
    model->optimize();


    // cout << "before optimizing" << endl;
    // for(uint e = 0; e < np; e++)
    // {
    //     float x = pose_vars[e*2].get(GRB_DoubleAttr_Start);
    //     cout << pose_vars[e*2].get(GRB_StringAttr_VarName) << " " << x << endl;
    //     float y = pose_vars[e*2+1].get(GRB_DoubleAttr_Start);
    //     cout << pose_vars[e*2+1].get(GRB_StringAttr_VarName) << " " << y << endl;
    
    // }

    // cout << "after optimizing" << endl;
    path.clear();
    for(uint e = 0; e < np; e++)
    {
        float x = pose_vars[e*NP].get(GRB_DoubleAttr_X);
        // cout << pose_vars[e*2].get(GRB_StringAttr_VarName) << " "
        // << x << endl;
        float y = pose_vars[e*NP+1].get(GRB_DoubleAttr_X);
        // cout << pose_vars[e*2+1].get(GRB_StringAttr_VarName) << " "
        // << y << endl;
        QVec p =  innerModel->transform("world", QVec::vec3(x, 0., y), "base");
        path.emplace_back(QPointF(p[0], p[2]));
    }
    // for(uint e = 0; e < np; e++)
    // {
    //     float x = vel_vars[e*2].get(GRB_DoubleAttr_X);
    //     cout << vel_vars[e*2].get(GRB_StringAttr_VarName) << " "
    //      << x << endl;
    //     float y = vel_vars[e*2+1].get(GRB_DoubleAttr_X);
    //     cout << vel_vars[e*2+1].get(GRB_StringAttr_VarName) << " "
    //      << y << endl;
    // }

}

//////////////////////////////////////////////////////////////////////////////////////////////////////
void SpecificWorker::init_drawing( Grid<>::Dimensions dim)
{
    graphicsView->setScene(&scene);
    graphicsView->setMinimumSize(400,400);
    scene.setSceneRect(dim.HMIN, dim.VMIN, dim.WIDTH, dim.HEIGHT);
    graphicsView->scale(1, -1);
    graphicsView->fitInView(scene.sceneRect(), Qt::KeepAspectRatio );
    graphicsView->show();
    connect(&scene, &MyScene::new_target, this, [this](QGraphicsSceneMouseEvent *e)
    {
        qDebug() << "Lambda SLOT: " << e->scenePos();
        target_buffer.put(Eigen::Vector2f ( e->scenePos().x() , e->scenePos().y()));
        atTarget = false;
        //target = QPointF(e->scenePos().x() , e->scenePos().y());
        //newTarget = true;
    });

    //Draw
    custom_plot.setParent(signal_frame);
    custom_plot.xAxis->setLabel("time");
    custom_plot.yAxis->setLabel("vx-blue vy-red vw-green dist-magenta ew-black");
    custom_plot.xAxis->setRange(0, 200);
    custom_plot.yAxis->setRange(-1500, 1500);
    xGraph = custom_plot.addGraph();
    xGraph->setPen(QColor("blue"));
    yGraph = custom_plot.addGraph();
    yGraph->setPen(QColor("red"));
    wGraph = custom_plot.addGraph();
    wGraph->setPen(QColor("green"));
    exGraph = custom_plot.addGraph();
    exGraph->setPen(QColor("magenta"));
    ewGraph = custom_plot.addGraph();
    ewGraph->setPen(QColor("black"));
    custom_plot.resize(signal_frame->size());
    custom_plot.show();

    //robot
    QPolygonF poly2;
    float size = ROBOT_LENGTH / 2.f;
    poly2 << QPoint(-size, -size)
          << QPoint(-size, size)
          << QPoint(-size / 3, size * 1.6)
          << QPoint(size / 3, size * 1.6)
          << QPoint(size, size)
          << QPoint(size, -size);
    QColor rc("DarkRed"); rc.setAlpha(60);
    robot_polygon = scene.addPolygon(poly2, QPen(QColor("DarkRed")), QBrush(rc));
    robot_polygon->setZValue(5);
    try
    {
        RoboCompGenericBase::TBaseState bState;
        omnirobot_proxy->getBaseState(bState);
        robot_polygon->setRotation(qRadiansToDegrees(bState.alpha));
        robot_polygon->setPos(bState.x, bState.z);
    }
    catch(const Ice::Exception &e){};;

    connect(splitter, &QSplitter::splitterMoved, [this](int pos, int index)
        {  custom_plot.resize(signal_frame->size()); graphicsView->fitInView(scene.sceneRect(), Qt::KeepAspectRatio); });
}

RoboCompGenericBase::TBaseState SpecificWorker::read_base()
{
    RoboCompGenericBase::TBaseState bState;
    try
    {
        omnirobot_proxy->getBaseState(bState);
        innerModel->updateTransformValues("base", bState.x, 0, bState.z, 0, -bState.alpha, 0);
        robot_polygon->setRotation(qRadiansToDegrees(bState.alpha));
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

