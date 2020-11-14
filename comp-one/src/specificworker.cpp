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
    graphicsView->setScene(&scene);
	graphicsView->setMinimumSize(400,400);
    scene.setItemIndexMethod(QGraphicsScene::NoIndex);
    scene.setSceneRect(dim.HMIN, dim.VMIN, dim.WIDTH, dim.HEIGHT);
    graphicsView->scale(1, -1);
    graphicsView->fitInView(scene.sceneRect(), Qt::KeepAspectRatio );
    graphicsView->show();

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

    // target
    target = QPointF(0, 2200);

    // path
    for(auto i : iter::range(0, 2200, 100))
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
    auto bState = read_base();
    auto laser_poly = read_laser();
    fill_grid(laser_poly);
    auto [x,z,alpha] = state_change(bState, 0.1);  //secs
    draw_path();
    qInfo() << bState.x << bState.z << bState.alpha << "--" << x << z << alpha;
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
    
    for (uint e = 0; e < np; e++)
    {
        ostringstream vnamex, vnamey;
        vnamex << "x" << e;
        model_vars[e*2].set(GRB_StringAttr_VarName, vnamex.str());
        vnamey << "y" << e;
        model_vars[e*2+1].set(GRB_StringAttr_VarName, vnamey.str());
    }
    for (uint e = np; e < 2*np; e++)
    {
        ostringstream vnameu, vnamev;
        vnameu << "u" << e-path.size();
        model_vars[e*2].set(GRB_StringAttr_VarName, vnameu.str());
        vnamev << "v" << e-path.size();
        model_vars[e*2+1].set(GRB_StringAttr_VarName, vnamev.str());
    }

    GRBQuadExpr fpoint, lpoint;
    fpoint = model_vars[0]*model_vars[0];
    fpoint += model_vars[1]*model_vars[1];
    lpoint = (model_vars[(np-1)*2]-0)*(model_vars[(np-1)*2]-0);
    lpoint +=(model_vars[(np-1)*2+1]-2200.)*(model_vars[(np-1)*2+1]-2200.);

    model->addQConstr(fpoint <= 0.00001, "c0");
    model->addQConstr(lpoint <= 0.00001, "c1");

    GRBQuadExpr obj;
    obj = 0;
    for (uint e = 0; e < np-1; e++)
    {
        obj += (model_vars[e*2]-model_vars[(e+1)*2])*(model_vars[e*2]-model_vars[(e+1)*2]);
        obj += (model_vars[e*2+1]-model_vars[(e+1)*2+1])*(model_vars[e*2+1]-model_vars[(e+1)*2+1]);
    }
    model->setObjective(obj, GRB_MINIMIZE);
    model->optimize();

    path.clear();
    for(uint e = 0; e < np; e++)
    {
        float x = model_vars[e*2].get(GRB_DoubleAttr_X);
        cout << model_vars[e*2].get(GRB_StringAttr_VarName) << " "
         << x << endl;
        float y = model_vars[e*2+1].get(GRB_DoubleAttr_X);
        cout << model_vars[e*2+1].get(GRB_StringAttr_VarName) << " "
         << y << endl;
        path.emplace_back(QPointF(x, y));
    }
}
void SpecificWorker::optimize()
{

}

//////////////////////////////////////////////////////////////////////////////////////////////////////
RoboCompGenericBase::TBaseState SpecificWorker::read_base()
{
    RoboCompGenericBase::TBaseState bState;
    try
    {
        differentialrobot_proxy->getBaseState(bState);
        innerModel->updateTransformValues("base", bState.x, 0, bState.z, 0, bState.alpha, 0);
        robot_polygon->setRotation(qRadiansToDegrees(-bState.alpha));
        robot_polygon->setPos(bState.x, bState.z);
    }
    catch(const Ice::Exception &e)
    { std::cout << "Error reading from Camera" << e << std::endl;}
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

