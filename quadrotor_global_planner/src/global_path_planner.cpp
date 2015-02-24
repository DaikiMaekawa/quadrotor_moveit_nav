#include <ros/ros.h>

#include <ompl/geometric/SimpleSetup.h>
#include <ompl/geometric/planners/rrt/RRT.h>
#include <ompl/base/spaces/SE2StateSpace.h>

#include <cmath>
#include <iostream>
#include <fstream>

namespace ob = ompl::base;
namespace og = ompl::geometric;

void planWithSimpleSetup(){
    ob::StateSpacePtr space(new ob::SE2StateSpace());
    ob::RealVectorBounds bounds(2);
    bounds.setLow(-1);
    bounds.setHigh(1);
    space->as<ob::SE2StateSpace>()->setBounds(bounds);

    og::SimpleSetup ss(space);

    //TODO: Set state validity checker
    
    ob::ScopedState<ob::SE2StateSpace> start(space);
    start->setXY(-0.9, -0.9);
    std::cout << "start: "; start.print(std::cout);

    ob::ScopedState<ob::SE2StateSpace> goal(space);
    goal->setXY(0.9, 0.9);
    std::cout << "goal: " goal.print(std::cout);

    ss.setStartAndGoalStates(start, goal);

    ob::PlannerPtr planner(new og::PRM(ss.getSpaceInformation()));
    ss.setPlanner(planner);

    std::cout << "-----------------" << std::endl;
    
    ob::PlannerStatus solved = ss.solve(1.0);

    if(solved){
        ss.simplifySolution();
        std::cout << "------------" << std::endl;
        std::cout << "Found solution:" << std::endl;
        ss.getSolutionPath().print(std::cout);

        std::ofstream ofs("path.dat");
        ss.getSolutionPath().printAsMatrix(ofs);
    }else{
        ROS_ERROR("Solution is not found");
    }
}

int main(int argc, char *argv[]){
    ros::init(argc, argv, "global_path_planner");
    planWithSimpleSetup();
    return 0;
}

