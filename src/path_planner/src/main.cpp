#include "base/frame.h"


int main(int argc, char** argv)
{
    ros::init(argc, argv, "path_optimization");
    frame f;

    ros::Rate rate(30);

    while(f.n_.ok()){

        f.marker_Vis();

        if(f.planReady()){
            f.Plan();
            
        }

        f.LoopAction();
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}