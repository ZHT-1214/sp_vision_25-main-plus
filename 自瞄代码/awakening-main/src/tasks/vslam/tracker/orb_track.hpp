#pragma once
#include "tasks/vslam/frame.hpp"
#include <vector>
namespace awakening::vslam {
class OrbTrack{
    void track(std::vector<Frame> &frames){

    }
    void track_one_frame(Frame &frame){
        current = frame;
        if(fsm == FSM::NOT_INITIALIZED){
        }
    }
    void init_mono(){
        
        if(!mono_init.ready){
            if(current.keypoints.size() >100){
                mono_init.frame_0 = current;
                mono_init.ready = true;
            }
        }else {
            if(current.keypoints.size() <100){
                mono_init.ready = false;
            }
            
        }
    }
    struct MonoInit{
        bool ready = false;
        Frame frame_0;
    }mono_init;

    Frame current;
    Frame last;
    int seq =-1;
    enum FSM {NOT_INITIALIZED,OK}fsm;
};
}