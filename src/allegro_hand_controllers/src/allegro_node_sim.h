#ifndef PROJECT_ALLEGRO_NODE_SIM_HPP
#define PROJECT_ALLEGRO_NODE_SIM_HPP


#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/float32.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "allegro_node.h"
// Forward class declaration.
class BHand;

// Simulated Allegro Hand.
//
// Simulated is probably a generous term, this is simply a pass-through for
// joint states: commanded -> current.
//
// It works by overriding `updateController` and setting the current position in
// computeDesiredTorque.
// Author: Leo

class AllegroNodeSim : public AllegroNode {
 public:
    AllegroNodeSim(const std::string node_name);

    ~AllegroNodeSim();

    void initController(const std::string &whichHand, const std::string &whichType);

    void computeDesiredTorque();

    void updateController();

    void doIt(bool polling);

 protected:

};

#endif //PROJECT_ALLEGRO_NODE_SIM_HPP
