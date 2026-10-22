// state_estimator executable: the E1/E2 executor layout of A.4.2 for one node.
//
//   E1  SingleThreadedExecutor  imu_grp only (propagate + publish)
//   E2  SingleThreadedExecutor  cam_grp + the node's default group
//                               (camera correction, lifecycle, parameters)
//
// Optional SCHED_FIFO priority for E1 via `fast_thread_priority` (0 = off).
// On a non-RT kernel or without privilege this logs a warning and continues;
// figures gathered that way are development evidence only (TD-10).

#include <pthread.h>
#include <sched.h>

#include <cstring>
#include <memory>
#include <thread>

#include "rclcpp/rclcpp.hpp"

#include "pat_estimator/state_estimator_node.hpp"

namespace
{

void try_set_fifo(const char * what, int prio, rclcpp::Logger log)
{
  if (prio <= 0) {return;}
  sched_param sp{};
  sp.sched_priority = prio;
  const int rc = pthread_setschedparam(pthread_self(), SCHED_FIFO, &sp);
  if (rc != 0) {
    RCLCPP_WARN(log, "%s: SCHED_FIFO %d not applied (%s); running at normal priority",
      what, prio, std::strerror(rc));
  } else {
    RCLCPP_INFO(log, "%s: SCHED_FIFO %d", what, prio);
  }
}

}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  rclcpp::NodeOptions options;
  options.use_intra_process_comms(true);
  auto node = std::make_shared<pat_estimator::StateEstimatorNode>(options);

  // Not declared by the node: it is a property of this process layout.
  const int prio = static_cast<int>(node->declare_parameter<int64_t>("fast_thread_priority", 0));

  rclcpp::executors::SingleThreadedExecutor e1;
  rclcpp::executors::SingleThreadedExecutor e2;
  e1.add_callback_group(node->fast_group(), node->get_node_base_interface());
  e2.add_node(node->get_node_base_interface());   // default group (lifecycle, params)
  e2.add_callback_group(node->slow_group(), node->get_node_base_interface());

  std::thread fast([&] {
      try_set_fifo("E1 fast executor", prio, node->get_logger());
      e1.spin();
    });
  e2.spin();
  e1.cancel();
  fast.join();
  rclcpp::shutdown();
  return 0;
}
