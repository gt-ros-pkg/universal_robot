#ifndef UR_TEST_CONTROLLER_H
#define UR_TEST_CONTROLLER_H

#include <ur_ctrl_server/ur_controller_iface.h>

namespace ur {

class URSimController : public URControllerInterface
{
protected:

  // Read state of the robot from UR robotinterface
  void readRobotState();
  // Send robot commands to joints using robotinterface
  void sendRobotCommands();

public:
  URSimController(SimpleSocket* socket_conn);
  ~URSimController() {}
  virtual int initRobot(int argc, char** argv);

private:
  double qdd_act[6];
};

}

#endif // UR_TEST_CONTROLLER_H
