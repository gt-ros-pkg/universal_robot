#ifndef UR_TEST_CONTROLLER_H
#define UR_TEST_CONTROLLER_H

#include <ur_ctrl_server/ur_controller_iface.h>

#define CMD_TIMEOUT 3

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
  double q_cur[6];
  double qd_cur[6];
  double qdd_cur[6];

};

typedef URSimController URController;
}

#endif // UR_TEST_CONTROLLER_H
