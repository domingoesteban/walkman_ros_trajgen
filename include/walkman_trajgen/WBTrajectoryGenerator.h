#ifndef WBTRAJECTORYGENERATOR_H
#define WBTRAJECTORYGENERATOR_H


#include <vector>
#include <queue>

class WBTrajectoryGenerator {
public:
  WBTrajectoryGenerator(unsigned int n_joints);
  ~WBTrajectoryGenerator();

  int GetNewWBJoints();

private:
  std::vector<std::queue<float>> trajectory_vector_;

};

#endif //WBTRAJECTORYGENERATOR_H
