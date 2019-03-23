#include <fstream>
#include <iostream>
#include <ctime>
void SimAnnPlanner::my_stuff_evaluate_grasps()
{


  Eigen::Vector3d base_center;
  Eigen::Quaterniond base_quat;
  std::ifstream file;
  std::ofstream output_file;
  file.open("/home/suhail/catkin_ws/src/gpg/Grasps/gpg_grasps.txt");
  output_file.open("/home/suhail/catkin_ws/src/gpg/Grasps/output_quality.txt", std::ios::out | std::ios::app);
  clock_t begin = clock();
  while(file.read((char*)&base_center,sizeof(base_center)))
  {
    file.read((char*)&base_quat,sizeof(base_quat));
    typedef Eigen::Transform<double, 3, Eigen::Affine> EigenTransform;
    EigenTransform Global_Robot_Transform, Local_Robot_Transform, Final_Robot_Transform;
    Global_Robot_Transform.setIdentity();Local_Robot_Transform.setIdentity();
    Global_Robot_Transform.translate(Eigen::Vector3d(base_center.x(),base_center.y(),base_center.z()));
    base_quat.normalize();
    Global_Robot_Transform.rotate(base_quat);
    Local_Robot_Transform.translate(Eigen::Vector3d(-0.12,0.0,0.0));
    Final_Robot_Transform = Global_Robot_Transform * Local_Robot_Transform;


    double energy;
    bool legal;
    PositionState* current_state = mCurrentState->getPosition();
    PostureState* current_state_posture = mCurrentState->getPosture();
    SearchVariable *var;
    // v = current_state->readVariable(2) - 10;
    double v;
    var = current_state_posture->getVariable(0);
    v = 0.30;
    var->setValue(v);

    vec3 newTranslation(Final_Robot_Transform.translation()[0] * 1000.0,
                        Final_Robot_Transform.translation()[1] * 1000.0,
                        Final_Robot_Transform.translation()[2] * 1000.0);

    Quaternion newRotation(base_quat.w(),
                          base_quat.x(),
                           base_quat.y(),
                           base_quat.z()
                           );

    transf newTransform(newRotation, newTranslation);
    current_state->setTran(newTransform);

    // current_state->print();
    mEnergyCalculator->analyzeState(legal, energy, mCurrentState);
    a+=1;
    // if(v==0)
    // {
    // if(legal == 0)
    // {
    //   std::cerr<<"legal: "<<legal<<" energy "<<energy<<" "<<a<<'\n';
    output_file.write((char*)&legal,sizeof(legal));
    output_file.write((char*)&energy,sizeof(energy));

    //   current_state->print();
    //   // render();
    // }
    // // std::cerr<<"Position "<<mCurrentState->getPosition()->getNumVariables()<<" Posture "<<mCurrentState->getPosture()->getNumVariables()<<'\n';
    // }

    // render();
  }
  clock_t end = clock();
  std::cerr<<double(end - begin) / CLOCKS_PER_SEC<<'\n';
  // break;
  output_file.close();
  file.close();
}
