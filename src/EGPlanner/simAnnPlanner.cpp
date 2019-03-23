//######################################################################
//
// GraspIt!
// Copyright (C) 2002-2009  Columbia University in the City of New York.
// All rights reserved.
//
// GraspIt! is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// GraspIt! is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with GraspIt!.  If not, see <http://www.gnu.org/licenses/>.
//
// Author(s): Matei T. Ciocarlie
//
// $Id: simAnnPlanner.cpp,v 1.16 2009/05/07 19:57:26 cmatei Exp $
//
//######################################################################

#include "graspit/EGPlanner/simAnnPlanner.h"

#include "graspit/EGPlanner/searchState.h"
#include "graspit/EGPlanner/energy/searchEnergy.h"
#include "graspit/EGPlanner/simAnn.h"

//#define GRASPITDBG
#include "graspit/debug.h"

#include "graspit/grasp.h"
//! How many of the best states are buffered. Should be a parameter
#define BEST_LIST_SIZE 20
//! Two states within this distance of each other are considered to be in the same neighborhood
#define DISTANCE_THRESHOLD 0.3

SimAnnPlanner::SimAnnPlanner(Hand *h)
{
  mHand = h;
  init();
  mEnergyCalculator = SearchEnergy::getSearchEnergy("CONTACT_ENERGY");
  mSimAnn = new SimAnn();
  //mSimAnn->writeResults(true);
}

SimAnnPlanner::~SimAnnPlanner()
{
  if (mSimAnn) { delete mSimAnn; }
}

void
SimAnnPlanner::setAnnealingParameters(SimAnnParams params) {
  if (isActive()) {
    DBGA("Stop planner before setting ann parameters");
    return;
  }
  mSimAnn->setParameters(params);
}

void
SimAnnPlanner::resetParameters()
{
  EGPlanner::resetParameters();
  mSimAnn->reset();
  mCurrentStep = mSimAnn->getCurrentStep();
  mCurrentState->setEnergy(1.0e8);
}

bool
SimAnnPlanner::initialized()
{
  if (!mCurrentState) { return false; }
  return true;
}

int
SimAnnPlanner::getStartingStep()
{
    return mSimAnn->getStartingStep();
}

void
SimAnnPlanner::setModelState(const GraspPlanningState *modelState)
{
  if (isActive()) {
    DBGA("Can not change model state while planner is running");
    return;
  }

  if (mCurrentState) { delete mCurrentState; }
  mCurrentState = new GraspPlanningState(modelState);
  mCurrentState->setEnergy(1.0e5);
  //my hand might be a clone
  mCurrentState->changeHand(mHand, true);

  if (mTargetState && (mTargetState->readPosition()->getType() != mCurrentState->readPosition()->getType() ||
                       mTargetState->readPosture()->getType() != mCurrentState->readPosture()->getType())) {
    delete mTargetState; mTargetState = NULL;
  }
  if (!mTargetState) {
    mTargetState = new GraspPlanningState(mCurrentState);
    mTargetState->reset();
    mInputType = INPUT_NONE;
  }
  invalidateReset();
}
double a = 0;
#include "my_stuff.h"
void
SimAnnPlanner::mainLoop()
{
  GraspPlanningState *input = NULL;
  if (processInput()) {
    input = mTargetState;
  }

  //call sim ann

  //-------------------------My stuff-------------------------

  my_stuff_evaluate_grasps();
  // double energy;
  // bool legal;
  // mEnergyCalculator->analyzeState(legal, energy, mCurrentState);
  // // std::cerr<<"legal: "<<legal<<" energy "<<energy<<'\n';
  // // std::cerr<<"Position "<<mCurrentState->getPosition()->getNumVariables()<<" Posture "<<mCurrentState->getPosture()->getNumVariables()<<'\n';
  // PositionState* current_state = mCurrentState->getPosition();
  // PostureState* current_state_posture = mCurrentState->getPosture();
  //
  // // std::cerr<<"x "<<current_state->readVariable(0)<<' '<< current_state->readVariable(1)<<" "<<current_state->readVariable(2)<<
  // // " "<<current_state->readVariable(3)<<" "<<current_state->readVariable(4)<<" "<<current_state->readVariable(5)<<'\n';
  // SearchVariable *var;
  // // v = current_state->readVariable(2) - 10;
  // double v;
  // var = current_state_posture->getVariable(0);
  // v = 0.20;
  // var->setValue(v);
  //
  // vec3 newTranslation(0.134 * 1000.0,
  //                     0.114 * 1000.0,
  //                     0.17 * 1000.0);
  //
  // Quaternion newRotation(0.008,
  //                       -0.3424,
  //                        0.939,
  //                        0.0089
  //                        );
  //
  // transf newTransform(newRotation, newTranslation);
  // current_state->setTran(newTransform);
  //
  // // current_state->print();
  // mEnergyCalculator->analyzeState(legal, energy, mCurrentState);
  // a+=1;
  // // if(v==0)
  // // {
  // // current_state->print();
  // std::cerr<<"legal: "<<legal<<" energy "<<energy<<" "<<a<<'\n';
  // // // std::cerr<<"Position "<<mCurrentState->getPosition()->getNumVariables()<<" Posture "<<mCurrentState->getPosture()->getNumVariables()<<'\n';
  // // }
  //
  // // render();

  //------------------------My stuff-----------------------------

  // SimAnn::Result result = mSimAnn->iterate(mCurrentState, mEnergyCalculator, input);
  // if (result == SimAnn::FAIL) {
  //   DBGP("Sim ann failed");
  //   return;
  // }
  // DBGP("Sim Ann success");
  // //
  // // //put result in list if there's room or it's better than the worst solution so far
  // double worstEnergy;
  // if ((int)mBestList.size() < BEST_LIST_SIZE) { worstEnergy = 1.0e5; }
  // else { worstEnergy = mBestList.back()->getEnergy(); }
  // if (result == SimAnn::JUMP && mCurrentState->getEnergy() < worstEnergy) {
  //   GraspPlanningState *insertState = new GraspPlanningState(mCurrentState);
  //   //but check if a similar solution is already in there
  //   if (!addToListOfUniqueSolutions(insertState, &mBestList, 0.2)) {
  //     delete insertState;
  //   } else {
  //     mBestList.sort(GraspPlanningState::compareStates);
  //     while ((int)mBestList.size() > BEST_LIST_SIZE) {
  //       delete(mBestList.back());
  //       mBestList.pop_back();
  //     }
  //   }
  // }
  // render();
  std::cerr<<" Not Here"<<'\n';
  mCurrentStep = 60000;
  // mCurrentStep = mSimAnn->getCurrentStep();
  // if (mCurrentStep % 100 == 0 && !mMultiThread) { Q_EMIT update(); }
  // if (mMaxSteps == 200) {DBGP("Child at " << mCurrentStep << " steps");}
}
