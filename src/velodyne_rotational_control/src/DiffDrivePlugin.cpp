/*
 * Copyright (C) 2012 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/
#include <velodyne_rotational_control/DiffDrivePlugin.h>

#include <functional>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/transport.hh>


using namespace gazebo;
GZ_REGISTER_MODEL_PLUGIN(DiffDrivePlugin)

enum {LEFT};

/////////////////////////////////////////////////
DiffDrivePlugin::DiffDrivePlugin()
{
  // this->velocity = 1.0;
  // this->wheelSeparation = 1.0;
  // this->wheelRadius = 1.0;
}

/////////////////////////////////////////////////
void DiffDrivePlugin::Load(physics::ModelPtr _model,
                           sdf::ElementPtr _sdf)
{
  this->model = _model;

  this->node = transport::NodePtr(new transport::Node());
  this->node->Init(this->model->GetWorld()->Name());

  this->velSub = this->node->Subscribe(std::string("~/") +
      this->model->GetName() + "/vel_cmd", &DiffDrivePlugin::OnVelMsg, this);

  if (!_sdf->HasElement("joint_d"))
    gzerr << "DiffDrive plugin missing <joint_d> element\n";

  // if (!_sdf->HasElement("right_joint"))
  //   gzerr << "DiffDrive plugin missing <right_joint> element\n";

  this->leftJoint = _model->GetJoint(
      _sdf->GetElement("joint_d")->Get<std::string>());
  // this->rightJoint = _model->GetJoint(
  //     _sdf->GetElement("right_joint")->Get<std::string>());


  if (!this->leftJoint)
    gzerr << "Unable to find left joint["
          << _sdf->GetElement("joint_d")->Get<std::string>() << "]\n";
  // if (!this->rightJoint)
  //   gzerr << "Unable to find right joint["
  //         << _sdf->GetElement("right_joint")->Get<std::string>() << "]\n";


  this->velocity = _sdf->GetElement("velocity")->Get<double>();

  this->wheelRadius = _sdf->GetElement("radius")->Get<double>();


  this->updateConnection = event::Events::ConnectWorldUpdateBegin(
          std::bind(&DiffDrivePlugin::OnUpdate, this));
}

/////////////////////////////////////////////////
void DiffDrivePlugin::Init()
{
  // this->wheelSeparation = this->leftJoint->Anchor(0).Distance(
  //     this->rightJoint->Anchor(0));

  physics::EntityPtr parent = boost::dynamic_pointer_cast<physics::Entity>(
      this->leftJoint->GetChild());

  // This assumes that the largest dimension of the wheel is the diameter
  // this->wheelRadius = 0.015587625;
}

/////////////////////////////////////////////////
void DiffDrivePlugin::OnVelMsg(ConstPosePtr &_msg)
{
  double vr, va;

  vr = _msg->position().x();
  va =  msgs::ConvertIgn(_msg->orientation()).Euler().Z();

  this->velocity = vr + va;
  // this->wheelSpeed[RIGHT] = vr - va * this->wheelSeparation / 2.0;
}

/////////////////////////////////////////////////
void DiffDrivePlugin::OnUpdate()
{
  /* double d1, d2;
  double dr, da;

  this->prevUpdateTime = currTime;

  // Distance travelled by front wheels
  d1 = stepTime.Double() * this->wheelRadius * this->leftJoint->GetVelocity(0);
  d2 = stepTime.Double() * this->wheelRadius * this->rightJoint->GetVelocity(0);

  dr = (d1 + d2) / 2;
  da = (d1 - d2) / this->wheelSeparation;
  common::Time currTime = this->model->GetWorld()->SimTime();
  common::Time stepTime = currTime - this->prevUpdateTime;
  */

  double leftVelDesired = (this->velocity / this->wheelRadius);
  // double rightVelDesired = (this->wheelSpeed[RIGHT] / this->wheelRadius);

  this->leftJoint->SetVelocity(0, leftVelDesired);
  // this->rightJoint->SetVelocity(0, rightVelDesired);
}
