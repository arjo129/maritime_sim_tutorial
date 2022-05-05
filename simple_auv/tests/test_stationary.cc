/*
 * Copyright (C) 2021 Open Source Robotics Foundation
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

#include "gtest/gtest.h"

#include <ignition/common/Console.hh>
#include <ignition/gazebo/Model.hh>
#include <ignition/gazebo/World.hh>
#include <ignition/gazebo/Server.hh>
#include <ignition/gazebo/Util.hh>
#include <ignition/gazebo/TestFixture.hh>

#include "TestConfig.hh"

TEST(simple_auv_test, test_stationary) {
  // Maximum verbosity helps with debugging
  ignition::common::Console::SetVerbosity(4);

  // Instantiate test fixture. It starts a server and provides hooks that we'll
  // use to inspect the running simulation.
  ignition::gazebo::TestFixture fixture(ignition::common::joinPaths(
    PROJECT_SOURCE_PATH, "worlds", "water_world.sdf"));

  std::size_t iterations{0};
  ignition::gazebo::Entity modelEntity;

  fixture.
  OnConfigure([&modelEntity](const ignition::gazebo::Entity &_worldEntity,
      const std::shared_ptr<const sdf::Element> &/*_sdf*/,
      ignition::gazebo::EntityComponentManager &_ecm,
      ignition::gazebo::EventManager &/*_eventMgr*/)
  {
    ignition::gazebo::World world(_worldEntity);

    // Get entity
    modelEntity = world.ModelByName(_ecm, "tethys");
    EXPECT_NE(ignition::gazebo::kNullEntity, modelEntity);
  }).
  OnPostUpdate(
  [&iterations, &modelEntity](
    const ignition::gazebo::UpdateInfo &_info,
    const ignition::gazebo::EntityComponentManager &_ecm)
  {
    // Get the pose of the model
    auto pose = ignition::gazebo::worldPose(modelEntity, _ecm);

    // Make sure its still at the same place
    EXPECT_NEAR(pose.Pos().X(), 0.0, 1e-6);
    EXPECT_NEAR(pose.Pos().Y(), 0.0, 1e-6);
    EXPECT_NEAR(pose.Pos().Z(), 0.0, 1e-6);

    // Make sure its still at the same orientation
    EXPECT_NEAR(pose.Rot().Euler().X(), 0.0, 1e-6);
    EXPECT_NEAR(pose.Rot().Euler().Y(), 0.0, 1e-6);
    EXPECT_NEAR(pose.Rot().Euler().Z(), 0.0, 1e-6);

    // Make sure we're running the test.
    iterations++;
  }).Finalize();

  // Setup simulation server, this will call the post-update callbacks.
  // It also calls pre-update and update callbacks if those are being used.
  fixture.Server()->Run(true, 1000, false);

  // Verify that the post update function was called 1000 times
  EXPECT_EQ(1000, iterations);

}