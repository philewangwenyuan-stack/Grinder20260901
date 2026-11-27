#include <gtest/gtest.h>
#include "regulated_pure_pursuit_controller/line_corner.hpp"
using namespace regulated_pure_pursuit_controller::line_corner;
TEST(LineCorner, DuplicatesDensificationAndFinalEndpoint) {
  Route r; r.set({{0,0},{0,0},{1,0},{2,0},{2,1},{2,2}},0.45);
  auto c=r.current({0,0}); ASSERT_NE(nullptr,c);
  EXPECT_DOUBLE_EQ(2,c->point.x); EXPECT_DOUBLE_EQ(0,c->point.y);
  r.complete(); EXPECT_EQ(nullptr,r.current({2,1}));
}
TEST(LineCorner, ProjectionTargetDoesNotShrinkOrFollowOutgoingEdge) {
  Route r; r.set({{0,0},{2,0},{2,2}},0.45); auto c=r.current({1.9,0.1});
  auto target=r.carrot(*c,{1.9,0.1},0.55);
  EXPECT_NEAR(2.45,target.x,1e-12); EXPECT_DOUBLE_EQ(0,target.y);
  EXPECT_NEAR(0.1,r.remaining(*c,{1.9,0.1}),1e-12);
  EXPECT_NEAR(-0.1,r.remaining(*c,{2.1,-0.3}),1e-12);
}
TEST(LineCorner, RotatedGeometryAndFortyFiveDegreeCorner) {
  Route r; r.set({{0,0},{1,1},{1,2}},0.45); auto c=r.current({0.9,0.8});
  ASSERT_NE(nullptr,c); EXPECT_NEAR(0.3/std::sqrt(2.0),r.remaining(*c,{0.9,0.8}),1e-12);
  auto target=r.carrot(*c,{0.9,0.8},0.55); EXPECT_NEAR(target.x,target.y,1e-12);
}
TEST(LineCorner, ResumeUsesCurrentSegmentNotPreviouslyPrunedIndex) {
  Route r; r.set({{0,0},{2,0},{2,2},{4,2}},0.45);
  ASSERT_NE(nullptr,r.current({0,0}));
  r.reacquire(); auto c=r.current({2,1}); ASSERT_NE(nullptr,c);
  EXPECT_DOUBLE_EQ(2,c->point.y);
}
TEST(LineCorner, DistinctSamplesAndDurationAreBothRequired) {
  StopEvidence e;
  EXPECT_FALSE(e.update(10,10,true,.25));
  EXPECT_FALSE(e.update(10.3,10,true,.25));
  EXPECT_TRUE(e.update(10.3,10.3,true,.25));
  EXPECT_FALSE(e.update(10.4,10.4,false,.25));
  EXPECT_FALSE(e.update(10.5,10.5,true,.25));
  EXPECT_FALSE(e.update(10.6,10.6,true,.25));
  EXPECT_TRUE(e.update(10.8,10.8,true,.25));
}
TEST(LineCorner, FreshFiniteFeedbackOnly) {
  EXPECT_TRUE(fresh(10,9.5,.6,0,0,0));
  EXPECT_FALSE(fresh(10,9,.6,0,0,0));
  EXPECT_FALSE(fresh(10,10.01,.6,0,0,0));
  EXPECT_FALSE(fresh(10,0,.6,0,0,0));
  EXPECT_FALSE(fresh(10,10,.6,NAN,0,0));
  EXPECT_FALSE(fresh(10,10,.6,0,INFINITY,0));
}
TEST(LineCorner, DepartureRampsPastTheFirstCommandWithZeroFeedback) {
  DepartureRamp ramp;
  constexpr double step = 0.02;
  EXPECT_FALSE(ramp.useCommandReference(10.0, 0.0, 0.0, step, .01));
  ramp.start(10.0);
  EXPECT_TRUE(ramp.useCommandReference(10.05, 0.0, 0.0, step, .01));
  EXPECT_TRUE(ramp.useCommandReference(10.10, 0.0, .02, step, .01));
  EXPECT_TRUE(ramp.useCommandReference(10.15, .012, .04, step, .01));
  EXPECT_FALSE(ramp.useCommandReference(10.20, .041, .06, step, .01));
  EXPECT_FALSE(ramp.useCommandReference(10.25, 0.0, .06, step, .01));
}
TEST(LineCorner, DepartureExpiresAndCanBeCancelledBeforeMotion) {
  DepartureRamp ramp;
  ramp.start(10.0);
  EXPECT_TRUE(ramp.useCommandReference(10.49, 0.0, .06, .02, .01));
  EXPECT_FALSE(ramp.useCommandReference(10.50, 0.0, .06, .02, .01));
  ramp.start(11.0);
  ramp.reset();
  EXPECT_FALSE(ramp.useCommandReference(11.05, 0.0, .02, .02, .01));
  ramp.start(12.0);
  EXPECT_FALSE(ramp.useCommandReference(11.95, 0.0, .02, .02, .01));
}
TEST(LineCorner, SpeedCapAndMinimumAreBounded) {
  EXPECT_DOUBLE_EQ(.03,approachSpeed(.03,1,.35,.01));
  EXPECT_DOUBLE_EQ(.01,approachSpeed(.03,.06,.35,.01));
  EXPECT_LE(approachSpeed(.005,.06,.35,.01),.005);
}
TEST(LineCorner, HeadingBrakeAccountsForFieldDecelerationAndDelay) {
  const double deceleration = 0.03;
  const double delay = 0.20;
  const double capture = 0.08;
  // The recorded overshoot began at 0.12 rad/s. The new profile limits speed
  // well before capture, then leaves enough angle for braking inside it.
  EXPECT_LT(headingSpeedLimit(0.22 - 0.5 * capture, deceleration, delay), 0.12);
  const double at_capture = headingSpeedLimit(0.5 * capture, deceleration, delay);
  EXPECT_LT(at_capture, 0.05);
  EXPECT_GT(at_capture, 0.01);
  EXPECT_NEAR(0.5 * capture, at_capture * delay + at_capture * at_capture / (2 * deceleration), 1e-9);
  EXPECT_DOUBLE_EQ(0.0, headingSpeedLimit(0.0, deceleration, delay));
}
int main(int argc,char** argv) { testing::InitGoogleTest(&argc,argv); return RUN_ALL_TESTS(); }
