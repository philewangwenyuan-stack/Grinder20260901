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
TEST(LineCorner, SpeedCapAndMinimumAreBounded) {
  EXPECT_DOUBLE_EQ(.03,approachSpeed(.03,1,.35,.01));
  EXPECT_DOUBLE_EQ(.01,approachSpeed(.03,.06,.35,.01));
  EXPECT_LE(approachSpeed(.005,.06,.35,.01),.005);
}
int main(int argc,char** argv) { testing::InitGoogleTest(&argc,argv); return RUN_ALL_TESTS(); }
