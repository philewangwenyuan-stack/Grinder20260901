#include <gtest/gtest.h>
#include <limits>
#include "regulated_pure_pursuit_controller/scan_classifier.hpp"
using namespace regulated_pure_pursuit_controller;
TEST(ScanClassifier, BodyIsNotAutomaticallyFiltered) {
  EXPECT_EQ(ScanZone::body, classifyScan(0,0,-.4,1.2,-.47,.47,.15,.1,{}));
  EXPECT_EQ(ScanZone::side, classifyScan(0,.5,-.4,1.2,-.47,.47,.15,.1,{}));
  EXPECT_EQ(ScanZone::front, classifyScan(1.3,0,-.4,1.2,-.47,.47,.15,.1,{}));
  EXPECT_EQ(ScanZone::outside, classifyScan(3,0,-.4,1.2,-.47,.47,.15,.1,{}));
}
TEST(ScanClassifier, ExplicitSelfPolygonKeepsBoundaryObstacle) {
  const std::vector<ScanXY> polygon{{-.1,-.1},{.1,-.1},{.1,.1},{-.1,.1}};
  EXPECT_EQ(ScanZone::self, classifyScan(0,0,-.4,1.2,-.47,.47,.15,.1,polygon));
  EXPECT_EQ(ScanZone::body, classifyScan(.1,0,-.4,1.2,-.47,.47,.15,.1,polygon));
  EXPECT_EQ(ScanZone::body, classifyScan(.101,0,-.4,1.2,-.47,.47,.15,.1,polygon));
  EXPECT_FALSE(strictlyInside({},0,0));
}

TEST(ScanClassifier, FootprintOnlyModeStillRunsClearTimer) {
  EXPECT_TRUE(scanCollisionEnabled(false,true));
  EXPECT_TRUE(scanCollisionEnabled(true,false));
  EXPECT_TRUE(scanCollisionEnabled(true,true));
  EXPECT_FALSE(scanCollisionEnabled(false,false));
}

TEST(ScanClassifier, ClearRequiresDistinctValidScansAndObstacleResetsConfirmation) {
  bool latched=true; std::uint64_t last=1; int confirm=0,clear=0;
  EXPECT_FALSE(advanceClearScan(latched,last,confirm,clear,2,0,10,2));
  EXPECT_FALSE(advanceClearScan(latched,last,confirm,clear,2,0,10,2));
  EXPECT_TRUE(latched);
  EXPECT_FALSE(advanceClearScan(latched,last,confirm,clear,3,10,10,2));
  EXPECT_EQ(0,clear);
  EXPECT_FALSE(advanceClearScan(latched,last,confirm,clear,4,0,10,2));
  EXPECT_TRUE(advanceClearScan(latched,last,confirm,clear,5,0,10,2));
  EXPECT_FALSE(latched);
}

TEST(ScanClassifier, OldFutureAndInvalidScansCannotSupplyClearConfirmations) {
  EXPECT_TRUE(freshScanAge(.1,.2,1.5));
  EXPECT_FALSE(freshScanAge(.1,2.,1.5));
  EXPECT_FALSE(freshScanAge(2.,.1,1.5));
  EXPECT_FALSE(freshScanAge(.1,-2.,1.5));
  EXPECT_FALSE(freshScanAge(.1,std::numeric_limits<double>::infinity(),1.5));
}

int main(int argc, char** argv) { ::testing::InitGoogleTest(&argc, argv); return RUN_ALL_TESTS(); }
