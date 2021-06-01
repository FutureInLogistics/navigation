#include <gtest/gtest.h>

#include <base_local_planner/velocity_iterator_exp.h>


namespace base_local_planner {

TEST(VelocityIteratorExpTest, testsingle) {
  double result[5];
  int i = 0;
  for(base_local_planner::VelocityIteratorExp x_it(0.0, 0.0, 1, 0, 1); !x_it.isFinished(); x_it++) {
    result[i] = x_it.getVelocity();
    i++;
  }
  EXPECT_EQ(1, i);
  EXPECT_EQ(0, result[0]);
}

TEST(VelocityIteratorExpTest, testsingle_pos) {
  double result[5];
  int i = 0;
  for(base_local_planner::VelocityIteratorExp x_it(2.2, 2.2, 1, 0, 1); !x_it.isFinished(); x_it++) {
    result[i] = x_it.getVelocity();
    i++;
  }
  EXPECT_EQ(1, i);
  EXPECT_EQ(2.2, result[0]);
}

TEST(VelocityIteratorExpTest, testsingle_neg) {
  double result[5];
  int i = 0;
  for(base_local_planner::VelocityIteratorExp x_it(-3.3, -3.3, 1, 0, 1); !x_it.isFinished(); x_it++) {
    result[i] = x_it.getVelocity();
    i++;
  }
  EXPECT_EQ(1, i);
  EXPECT_EQ(-3.3, result[0]);
}

TEST(VelocityIteratorExpTest, test1) {
  double result[5];
  int i = 0;
  for(base_local_planner::VelocityIteratorExp x_it(-30, 30, 1, 0, 1); !x_it.isFinished(); x_it++) {
    result[i] = x_it.getVelocity();
    i++;
  }
  EXPECT_EQ(3, i);
  double expected [3]= {-30.0, 0.0, 30.0};
  for (int j = 0; j < 3; ++j) {
    EXPECT_EQ(expected[j], result[j]);
  }
}

TEST(VelocityIteratorExpTest, test1_squared) {
  double result[5];
  int i = 0;
  for(base_local_planner::VelocityIteratorExp x_it(0, -30, 30, 1, 0, 2); !x_it.isFinished(); x_it++) {
    result[i] = x_it.getVelocity();
    i++;
  }
  EXPECT_EQ(3, i);
  double expected [3]= {-30.0, 0.0, 30.0};
  for (int j = 0; j < 3; ++j) {
    EXPECT_EQ(expected[j], result[j]);
  }
}

TEST(VelocityIteratorExpTest, test1_pos) {
  double result[5];
  int i = 0;
  for(base_local_planner::VelocityIteratorExp x_it(0, 10, 30, 1, 0, 1); !x_it.isFinished(); x_it++) {
    result[i] = x_it.getVelocity();
    i++;
  }
  EXPECT_EQ(2, i);
  double expected [2]= {10.0, 30.0};
  for (int j = 0; j < 2; ++j) {
    EXPECT_EQ(expected[j], result[j]);
  }
}

TEST(VelocityIteratorExpTest, test1_neg) {
  double result[5];
  int i = 0;
  for(base_local_planner::VelocityIteratorExp x_it(-10, -30, -10, 1, 0, 1); !x_it.isFinished(); x_it++) {
    result[i] = x_it.getVelocity();
    i++;
  }
  EXPECT_EQ(2, i);
  double expected [2]= {-30.0, -10.0};
  for (int j = 0; j < 2; ++j) {
    EXPECT_EQ(expected[j], result[j]);
  }
}

TEST(VelocityIteratorExpTest, test3) {
  double result[5];
  int i = 0;
  for(base_local_planner::VelocityIteratorExp x_it(-30, 30, 3, 0, 1); !x_it.isFinished(); x_it++) {
    result[i] = x_it.getVelocity();
    i++;
  }
  EXPECT_EQ(3, i);
  double expected [3]= {-30.0, 0.0, 30};
  for (int j = 0; j < 3; ++j) {
    EXPECT_EQ(expected[j], result[j]);
  }
}

TEST(VelocityIteratorExpTest, test3_minvel) {
  double result[8];
  int i = 0;
  for(base_local_planner::VelocityIteratorExp x_it(-30, 30, 3, 1, 1); !x_it.isFinished(); x_it++) {
    result[i] = x_it.getVelocity();
    i++;
  }
  EXPECT_EQ(5, i);
  double expected [5]= {-30.0, -1.0, 0.0, 1.0, 30};
  for (int j = 0; j < 3; ++j) {
    EXPECT_EQ(expected[j], result[j]);
  }
}

TEST(VelocityIteratorExpTest, test3_squared) {
  double result[5];
  int i = 0;
  for(base_local_planner::VelocityIteratorExp x_it(0, -30, 30, 3, 0, 2); !x_it.isFinished(); x_it++) {
    result[i] = x_it.getVelocity();
    i++;
  }
  EXPECT_EQ(3, i);
  double expected [3]= {-30.0, 0.0, 30};
  for (int j = 0; j < 3; ++j) {
    EXPECT_EQ(expected[j], result[j]);
  }
}

TEST(VelocityIteratorExpTest, test4) {
  double result[5];
  int i = 0;
  for(base_local_planner::VelocityIteratorExp x_it(-30, 30, 4, 0, 1); !x_it.isFinished(); x_it++) {
    result[i] = x_it.getVelocity();
    i++;
  }
  EXPECT_EQ(5, i);
  double expected [5]= {-30.0, -10.0, 0.0, 10.0, 30};
  for (int j = 0; j < 5; ++j) {
    EXPECT_EQ(expected[j], result[j]);
  }
}

TEST(VelocityIteratorExpTest, test4_squared) {
  double result[5];
  int i = 0;
  for(base_local_planner::VelocityIteratorExp x_it(0, -45, 45, 4, 0, 2); !x_it.isFinished(); x_it++) {
    result[i] = x_it.getVelocity();
    i++;
  }
  EXPECT_EQ(5, i);
  double expected [5]= {-45.0, -5, 0.0, 5, 45};
  for (int j = 0; j < 5; ++j) {
    EXPECT_EQ(expected[j], result[j]);
  }
}

TEST(VelocityIteratorExpTest, test10_minvel) {
  double result[11];
  int i = 0;
  for(base_local_planner::VelocityIteratorExp x_it(0, -10, 10, 5, 9, 1); !x_it.isFinished(); x_it++) {
    result[i] = x_it.getVelocity();
    i++;
  }
  EXPECT_EQ(7, i);
  double expected [7]= {-10.0, -9.5, -9.0, 0.0, 9.0, 9.5, 10.0};
  for (int j = 0; j < 5; ++j) {
    EXPECT_EQ(expected[j], result[j]);
  }
}

TEST(VelocityIteratorExpTest, test_shifted) {
  // test where zero is not in the middle
  double result[6];
  int i = 0;
  for(base_local_planner::VelocityIteratorExp x_it(0, -10, 50, 4, 0, 1); !x_it.isFinished(); x_it++) {
    result[i] = x_it.getVelocity();
    i++;
  }
  EXPECT_EQ(5, i);
  double expected [5]= {-10.0, 0.0, 10.0, 30, 50};
  for (int j = 0; j < 5; ++j) {
    EXPECT_EQ(expected[j], result[j]);
  }
}

TEST(VelocityIteratorExpTest, test5_frac_exp) {
  // test where zero is not in the middle
  double result[6];
  int i = 0;
  double lim = 2.8284271247462;
  for(base_local_planner::VelocityIteratorExp x_it(0, -lim, lim, 5, 0, 1.5); !x_it.isFinished(); x_it++) {
    result[i] = x_it.getVelocity();
    i++;
  }
  EXPECT_EQ(5, i);
  double expected [5]= {-lim, -1, 0.0, 1, lim};
  for (int j = 0; j < 5; ++j) {
    EXPECT_NEAR(expected[j], result[j], 1e-9);
  }
}

TEST(VelocityIteratorExpTest, test_cranky) {
  // test where one value is almost zero (nothing to do about that)
  double result[5];
  int i = 0;
  for(base_local_planner::VelocityIteratorExp x_it(-10.00001, 10, 3, 0, 1); !x_it.isFinished(); x_it++) {
    result[i] = x_it.getVelocity();
    i++;
  }
  EXPECT_EQ(4, i);
  for (int j = 0; j < 5; ++j) {
	double expected [5]= {-10.00001, -0.000005, 0.0, 10.0};
    EXPECT_FLOAT_EQ(expected[j], result[j]);
  }
}



} // namespace
