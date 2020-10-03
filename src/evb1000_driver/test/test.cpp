#include <iostream>
#include <string>
#include <sstream>
#include <vector>
#include <stdint.h>
#include <gtest/gtest.h>
#include <ros/ros.h>

TEST (Class12, test12)
{
  EXPECT_TRUE(true);
}


int main ( int argc, char ** argv )
{
  try
  {
    ::testing::InitGoogleTest( &argc, argv );
    return RUN_ALL_TESTS( );
  }
  catch (std::exception &e)
  {
    std::cerr << "Unhandled Exception: " << e.what( ) << std::endl;
  }

  return 1;
}
