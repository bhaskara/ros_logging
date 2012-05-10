/*
 * Copyright (c) 2008, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 */

/**
 * \file 
 * 
 * Test viewport stuff
 *
 * \author Bhaskara Marthi
 */

#include <roslog_gui/viewport.h>
#include <gtest/gtest.h>

typedef std::vector<int> IntVec;
using std::cerr;
using std::endl;
using roslog_gui::Viewport;
using roslog_gui::Rows;
using std::ostream;

void verifyState (const Viewport<int>& vp, size_t expected_start,
                  size_t expected_size, size_t expected_begin,
                  size_t expected_end)
{
  bool same = true;
  size_t expected_num_items = expected_end - expected_begin;
  if (vp.getStart()!=expected_start || vp.getSize()!=expected_size)
    same = false;
  else if (expected_num_items!=vp.getItems().size())
    same = false;
  else
  {
    for (size_t i=0; i<expected_num_items; i++)
    {
      if ((size_t)vp.getItems().at(i)!=(expected_begin+i)*(expected_begin+i))
      {
        same = false;
        break;
      }
    }
  }
  if (!same)
  {
    cerr << "State did not match expected values start=" << expected_start <<
      ", size=" << expected_size << ", begin=" << expected_begin << ", end="
         << expected_end << endl;
    for (size_t i=0; i<vp.getItems().size(); i++)
    {
      cerr << "  " << i << ": " << vp.getItems()[i] << endl;
    }
  }
  
  // For debugging
  else
  {
    cerr << "As expected, vp has start=" << expected_start << ", size=" <<
      expected_size << ", begin=" << expected_begin << ", end=" <<
      expected_end << endl;
  }
  EXPECT_TRUE(same);
}

IntVec squares(size_t a, size_t b)
{
  IntVec v(b-a);
  for (size_t i=0; i<b-a; i++)
    v[i] = (a+i)*(a+i);
  return v;
}

template <typename T>
ostream& operator<< (ostream& str, std::pair<T, T> p)
{
  str << "(" << p.first << ", " << p.second << ")";
  return str;
}

namespace roslog_gui
{

double getStamp (const int x)
{
  return double(x*10);
}



}

// We're going to have a viewport where the underlying set is the squares of
// integers, at any point the item set is a subrange of this, and the viewable
// set is a subrange of the item set
TEST(TestGui, TestViewport)
{
  // Create empty viewport
  Viewport<int> vp(IntVec(), 10);
  verifyState(vp, 0, 10, 0, 0);
  
  // Scrolling the empty viewport never changes it
  EXPECT_EQ(Rows(0, 10), vp.scroll(3));
  verifyState(vp, 0, 10, 0, 0);
  EXPECT_EQ(Rows(0, 10), vp.scroll(-5));
  verifyState(vp, 0, 10, 0, 0);
  EXPECT_EQ(Rows(0, 10), vp.scroll(0));
  verifyState(vp, 0, 10, 0, 0);

  // Resizing just changes the size without redisplays
  EXPECT_EQ(Rows(0, 20), vp.resize(20));
  verifyState(vp, 0, 20, 0, 0);

  // Adding items to empty viewport in various ways
  EXPECT_EQ(Rows(0, 20), vp.postpend(squares(30, 60)));
  verifyState(vp, 0, 20, 30, 60);
  vp = Viewport<int>(IntVec(), 10);
  EXPECT_EQ(Rows(0, 0), vp.prepend(squares(20, 40)));
  verifyState(vp, 0, 10, 20, 40);
  vp = Viewport<int>(IntVec(), 30);
  EXPECT_EQ(Rows(0, 30), vp.updateItems(squares(15, 35)));
  verifyState(vp, 0, 30, 15, 35);

  // Since we're looking past the end, adding new items to the end
  // should scroll forward once we reach the end of the screen
  EXPECT_EQ(Rows(20, 25), vp.postpend(squares(35, 40)));
  verifyState(vp, 0, 30, 15, 40);
  EXPECT_EQ(Rows(0, 30), vp.postpend(squares(40, 60)));
  verifyState(vp, 15, 30, 15, 60);

  // Remove from beginning shouldn't assert
  vp.removeFromBeginning(5);
  verifyState(vp, 10, 30, 20, 60);
  
}


int main (int argc, char** argv)
{
  testing::InitGoogleTest (&argc, argv);
  return RUN_ALL_TESTS();
}
