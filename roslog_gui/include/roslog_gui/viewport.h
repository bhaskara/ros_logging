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
 * Defines viewport class
 *
 * \author Bhaskara Marthi
 */

#ifndef ROSLOG_GUI_VIEWPORT_H
#define ROSLOG_GUI_VIEWPORT_H

#include <boost/optional.hpp>
#include <vector>
#include <ostream>

namespace roslog_gui
{

typedef std::pair<size_t, size_t> Rows;

// Viewport maintains state of a window that is displaying a set of rows
// from some larger set of items.  You can do various operations to
// scroll the viewport and change the underlying item set.  All of them
// return the range of rows (relative to the viewport) that must now be
// redisplayed.
template <typename Item>
class Viewport
{
public:

  typedef std::vector<Item> ItemVec;

  // Constructor
  // size must be positive, and start < num items
  Viewport (const ItemVec& items, const size_t size, const size_t start=0);

  // Add some new items to the end of the set
  Rows postpend (const ItemVec& new_items);

  // Add some new items at the beginning
  Rows prepend (const ItemVec& new_items);
  
  // Remove items at end
  void removeFromEnd (size_t n);

  // Remove items at beginning
  void removeFromBeginning (size_t n);

  // Update item set
  Rows updateItems (const ItemVec& new_items);
  
  // Scroll the viewport by n lines (negative means scroll down)
  Rows scroll (int n);
  
  // Resize the viewport (new size must be positive)
  Rows resize (size_t new_size);
  
  // Get item displayed on row I, or empty if there's no item
  boost::optional<Item> itemOnRow (size_t r);

  // Accessors (for white-box testing)
  const ItemVec& getItems() const { return items_; }
  size_t getStart() const { return start_; }
  size_t getSize() const { return size_; }
  
  
private:
  
  // Debugging method to check consistency
  void verifyInvariants();

  // Current item set
  ItemVec items_;
  
  // The first item that is being displayed, so between 0 and items_.size()-1,
  // except can be equal to items.size() if items is empty
  size_t start_;
  
  // The number of items being displayed
  size_t size_;
};

} // namespace

#include "impl/viewport_impl.h"

#endif // include guard
