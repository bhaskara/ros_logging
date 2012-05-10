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
 * Implementation of viewport.h
 *
 * \author Bhaskara Marthi
 */

#ifndef ROSLOG_GUI_VIEWPORT_IMPL_H
#define ROSLOG_GUI_VIEWPORT_IMPL_H

#include <iostream>
#include <boost/format.hpp>
#include <stdexcept>

namespace roslog_gui
{

using boost::format;
using std::range_error;

template <typename Item>
Viewport<Item>::Viewport (const ItemVec& items, const size_t size,
                          const size_t start) :
  items_(items), start_(start), size_(size)
{
  verifyInvariants();
}

template <typename Item>
void Viewport<Item>::verifyInvariants()
{
  if (size_==0)
    throw range_error("Size was zero");
  if (!(start_<items_.size() || (start_==0 && items_.empty())))
  {
    format e("Illegal values %1% and %2% for start and num items");
    throw range_error((e % start_ % items_.size()).str());
  }
}

template <typename Item>
boost::optional<Item> Viewport<Item>::itemOnRow (const size_t r)
{
  if (r>=size_)
  {
    format e("Can't get item on row %1% since size is %2%");
    throw range_error((e % r % size_).str());
  }
  const size_t n = r + start_;
  boost::optional<Item> item;
  if (n<items_.size())
    item = items_.at(n);
  return item;
}

template <typename Item>
Rows Viewport<Item>::scroll (const int num_rows)
{
  if (num_rows < 0)
  {
    const size_t nr = -num_rows;
    if (nr>start_)
      start_ = 0;
    else
      start_ -= nr;
  }
  else if (items_.size()>0)
  {
    const size_t last = items_.size()-1;
    start_ = std::min(start_+num_rows, last);
  }
  verifyInvariants();
  return Rows(0, size_);
}

template <typename Item>
Rows Viewport<Item>::postpend (const ItemVec& new_items)
{
  const size_t prev_size = items_.size();
  const size_t last_visible = start_ + size_ - 1;
  items_.insert(items_.end(), new_items.begin(), new_items.end());
  const size_t new_size = items_.size();
  
  if (prev_size == 0)
    return Rows(0, size_);
  
  // If we can currently see the last item
  else if (prev_size-1 <= last_visible)
  {
    // Special case logic to deal with case when we're at the end and 
    // need to scroll up as new items come in
    if (new_size-1 > last_visible)
    {
      start_ = new_size - size_;
      verifyInvariants();
      return Rows(0, size_);
    }
    else
    {
      return Rows(prev_size-start_, new_size-start_);
    }
  }
  // If nothing changes in visible region
  else
    return Rows(0, 0);
}

// Just insert new items; visible region guaranteed not to change
template <typename Item>
Rows Viewport<Item>::prepend (const ItemVec& new_items)
{
  ItemVec items = new_items;
  items.insert(items.end(), items_.begin(), items_.end());
  items_.swap(items);
  verifyInvariants();
  return Rows(0, 0);
}

// Change size and redisplay everything
template <typename Item>
Rows Viewport<Item>::resize (const size_t new_size)
{
  if (new_size==0)
    throw range_error("New size must be positive");
  size_ = new_size;
  verifyInvariants();
  return Rows(0, size_);
}

// Update the item set, then figure out where within the new set our
// viewport should be; we do this by getting a real-valued 'stamp'
// from the current first item, then looking for that stamp among
// the new items
template <typename Item>
Rows Viewport<Item>::updateItems (const ItemVec& new_items)
{
  const double stamp = (items_.size()>0) ? getStamp(items_[0]) : 0.0;
  size_t i=0;
  for (; i<new_items.size() && getStamp(new_items[i])<stamp; i++);
  if (new_items.empty())
    start_=0;
  else if (i==new_items.size())
    start_=i-1;
  else
    start_=i;
  items_ = new_items;
  verifyInvariants();
  return Rows(0, size_);
}

// Check that none of the removed items are visible, then remove them 
template <typename Item>
void Viewport<Item>::removeFromEnd (const size_t n)
{
  const size_t m = items_.size();
  if (m<start_+size_+n)
  {
    format e("Can't remove %1% items from end given start %2%"
             ", size %3%, and %4% items");
    throw range_error((e % n % start_ % size_ % m).str());
  }
  items_.erase(items_.begin()+m-n, items_.end());
  verifyInvariants();
}

// Check that removed items aren't visible, remove them and update
template <typename Item>
void Viewport<Item>::removeFromBeginning (const size_t n)
{
  if (n>=items_.size() || n>=start_)
  {
    format e("Can't remove %1% items from end given start %2%, %3% items");
    throw range_error((e % n % start_ % items_.size()).str());
  }
  ItemVec new_items(items_.begin()+n, items_.end());
  items_.swap(new_items);
  start_ -= n;
  verifyInvariants();
}


} // namespace

#endif // include guard
