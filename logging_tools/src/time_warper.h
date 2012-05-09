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
 * Encapsulates logic for playing back messages at a different rate than
 * they were recorded
 *
 * \author Bhaskara Marthi
 */

#ifndef LOGGING_TOOLS_TIME_WARPER_H
#define LOGGING_TOOLS_TIME_WARPER_H

#include <boost/optional.hpp>
#include <cmath>

namespace logging_tools
{

using boost::optional;


// We're going to be playing back messages received between t1 and t2, at
// some rate; we're also going to be skipping messages to ensure that we
// publish at most one message every inc seconds.  This class wraps all of
// that logic and related state.
class TimeWarper
{
public:
  TimeWarper (double playback_start_time, double earliest_receipt_time,
              double latest_receipt_time, double rate, double inc) :
    playback_start_time_(playback_start_time),
    earliest_receipt_time_(earliest_receipt_time),
    latest_receipt_time_(latest_receipt_time), rate_(rate), inc_(inc),
    last_ind_(-1)
  {}

  // Given a new message with timestamp t, and given current time,
  // return how long, in seconds, to wait before publishing the message,
  // or an empty value if the message should be skipped, either because
  // it's in the same time window as an already published message, or because
  // it's too late to publish this one.
  optional<double> waitTime (double t, double now)
  {
    const int ind = int(floor((t-earliest_receipt_time_)/(rate_*inc_)));
    if (ind>last_ind_)
    {
      last_ind_ = ind;
      const double publish_at = playback_start_time_+ind*inc_;
      if (publish_at > now)
        return publish_at-now;
    }
    return optional<double>();
  }

private:
  const double playback_start_time_;
  const double earliest_receipt_time_;
  const double latest_receipt_time_;
  const double rate_;
  const double inc_;
  int last_ind_;
};

} // namespace

#endif // include guard
