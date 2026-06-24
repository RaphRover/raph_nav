// Copyright (c) 2020, Samsung Research America
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef NAV2_CONTROLLER__PATH_UTILS_HPP_
#define NAV2_CONTROLLER__PATH_UTILS_HPP_

#include <cmath>
#include <optional>
#include <vector>

#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

namespace nav2_controller
{

/**
 * @brief Computes the dot product of two 2-D direction vectors derived from
 *        consecutive path poses.
 *
 * Each direction vector points from pose[i] to pose[i+1] and is projected
 * onto the heading of pose[i] (cos(yaw), sin(yaw)).  A negative dot product
 * means the robot would have to reverse – i.e. a Reeds-Shepp cusp.
 *
 * @param p0   The pose before the candidate cusp.
 * @param p1   The pose at the candidate cusp (transition point).
 * @param p2   The pose after the candidate cusp.
 * @return     The dot product of the two successive motion direction vectors.
 *             Negative ⟹ direction reversal (cusp).
 */
inline double computeDirectionDot(
  const geometry_msgs::msg::PoseStamped & p0,
  const geometry_msgs::msg::PoseStamped & p1,
  const geometry_msgs::msg::PoseStamped & p2)
{
  // Vector from p0 → p1
  const double dx01 = p1.pose.position.x - p0.pose.position.x;
  const double dy01 = p1.pose.position.y - p0.pose.position.y;

  // Vector from p1 → p2
  const double dx12 = p2.pose.position.x - p1.pose.position.x;
  const double dy12 = p2.pose.position.y - p1.pose.position.y;

  // If either segment has (near-)zero length the poses are on top of each
  // other; treat as no reversal to avoid false positives from numerical noise.
  const double len01 = std::hypot(dx01, dy01);
  const double len12 = std::hypot(dx12, dy12);
  if (len01 < 1e-6 || len12 < 1e-6) {
    return 1.0;  // positive → no cusp
  }

  // Dot product of the two unit direction vectors.
  return (dx01 * dx12 + dy01 * dy12) / (len01 * len12);
}

/**
 * @brief Accumulates the Euclidean arc length between consecutive poses
 *        in the range [begin, end).
 *
 * @param begin  Iterator to the first pose.
 * @param end    Past-the-end iterator.
 * @return       Arc length in metres.
 */
inline double computeSegmentLength(
  std::vector<geometry_msgs::msg::PoseStamped>::const_iterator begin,
  std::vector<geometry_msgs::msg::PoseStamped>::const_iterator end)
{
  double length = 0.0;
  for (auto it = begin; std::next(it) != end; ++it) {
    const double dx = std::next(it)->pose.position.x - it->pose.position.x;
    const double dy = std::next(it)->pose.position.y - it->pose.position.y;
    length += std::hypot(dx, dy);
  }
  return length;
}

/**
 * @brief Splits a path into contiguous segments separated by Reeds-Shepp
 *        cusps (direction reversals).
 *
 * Each element of the returned vector is a sub-path whose poses are a
 * contiguous slice of the input.  The cusp pose itself is included as the
 * *last* pose of the segment that ends there, so the robot stops precisely
 * at the direction-change point.  The same pose is NOT repeated as the first
 * pose of the next segment — the next segment starts one pose after the cusp.
 *
 * If the path has no cusps a single-element vector containing the full path
 * is returned.  If the path is too short to detect any cusp (< 3 poses) the
 * full path is likewise returned as a single segment.
 *
 * @param full_path  The complete path to split.
 * @return           Ordered vector of path segments.
 */
inline std::vector<nav_msgs::msg::Path> splitPathAtCusps(
  const nav_msgs::msg::Path & full_path)
{
  std::vector<nav_msgs::msg::Path> segments;

  if (full_path.poses.size() < 3) {
    segments.push_back(full_path);
    return segments;
  }

  size_t segment_start = 0;  // index of the first pose of the current segment

  for (size_t i = 0; i + 2 < full_path.poses.size(); ++i) {
    const double dot = computeDirectionDot(
      full_path.poses[i],
      full_path.poses[i + 1],
      full_path.poses[i + 2]);

    if (dot < 0.0) {
      // pose[i+1] is the cusp — end the current segment here (inclusive).
      nav_msgs::msg::Path seg;
      seg.header = full_path.header;
      seg.poses.assign(
        full_path.poses.begin() + segment_start,
        full_path.poses.begin() + i + 2);  // +2 to include pose[i+1]
      segments.push_back(std::move(seg));

      // Next segment starts immediately after the cusp pose.
      segment_start = i + 1;
    }
  }

  // Always append the tail segment (from segment_start to the end).
  // This also handles the no-cusp case where segment_start == 0.
  nav_msgs::msg::Path tail;
  tail.header = full_path.header;
  tail.poses.assign(
    full_path.poses.begin() + segment_start,
    full_path.poses.end());
  segments.push_back(std::move(tail));

  return segments;
}

/**
 * @brief Returns the first path segment whose arc length meets or exceeds
 *        @p min_segment_length, or — if no such segment exists — the first
 *        segment of the path.
 *
 * This is the primary entry point for cusp-aware path truncation.
 *
 * Rationale
 * ---------
 * The Reeds-Shepp planner occasionally inserts very short reverse segments
 * (micro-cusps) that cause the RPP controller's look-ahead carrot to oscillate.
 * Simply returning the first segment would hand the controller a trivially
 * short path.  Instead, we scan forward through all segments and return the
 * first one that is long enough to be worth following.  The only exception is
 * when *every* segment is shorter than the threshold — then the first segment
 * is returned so navigation still makes progress rather than stalling.
 *
 * Behaviour summary
 * -----------------
 * | Segments found | First viable (>= min_len) | Returned segment        |
 * |----------------|---------------------------|-------------------------|
 * | 1              | N/A                       | that segment (full path)|
 * | N              | segment k                 | segment k               |
 * | N              | none (all short)          | segment 0               |
 *
 * @param full_path          Complete path from the planner.
 * @param min_segment_length Minimum arc length in metres a segment must have
 *                           before it is considered viable.  Segments shorter
 *                           than this are treated as micro-cusps and skipped,
 *                           unless they are the only option.
 *                           Recommended starting value: 2× minimum_turning_radius.
 * @return                   A single nav_msgs::msg::Path representing the
 *                           segment the controller should follow next.
 */
inline nav_msgs::msg::Path firstViableSegment(
  const nav_msgs::msg::Path & full_path,
  double min_segment_length = 0.1)
{
  const std::vector<nav_msgs::msg::Path> segments = splitPathAtCusps(full_path);

  // Single segment (no cusps, or path too short to split) — return as-is.
  if (segments.size() == 1) {
    return segments.front();
  }

  // Walk segments in order; return the first one that meets the length threshold.
  for (const auto & seg : segments) {
    if (seg.poses.size() < 2) {
      continue;  // degenerate single-pose segment — always skip
    }
    const double len = computeSegmentLength(seg.poses.begin(), seg.poses.end());
    if (len >= min_segment_length) {
      return seg;
    }
  }

  // Every segment was below the threshold — return the first one so the robot
  // still makes progress (avoids an infinite replanning loop).
  return segments.front();
}

}  // namespace nav2_controller

#endif  // NAV2_CONTROLLER__PATH_UTILS_HPP_