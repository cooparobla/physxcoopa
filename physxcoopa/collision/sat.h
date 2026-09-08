/**
 * @file sat.h
 * @brief Box-box contact generation via the Separating Axis Theorem: 6 face axes + 9
 *        edge-cross axes, then either Sutherland-Hodgman face clipping (face contacts, up to
 *        4 points) or a single closest-points-between-segments point (edge contacts).
 */

#ifndef PHYSXCOOPA_COLLISION_SAT_H
#define PHYSXCOOPA_COLLISION_SAT_H

#include <physxcoopa/geometry/obb.h>
#include <physxcoopa/geometry/capsule.h>
#include <physxcoopa/collision/clip.h>
#include <physxcoopa/collision/manifold.h>
#include <physxcoopa/util/math.h>

#include <glm/glm.hpp>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>

namespace coopa {
namespace physx {
namespace collision {

/**
 * @struct SatAxisResult
 * @brief The minimum-penetration separating axis found across all 15 candidates.
 *
 * `best_axis` encodes which axis won: 0-2 = A's local face axes, 3-5 = B's local face axes,
 * 6-14 = edge-cross axes (`6 + i*3 + j`, A's edge `i` crossed with B's edge `j`).
 */
struct SatAxisResult {
    float penetration = 0.0f;
    int best_axis = -1;
    glm::vec3 normal{0.0f, 0.0f, 1.0f}; /**< Points from A toward B. */
};

/**
 * @brief Runs the 15-axis SAT test between two OBBs.
 *
 * Edge-cross axes are given a small overlap penalty relative to face axes before comparing,
 * so a near-tie (common for axis-aligned or near-axis-aligned boxes) resolves to the more
 * stable face-face manifold rather than flickering between face and edge contacts frame to
 * frame. A near-degenerate edge-cross axis (parallel edges) is skipped as a candidate rather
 * than causing a false separation.
 *
 * @return False if any axis proves separation (no collision); true with `result` populated
 *         otherwise.
 */
inline bool sat_test_obb_obb(const geometry::OBB& a, const geometry::OBB& b, SatAxisResult& result) {
    glm::vec3 a_axes[3];
    a.axes(a_axes[0], a_axes[1], a_axes[2]);
    glm::vec3 b_axes[3];
    b.axes(b_axes[0], b_axes[1], b_axes[2]);
    glm::vec3 d = b.center - a.center;

    float best_score = std::numeric_limits<float>::max();
    float best_overlap = 0.0f;
    int best_axis = -1;
    glm::vec3 best_normal(0.0f, 0.0f, 1.0f);
    bool separated = false;

    auto test_axis = [&](const glm::vec3& axis_raw, int axis_id, float bias) {
        if (separated) return;
        float len2 = glm::dot(axis_raw, axis_raw);
        if (len2 < 1e-8f) return; // near-parallel edges: not a valid candidate, just skip it

        glm::vec3 axis = axis_raw / std::sqrt(len2);
        float ra = 0.0f, rb = 0.0f;
        for (int k = 0; k < 3; ++k) {
            ra += a.half_extents[k] * std::abs(glm::dot(axis, a_axes[k]));
            rb += b.half_extents[k] * std::abs(glm::dot(axis, b_axes[k]));
        }
        float dist = std::abs(glm::dot(d, axis));
        float overlap = ra + rb - dist;
        if (overlap < 0.0f) {
            separated = true;
            return;
        }
        float score = overlap + bias;
        if (score < best_score) {
            best_score = score;
            best_overlap = overlap;
            best_axis = axis_id;
            best_normal = glm::dot(d, axis) < 0.0f ? -axis : axis;
        }
    };

    constexpr float k_edge_bias = 0.01f;
    test_axis(a_axes[0], 0, 0.0f);
    test_axis(a_axes[1], 1, 0.0f);
    test_axis(a_axes[2], 2, 0.0f);
    test_axis(b_axes[0], 3, 0.0f);
    test_axis(b_axes[1], 4, 0.0f);
    test_axis(b_axes[2], 5, 0.0f);
    for (int i = 0; i < 3 && !separated; ++i) {
        for (int j = 0; j < 3 && !separated; ++j) {
            test_axis(glm::cross(a_axes[i], b_axes[j]), 6 + i * 3 + j, k_edge_bias);
        }
    }

    if (separated || best_axis < 0) return false;

    result.penetration = best_overlap;
    result.best_axis = best_axis;
    result.normal = best_normal;
    return true;
}

/**
 * @brief Generates a face-face or edge-edge contact manifold for two overlapping OBBs.
 *
 * Feature ids: face contacts pack `(reference_face << 8) | (incident_face << 4) |
 * clipped_point_index` (reference/incident face each 0-5: `axis*2 + (sign<0)`); edge
 * contacts use a disjoint range (`0x1000 | (edge_a << 4) | edge_b`) so the two schemes can
 * never collide. Both are stable across steps for a fixed pair of touching
 * faces/edges, which is what warm starting needs.
 *
 * @param a   First box.
 * @param b   Second box.
 * @param out Output manifold (`out.a`/`out.b`/material/friction are NOT set here -- the
 *            caller, collision::generate_contacts, fills those in).
 * @return True if `a` and `b` overlap and `out` now holds contact points.
 */
inline bool generate_box_box_contacts(const geometry::OBB& a, const geometry::OBB& b, ContactManifold& out) {
    SatAxisResult r;
    if (!sat_test_obb_obb(a, b, r)) return false;

    out = ContactManifold{};
    out.normal = r.normal;

    if (r.best_axis <= 5) {
        bool ref_is_a = r.best_axis <= 2;
        const geometry::OBB& ref = ref_is_a ? a : b;
        const geometry::OBB& inc = ref_is_a ? b : a;
        int ref_axis = ref_is_a ? r.best_axis : r.best_axis - 3;

        glm::vec3 ref_axes[3];
        ref.axes(ref_axes[0], ref_axes[1], ref_axes[2]);
        // `r.normal` always points A -> B. The reference face's outward normal must point
        // toward the OTHER box: toward B if the reference is A, toward A (i.e. -r.normal) if
        // the reference is B.
        glm::vec3 desired_outward = ref_is_a ? r.normal : -r.normal;
        float ref_sign = glm::dot(ref_axes[ref_axis], desired_outward) >= 0.0f ? 1.0f : -1.0f;

        glm::vec3 ref_corners[4], ref_normal;
        obb_face(ref, ref_axis, ref_sign, ref_corners, ref_normal);

        glm::vec3 inc_axes[3];
        inc.axes(inc_axes[0], inc_axes[1], inc_axes[2]);
        int inc_axis = 0;
        float inc_sign = 1.0f;
        float most_anti_parallel = std::numeric_limits<float>::max();
        for (int k = 0; k < 3; ++k) {
            for (float s : {1.0f, -1.0f}) {
                float dp = glm::dot(inc_axes[k] * s, ref_normal);
                if (dp < most_anti_parallel) {
                    most_anti_parallel = dp;
                    inc_axis = k;
                    inc_sign = s;
                }
            }
        }
        glm::vec3 inc_corners[4], inc_normal;
        obb_face(inc, inc_axis, inc_sign, inc_corners, inc_normal);

        ClipPlane planes[4];
        obb_side_planes(ref, ref_axis, planes);

        glm::vec3 poly_a[k_max_clip_vertices];
        glm::vec3 poly_b[k_max_clip_vertices];
        int count_a = 4;
        for (int i = 0; i < 4; ++i) poly_a[i] = inc_corners[i];
        for (int p = 0; p < 4 && count_a > 0; ++p) {
            int count_b = 0;
            clip_polygon(poly_a, count_a, planes[p], poly_b, count_b);
            for (int i = 0; i < count_b; ++i) poly_a[i] = poly_b[i];
            count_a = count_b;
        }
        if (count_a == 0) return false;

        float ref_d = glm::dot(ref_normal, ref_corners[0]);
        struct Candidate { glm::vec3 pos; float penetration; };
        Candidate candidates[k_max_clip_vertices];
        int kept = 0;
        for (int i = 0; i < count_a; ++i) {
            float signed_dist = glm::dot(ref_normal, poly_a[i]) - ref_d;
            float penetration = -signed_dist;
            if (penetration > -util::k_epsilon) {
                candidates[kept].pos = poly_a[i];
                candidates[kept].penetration = std::max(penetration, 0.0f);
                ++kept;
            }
        }
        if (kept == 0) return false;

        if (kept > 4) {
            std::sort(candidates, candidates + kept,
                      [](const Candidate& x, const Candidate& y) { return x.penetration > y.penetration; });
            kept = 4;
        }

        int ref_face_id = ref_axis * 2 + (ref_sign < 0.0f ? 1 : 0);
        int inc_face_id = inc_axis * 2 + (inc_sign < 0.0f ? 1 : 0);
        for (int i = 0; i < kept; ++i) {
            uint32_t feature_id = (static_cast<uint32_t>(ref_face_id) << 8) |
                                   (static_cast<uint32_t>(inc_face_id) << 4) |
                                   static_cast<uint32_t>(i & 0xF);
            out.add_point(candidates[i].pos, candidates[i].penetration, feature_id);
        }
        return out.count > 0;
    }

    // Edge-edge contact.
    int idx = r.best_axis - 6;
    int ei = idx / 3;
    int ej = idx % 3;

    glm::vec3 a_axes[3];
    a.axes(a_axes[0], a_axes[1], a_axes[2]);
    glm::vec3 b_axes[3];
    b.axes(b_axes[0], b_axes[1], b_axes[2]);

    glm::vec3 d_ab = b.center - a.center;
    int aj = (ei + 1) % 3, ak = (ei + 2) % 3;
    float a_sj = glm::dot(d_ab, a_axes[aj]) >= 0.0f ? 1.0f : -1.0f;
    float a_sk = glm::dot(d_ab, a_axes[ak]) >= 0.0f ? 1.0f : -1.0f;
    glm::vec3 edge_a_center = a.center + a_axes[aj] * (a_sj * a.half_extents[aj]) +
                               a_axes[ak] * (a_sk * a.half_extents[ak]);
    glm::vec3 edge_a_p0 = edge_a_center - a_axes[ei] * a.half_extents[ei];
    glm::vec3 edge_a_p1 = edge_a_center + a_axes[ei] * a.half_extents[ei];

    glm::vec3 d_ba = -d_ab;
    int bj = (ej + 1) % 3, bk = (ej + 2) % 3;
    float b_sj = glm::dot(d_ba, b_axes[bj]) >= 0.0f ? 1.0f : -1.0f;
    float b_sk = glm::dot(d_ba, b_axes[bk]) >= 0.0f ? 1.0f : -1.0f;
    glm::vec3 edge_b_center = b.center + b_axes[bj] * (b_sj * b.half_extents[bj]) +
                               b_axes[bk] * (b_sk * b.half_extents[bk]);
    glm::vec3 edge_b_p0 = edge_b_center - b_axes[ej] * b.half_extents[ej];
    glm::vec3 edge_b_p1 = edge_b_center + b_axes[ej] * b.half_extents[ej];

    glm::vec3 c1, c2;
    geometry::closest_points_segment_segment(edge_a_p0, edge_a_p1, edge_b_p0, edge_b_p1, c1, c2);
    glm::vec3 point = 0.5f * (c1 + c2);

    uint32_t feature_id = 0x1000u | (static_cast<uint32_t>(ei) << 4) | static_cast<uint32_t>(ej);
    out.add_point(point, r.penetration, feature_id);
    return true;
}

} // namespace collision
} // namespace physx
} // namespace coopa

#endif // PHYSXCOOPA_COLLISION_SAT_H
