#pragma once
#include <array>
#include <cmath>
#include <limits>
#include <vector>

struct Vec2
{
    float x{0.f};
    float y{0.f};
    
    constexpr Vec2() = default;
    constexpr Vec2(const float x, const float y) : x{x}, y{y} {}
    
    constexpr Vec2 operator+(const Vec2& rhs) const { return {x + rhs.x, y + rhs.y}; }
    constexpr Vec2 operator-(const Vec2& rhs) const { return {x - rhs.x, y - rhs.y}; }
    constexpr Vec2 operator*(const float scalar) const { return {x * scalar, y * scalar}; }
    constexpr Vec2 operator/(const float scalar) const { return {x / scalar, y / scalar}; }
    Vec2 operator-() const { return {-x, -y}; }
    
    constexpr Vec2& operator+=(const Vec2& rhs) { x += rhs.x; y += rhs.y; return *this; }
    constexpr Vec2& operator-=(const Vec2& rhs) { x -= rhs.x; y -= rhs.y; return *this; }
    constexpr Vec2& operator*=(const float scalar) { x *= scalar; y *= scalar; return *this; }
    
    constexpr float Dot(const Vec2& rhs) const { return x * rhs.x + y * rhs.y; }
    constexpr float Cross(const Vec2& rhs) const { return x * rhs.y - y * rhs.x; }
    static Vec2 ZCross(const float angular_vel, const Vec2& r) { return { -angular_vel * r.y, angular_vel * r.x }; }
    
    float Length() const { return sqrt(x * x + y * y); }
    constexpr float LengthSquared() const { return x * x + y * y; }
    
    Vec2 Normalized() const
    {
        const float len = Length();
        return len > 0.f ? Vec2{x / len, y / len} : Vec2{0.f, 0.f};
    }
    
    // Cartesian Coordinate System (The Y-axis direction is upwards as positive).
    // Counter-Clock-Wise when normal.
    Vec2 Normal() const { return Vec2{-y, x}.Normalized(); }
    
    constexpr Vec2 NormalWithoutNormalized() const { return {-y, x}; }
};

// If contact point is not clipped, incident index is vertex index, reference index is edge index.
// If contact point is result after clip, incident index is edge index, reference index is index of "the endpoint on the clip-face side".
union FeatureId {
    struct {
        uint8_t index_a;
        uint8_t index_b;
        uint8_t type_a; // (0: Vertex, 1: Edge, 2: Circle)
        uint8_t type_b;
    } features;
    uint32_t key = 0;
};

struct Contact
{
    Vec2 collision_point_a;
    Vec2 collision_point_b;
    float depth;
    
    float normal_impulse_sum{0.f};
    float tangent_impulse_sum{0.f};
    float pseudo_impulse_sum{0.f};
    
    float mass_normal{0.f};
    float mass_tangent{0.f};
    float bias{0.f};
    float restitution_velocity{0.f};
    
    FeatureId id;
};

struct Manifold
{
    std::array<Contact, 2> contacts;
    Vec2 normal;
    uint32_t a;
    uint32_t b;
    uint8_t contact_count{0};
    bool is_overlapping{true};
    
    Manifold(const uint32_t a, const uint32_t b) : a(a), b(b){}
};

struct XpbdDistanceConstraint
{
    uint32_t p1;
    uint32_t p2;
    float rest_length;
    float compliance;
    float lambda{0.f};
};

struct XpbdRigidParticleCollisionConstraint
{
    uint32_t particle_idx;
    uint32_t body_idx;
    Vec2 contact_normal;
    Vec2 contact_point;
    float depth;
};

struct XpbdRigidParticleJointConstraint
{
    Vec2 body_local_anchor;
    uint32_t particle_idx;
    uint32_t body_idx;
};

struct XpbdRigidParticleAngularConstraint
{
    Vec2 rest_direction_local;
    Vec2 anchor_local;
    uint32_t particle_idx;
    uint32_t body_idx;
    float compliance = 0.f;
    float lambda = 0.f;
};

struct XpbdTriangleAreaConstraint
{
    uint32_t p1, p2, p3; // Clockwise
    float rest_area;
    float compliance{0.f};
    float lambda{0.f};
};

struct XpbdGlobalAreaConstraint
{
    // Particles arranged clockwise. 
    uint32_t start_particle_idx{0};
    uint16_t particle_count{0};
    float rest_area;
    float compliance{0.f};
    float lambda{0.f};
};

struct XpbdParticleEdgeCollisionConstraint {
    Vec2 normal;
    uint32_t body_a_particle_idx;
    uint32_t body_b_p1_idx;
    uint32_t body_b_p2_idx;
    float t; // The ratio of the projected points on the line segment is [0, 1].
};

struct BodyHandle
{
    uint32_t id;
    bool IsValid() const { return id != 0xFFFFFFFF; }
};

struct SoftBodyHandle
{
    uint32_t start_particle_idx{0};
    uint32_t start_surface_edge_idx{0};
    uint32_t start_surface_particle_idx{0};
    uint32_t start_distance_constraint_idx{0};
    uint16_t distance_constraint_count{0};
    uint16_t surface_particle_count{0};
    uint16_t surface_edge_count{0};
    uint16_t particle_count{0};
    
    bool IsValid() const { return particle_count > 0; }
};

template <typename T, uint8_t Capacity>
struct FixedVector 
{
    std::array<T, Capacity> data{};
    uint8_t sz = 0;

    void push_back(const T& val) 
    {
        data[sz++] = val;
    }
    
    void insert(uint8_t index, const T& val)
    {
        for (uint8_t i = sz; i > index; --i)
        {
            data[i] = data[i - 1];
        }
        data[index] = val;
        ++sz;
    }

    void erase_at(const uint8_t index) 
    {
        for (uint8_t i = index; i < sz - 1; ++i) 
        {
            data[i] = data[i + 1];
        }
        --sz;
    }

    T& operator[](uint8_t i) { return data[i]; }
    const T& operator[](uint8_t i) const { return data[i]; }

    Vec2& back() { return data[sz - 1]; }
    const Vec2& back() const  { return data[sz - 1]; }
    uint8_t size() const { return sz; }
    static constexpr uint8_t capacity() { return Capacity; }
};

using FixedSimplex = FixedVector<Vec2, 3>;
using FixedPolytope = FixedVector<Vec2, 32>;

struct Aabb
{
    float min_x, min_y;
    float max_x, max_y;

    static Aabb Combine(const Aabb& a, const Aabb& b)
    {
        return {
            .min_x = std::min(a.min_x, b.min_x), .min_y = std::min(a.min_y, b.min_y),
            .max_x = std::max(a.max_x, b.max_x), .max_y = std::max(a.max_y, b.max_y)};
    }
    static bool Overlap(const Aabb& a, const Aabb& b)
    {
        return !(a.max_x < b.min_x || a.min_x > b.max_x ||
                 a.max_y < b.min_y || a.min_y > b.max_y);
    }
};

struct BvhNode
{
    Aabb aabb;
    uint32_t left_child{std::numeric_limits<uint32_t>::max()};
    uint32_t right_child{std::numeric_limits<uint32_t>::max()};
    uint32_t leaf_id{std::numeric_limits<uint32_t>::max()};
    
    bool IsLeaf() const { return leaf_id != std::numeric_limits<uint32_t>::max(); }
};

struct Item
{
    uint32_t id;
    uint32_t morton_code;
};

struct QueryOutput
{
    std::vector<std::pair<uint32_t, uint32_t>> rr;
    std::vector<std::pair<uint32_t, uint32_t>> rs;
    std::vector<std::pair<uint32_t, uint32_t>> ss;

    void Clear()
    {
        rr.clear();
        rs.clear();
        ss.clear();
    }
};