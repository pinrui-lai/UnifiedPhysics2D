#pragma once

#include <array>
#include <cmath>
#include <vector>

class PhysicsWorld;

struct Vec2
{
    float x{0.0f};
    float y{0.0f};
    
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
        return len > 0.0f ? Vec2{x / Length(), y / Length()} : Vec2{0.0f, 0.0f};
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
    size_t a;
    size_t b;
    size_t contact_count{0};
    static constexpr float break_distance_sq{0.01f}; // distance threshold square
    bool is_overlapping{true};
    
    Manifold(const size_t a, const size_t b) : a(a), b(b){}
};

struct XpbdDistanceConstraint
{
    size_t p1;
    size_t p2;
    float rest_length;
    float compliance;
    float lambda{0.0f};
};

// Rigid-Particle Collision Constraint
struct XpbdContactConstraint
{
    size_t particle_idx;
    size_t body_idx;
    Vec2 contact_normal;
    Vec2 contact_point;
    float depth;
    float compliance{0.0f};
    float lambda{0.0f};
};

struct XpbdTriangleAreaConstraint
{
    size_t p1, p2, p3; // Clockwise
    float rest_area;
    float compliance{0.0f};
    float lambda{0.0f};
};

struct XpbdGlobalAreaConstraint
{
    // Particles arranged clockwise. 
    size_t start_particle_idx{0};
    size_t particle_count{0};
    float rest_area;
    float compliance{0.0f};
    float lambda{0.0f};
};

struct BodyHandle
{
    size_t id;
    bool IsValid() const { return id != 0xFFFFFFFF; }
};

struct SoftBodyHandle
{
    size_t start_particle_idx{0};
    size_t particle_count{0};
    
    bool IsValid() const { return particle_count > 0; }
};

template <typename T, size_t Capacity>
struct FixedVector 
{
    std::array<T, Capacity> data{};
    size_t sz = 0;

    void push_back(const T& val) 
    {
        data[sz++] = val;
    }
    
    void insert(size_t index, const T& val)
    {
        for (size_t i = sz; i > index; --i)
        {
            data[i] = data[i - 1];
        }
        data[index] = val;
        ++sz;
    }

    void erase_at(const size_t index) 
    {
        for (size_t i = index; i < sz - 1; ++i) 
        {
            data[i] = data[i + 1];
        }
        --sz;
    }

    T& operator[](size_t i) { return data[i]; }
    const T& operator[](size_t i) const { return data[i]; }

    Vec2& back() { return data[sz - 1]; }
    const Vec2& back() const  { return data[sz - 1]; }
    size_t size() const { return sz; }
    static constexpr size_t capacity() { return Capacity; }
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
    size_t left_child{std::numeric_limits<size_t>::max()};
    size_t right_child{std::numeric_limits<size_t>::max()};
    size_t leaf_id{std::numeric_limits<size_t>::max()};
    
    bool IsLeaf() const { return leaf_id != std::numeric_limits<size_t>::max(); }
};

struct Item
{
    size_t id;
    uint32_t morton_code;
};

class MortonBvh
{
public:
    void Build(PhysicsWorld& world);
    void Query(PhysicsWorld& world, size_t query_obj_idx);
    
private:
    std::vector<BvhNode> nodes_;
    std::vector<Aabb> world_aabbs_;
    size_t root_index_{std::numeric_limits<size_t>::max()};
    
    static Aabb ComputeFastAabbFromLocalAabb(const Vec2& pos, float rot, const Aabb& local_aabb);
    static uint32_t ExpandBits(uint32_t bits);
    static uint32_t GetMortonCode(const Vec2& pos, float min_x, float min_y, float max_x, float max_y);
    size_t BuildRecursive(std::pmr::vector<Item>& items, const std::vector<Aabb>& world_aabbs, size_t start, size_t end);
    
    bool CheckRigidParticleCollision(PhysicsWorld& world, size_t query_obj_idx, size_t particle_idx, XpbdContactConstraint& out_constraint);
    static float SdfCircle(Vec2 p, float r) { return p.Length() - r; }
    float SdfBox(Vec2 p, Vec2 b) const;
    Vec2 GetBoxNormal(Vec2 p, Vec2 b);
    friend PhysicsWorld;
};